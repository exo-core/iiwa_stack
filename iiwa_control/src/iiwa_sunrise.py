#!/usr/bin/env python2

# Copyright (C) 2016-2019
#
# David Wuthier - daw@mp.aau.dk
# Aalborg University
# Robotics, Vision and Machine Intelligence Laboratory
# Department of Materials and Production
# A. C. Meyers Vaenge 15, 2450 Copenhagen SV, Denmark
# http://rvmi.aau.dk/
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer
#    in the documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
# OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGE.


import rospy

from iiwa_msgs.msg import JointPosition, JointQuantity, CartesianPose, RedundancyInformation
from iiwa_msgs.srv import ConfigureControlMode, ConfigureControlModeRequest, ConfigureControlModeResponse
from iiwa_msgs.srv import SetSmartServoJointSpeedLimits, SetSmartServoJointSpeedLimitsRequest, SetSmartServoJointSpeedLimitsResponse
from iiwa_msgs.srv import SetSmartServoLinSpeedLimits, SetSmartServoLinSpeedLimitsRequest, SetSmartServoLinSpeedLimitsResponse
import numpy as np
from numpy import pi, sqrt, cos, sin, arctan2, array, matrix
from numpy.linalg import norm
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped, WrenchStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from rospy import DEBUG, INFO, logdebug, loginfo, loginfo_throttle, logwarn, logerr
from rospy import Subscriber, Publisher, Service, ServiceProxy
from rospy import ROSException
from rospy import init_node, get_param, spin
from tf.transformations import quaternion_from_matrix, quaternion_matrix
from std_msgs.msg import Float64, Header
from time import clock

def linearlyMap(x, x1, x2, y1, y2):
  return (y2 - y1)/(x2 - x1) * (x - x1) + y1

def R(q):
  return matrix(quaternion_matrix(q)[:3,:3])

def trigonometry(t):
  return (cos(t), sin(t))

def rr(p):
  ty = arctan2(sqrt(p[0,0]**2 + p[1,0]**2), p[2,0])
  tz = arctan2(p[1,0], p[0,0])

  if tz < -pi/2.0:
    ty = -ty
    tz += pi
  elif tz > pi/2.0:
    ty = -ty
    tz -= pi

  return (ty, tz)

def Rz(tz):
  (cz, sz) = trigonometry(tz)
  return matrix([[ cz, -sz, 0.0],
                 [ sz,  cz, 0.0],
                 [0.0, 0.0, 1.0]])

def Ryz(ty, tz):
  (cy, sy) = trigonometry(ty)
  (cz, sz) = trigonometry(tz)
  return matrix([[cy * cz, -sz, sy * cz],
                 [cy * sz, cz, sy * sz],
                 [-sy, 0.0, cy]])

def Hrrt(ty, tz, l):
  (cy, sy) = trigonometry(ty)
  (cz, sz) = trigonometry(tz)
  return matrix([[cy * cz, -sz, sy * cz, 0.0],
                 [cy * sz, cz, sy * sz, 0.0],
                 [-sy, 0.0, cy, l],
                 [0.0, 0.0, 0.0, 1.0]])

def normalize_angle(angle):
  while angle > pi:
    angle -= 2*pi
  while angle < -pi:
    angle += 2*pi
  return angle

class IiwaSunrise(object):
  def __init__(self):
    init_node('iiwa_sunrise', log_level = INFO)

    hardware_interface = get_param('~hardware_interface', 'PositionJointInterface')
    self.robot_name = get_param('~robot_name', 'iiwa')
    model = get_param('~model', 'iiwa14')
    tool_length = get_param('~tool_length', 0.0)
    flange_type = get_param('~flange_type', 'basic')
    self.current_joint_positions = [0,0,0,0,0,0,0]
    self.current_cartesian_pose = Pose()

    if model == 'iiwa7':
      self.l02 = 0.34
      self.l24 = 0.4
    elif model == 'iiwa14':
      self.l02 = 0.36
      self.l24 = 0.42
    else:
      logerr('unknown robot model')
      return

    if flange_type in ['basic', 'electrical', 'pneumatic', 'IO_pneumatic', 'IO_electrical', 'inside_electrical']:
      flange_offset = 0.0
    elif flange_type in ['touch_pneumatic', 'touch_electrical', 'IO_valve_pneumatic']:
      flange_offset = 0.026
    else:
      logerr('unkown flange type')
      return

    self.l46 = 0.4
    self.l6E = 0.126 + tool_length + flange_offset

    self.tr = 0.0
    self.rs = 2.0
    self.v = 1.0

    # For iiwa 7 - A1: 98deg/s, A2: 98deg/s, A3: 100deg/s, A4: 130deg/s, A5: 140deg/s, A6: 180deg/s, A7: 180deg/s
    # For iiwa 14 - A1: 85/s, A2: 85deg/s, A3: 100deg/s, A4: 75deg/s, A5: 130deg/s, A6: 135deg/s, A7: 135deg/s
    self.max_joint_velocities = [np.deg2rad(85),
                                 np.deg2rad(85),
                                 np.deg2rad(100),
                                 np.deg2rad(75),
                                 np.deg2rad(130),
                                 np.deg2rad(135),
                                 np.deg2rad(135)]
    self.relative_joint_velocity = 1.0

    self.joint_names = ['{}_joint_1'.format(self.robot_name),
                        '{}_joint_2'.format(self.robot_name),
                        '{}_joint_3'.format(self.robot_name),
                        '{}_joint_4'.format(self.robot_name),
                        '{}_joint_5'.format(self.robot_name),
                        '{}_joint_6'.format(self.robot_name),
                        '{}_joint_7'.format(self.robot_name)]

    joint_states_sub = Subscriber('joint_states', JointState, self.jointStatesCb, queue_size = 1)
    command_pose_sub = Subscriber('command/CartesianPose', CartesianPose, self.commandPoseCb, queue_size = 1)
    command_pose_lin_sub = Subscriber('command/CartesianPoseLin', CartesianPose, self.commandPoseLinCb, queue_size = 1)
    redundancy_sub = Subscriber('command/redundancy', RedundancyInformation, self.redundancyCb, queue_size = 1)
    joint_position_sub = Subscriber('command/JointPosition', JointPosition, self.jointPositionCb, queue_size = 1)

    self.state_pose_pub = Publisher('state/CartesianPose', CartesianPose, queue_size = 1)
    self.joint_position_pub = Publisher('state/JointPosition', JointPosition, queue_size = 1)
    self.joint_trajectory_pub = Publisher(
        '{}_trajectory_controller/command'.format(hardware_interface), JointTrajectory, queue_size = 1)

    path_parameters_configuration_srv = Service(
        'configuration/setSmartServoLimits', SetSmartServoJointSpeedLimits, self.handlePathParametersConfiguration)
    path_parameters_lin_configuration_srv = Service(
        'configuration/setSmartServoLinLimits', SetSmartServoLinSpeedLimits, self.handlePathParametersLinConfiguration)
    smart_servo_configuration_srv = Service(
        'configuration/ConfigureControlMode', ConfigureControlMode, self.handleSmartServoConfiguration)

    spin()

  def jointPositionCb(self, msg):
    target_angles = [msg.position.a1, msg.position.a2, msg.position.a3, msg.position.a4, msg.position.a5, msg.position.a6, msg.position.a7]
    self.publishJointPositionCommand(target_angles, self.getJointMotionTime(target_angles, self.current_joint_positions))

  def handleSmartServoConfiguration(self, request):
    return ConfigureControlModeResponse(True, '')

  def handlePathParametersConfiguration(self, request):
    loginfo('setting relative joint velocity to '+str(request.joint_relative_velocity))

    if request.joint_relative_velocity >= 0.0 and request.joint_relative_velocity <= 1.0:
      self.relative_joint_velocity = request.joint_relative_velocity
      return SetSmartServoJointSpeedLimitsResponse(True, '')
    else:
      return SetSmartServoJointSpeedLimitsResponse(False, '')

  def handlePathParametersLinConfiguration(self, request):
    loginfo('setting path parameters linear')

    v = request.max_cartesian_velocity.linear.x

    if v >= 0.0 and v <= 1000.0:
      self.v = linearlyMap(v, 0.0, 1000.0, 2.0, 0.5)
      return SetSmartServoLinSpeedLimitsResponse(True, '')
    else:
      return SetSmartServoLinSpeedLimitsResponse(False, '')

  def redundancyCb(self, msg):
    if not (msg.status == -1 or msg.turn == -1):
      self.rs = msg.status 

    self.tr = msg.e1

  def jointStatesCb(self, msg):
    if len(msg.name) != 7 or msg.name[0] != '{}_joint_1'.format(self.robot_name):
      return

    self.current_joint_positions = msg.position

    rs = 0
    rs += 1 if self.current_joint_positions[1] < 0 else 0
    rs += 2 if self.current_joint_positions[3] < 0 else 0
    rs += 4 if self.current_joint_positions[5] < 0 else 0

    self.rs = rs

    H02 = Hrrt(self.current_joint_positions[1], self.current_joint_positions[0], self.l02)
    H24 = Hrrt(-self.current_joint_positions[3], self.current_joint_positions[2], self.l24)
    H46 = Hrrt(self.current_joint_positions[5], self.current_joint_positions[4], self.l46)
    H6E = Hrrt(0.0, self.current_joint_positions[6], self.l6E)

    H0E = H02 * H24 * H46 * H6E
    q0E = quaternion_from_matrix(H0E)

    self.current_cartesian_pose = Pose(
      position = Point(x = H0E[0,3], y = H0E[1,3], z = H0E[2,3]),
      orientation = Quaternion(x = q0E[0], y = q0E[1], z = q0E[2], w = q0E[3]))

    self.state_pose_pub.publish(
      CartesianPose(
        poseStamped = PoseStamped(
          header = Header(
            frame_id = '{}_link_0'.format(self.robot_name)),
          pose = self.current_cartesian_pose),
        redundancy = RedundancyInformation(
          e1 = self.tr)))

    self.joint_position_pub.publish(
      JointPosition(header = Header(
        frame_id = '{}_link_0'.format(self.robot_name)),
      position = JointQuantity(
       a1 = self.current_joint_positions[0],
       a2 = self.current_joint_positions[1],
       a3 = self.current_joint_positions[2],
       a4 = self.current_joint_positions[3],
       a5 = self.current_joint_positions[4],
       a6 = self.current_joint_positions[5],
       a7 = self.current_joint_positions[6],
    )))

  def commandPoseCb(self, msg):
    T0 = clock()
    self.redundancyCb(msg.redundancy)

    t = 7 * [0.0]
    pE0 = matrix([[msg.poseStamped.pose.position.x],
                  [msg.poseStamped.pose.position.y],
                  [msg.poseStamped.pose.position.z]])
    qE0 = array([msg.poseStamped.pose.orientation.x,
                 msg.poseStamped.pose.orientation.y,
                 msg.poseStamped.pose.orientation.z,
                 msg.poseStamped.pose.orientation.w])

    rs = self.rs
    rs2 = - np.sign((rs & (1 << 0))-0.5)
    rs4 = - np.sign((rs & (1 << 1))-0.5)
    rs6 = - np.sign((rs & (1 << 2))-0.5)

    pE6 = matrix([[0.0], [0.0], [self.l6E]])
    p20 = matrix([[0.0], [0.0], [self.l02]])

    RE0 = R(qE0)
    p6E0 = RE0 * pE6
    p60 = pE0 - p6E0
    p260 = p60 - p20

    s = norm(p260)

    if s > self.l24 + self.l46:
      logwarn('invalid pose command')
      return

    t[3] = rs4 * (np.pi - np.arccos((self.l24**2 + self.l46**2 - s**2)/(2*self.l24 * self.l46)))
    t[2] = self.tr

    x = -cos(t[2])*sin(t[3]) * self.l46
    y = -sin(t[2])*sin(t[3]) * self.l46
    z = cos(t[3]) * self.l46 + self.l24
    xz = (x**2 + z **2) ** 0.5

    z_des = p260[2].item()

    t[1] = np.arccos(z_des / xz) - np.arctan2(x, z)
    if np.sign(t[1]) != rs2:
      t[1] = - np.arccos(z_des / xz) - np.arctan2(x, z)
    if np.sign(t[1]) != rs2:
      logwarn('Joint 2 has no solution for required {} sign.'.format('negative' if rs2 == -1 else 'positive'))
      return

    x = self.l24*sin(t[1]) + (cos(t[3])*sin(t[1]) - (cos(t[1])*cos(t[2]))*sin(t[3]))*self.l46

    x_des = p260[0].item()
    y_des = p260[1].item()

    t[0] = normalize_angle(np.arctan2(y_des, x_des) - np.arctan2(y,x))

    R20 = Ryz(t[1], t[0])
    R42 = Ryz(-t[3], t[2])
    R40 = R20 * R42
    p6E4 = R40.T * p6E0
    (t[5], t[4]) = rr(p6E4)
    if np.sign(t[5]) != rs6:
      t[4] = normalize_angle(t[4] + pi)
      t[5] = -t[5]

    R64 = Ryz(t[5], t[4])
    R60 = R40 * R64
    RE6 = R60.T * RE0
    t[6] = arctan2(RE6[1,0], RE6[0,0])

    self.publishJointPositionCommand(t, self.getCartesianMotionTime(msg.poseStamped.pose, self.current_cartesian_pose))

    logdebug('timing: %s ms', 1.0e3 * (clock() - T0))

  def commandPoseLinCb(self, msg):
    self.commandPoseCb(msg)

  def publishJointPositionCommand(self, trajectory, duration):
    jtp = JointTrajectoryPoint()
    jtp.positions = trajectory
    jtp.time_from_start = duration
    jt = JointTrajectory()
    jt.joint_names = self.joint_names
    jt.points.append(jtp)
    self.joint_trajectory_pub.publish(jt)

  def getCartesianMotionTime(self, current, target):
    return rospy.Duration.from_sec(self.v)

  def getJointMotionTime(self, current, target):
    slowest_joint_time = 0

    for c, t, v in zip(current, target, self.max_joint_velocities):
      motion_time = abs(t - c) / (v * self.relative_joint_velocity)
      if motion_time > slowest_joint_time:
        slowest_joint_time = motion_time

    return rospy.Duration.from_sec(motion_time)

if __name__ == "__main__":
  ik = IiwaSunrise()
