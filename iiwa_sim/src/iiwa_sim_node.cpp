/**
 * Copyright (C) 2016-2019 Arne Peters - arne.peters@tum.de
 * Technische Universität München
 * Chair for Chair of Robotics, Artificial Intelligence and Real-time Systems
 * Fakultät für Informatik / I16, Boltzmannstraße 3, 85748 Garching bei München, Germany
 * https://www6.in.tum.de
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <iiwa_sim/iiwa_sim_node.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

iiwa_sim::SimNode::SimNode()
: _tfListener(_nh, ros::Duration(30), true),
  _jointStateListener(_nh),
  _moveToJointPositionServer(_nh, "action/move_to_joint_position", false),
  _moveToCartesianPoseServer(_nh, "action/move_to_cartesian_pose", false),
  _moveToCartesianPoseLinServer(_nh, "action/move_to_cartesian_pose_lin", false),
  _moveAlongSplineServer(_nh, "action/move_along_spline", false) {
	_jointNames = {"iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"};

	_jointPositionPub = _nh.advertise<iiwa_msgs::JointPosition>("state/JointPosition", 1);
	_jointVelocityPub = _nh.advertise<iiwa_msgs::JointPosition>("state/JointVelocity", 1);
	_jointTorquePub = _nh.advertise<iiwa_msgs::JointPosition>("state/JointTorque", 1);
	_cartesianPosePub = _nh.advertise<iiwa_msgs::CartesianPose>("state/CartesianPose", 1);

	ROS_INFO("[iiwa_sim] Starting action servers...");

	_moveToJointPositionServer.registerGoalCallback(boost::bind(&SimNode::moveToJointPositionGoalCB, this));
	//_moveToJointPositionServer.registerPreemptCallback(boost::bind(&SimNode::moveToJointPositionPreemptCB, this));
	_moveToJointPositionServer.start();

	_moveToCartesianPoseServer.registerGoalCallback(boost::bind(&SimNode::moveToCartesianPoseGoalCB, this));
	//_moveToCartesianPoseServer.registerPreemptCallback(boost::bind(&SimNode::moveToCartesianPosePreemptCB, this));
	_moveToCartesianPoseServer.start();

	_moveToCartesianPoseLinServer.registerGoalCallback(boost::bind(&SimNode::moveToCartesianPoseLinGoalCB, this));
	//_moveToCartesianPoseLinServer.registerPreemptCallback(boost::bind(&SimNode::moveToCartesianPoseLinPreemptCB, this));
	_moveToCartesianPoseLinServer.start();

	_moveAlongSplineServer.registerGoalCallback(boost::bind(&SimNode::moveAlongSplineGoalCB, this));
	//_moveAlongSplineServer.registerPreemptCallback(boost::bind(&SimNode::moveAlongSplinePreemptCB, this));
	_moveAlongSplineServer.start();

	ROS_INFO("[iiwa_sim] Starting service servers...");

	_setPTPJointLimitsServiceServer = _nh.advertiseService("configuration/setPTPJointLimits", &SimNode::setPTPJointLimitsServiceCB, this);
	_setPTPCartesianLimitsServiceServer = _nh.advertiseService("configuration/setPTPCartesianLimits", &SimNode::setPTPCartesianLimitsServiceCB, this);
	_setEndpointFrameServiceServer = _nh.advertiseService("configuration/setEndpointFrame", &SimNode::setEndpointFrameServiceCB, this);
	_setWorkpieceServiceServer = _nh.advertiseService("configuration/setWorkpiece", &SimNode::setWorkpieceServiceCB, this);

	ROS_INFO("[iiwa_sim] Ready.");
}

// ---------------------------------------------------------------------------------------------------------------------

iiwa_sim::SimNode::~SimNode() {
}

// ---------------------------------------------------------------------------------------------------------------------

void iiwa_sim::SimNode::spin() {
	ros::Rate rate(100);
	while(ros::ok()) {
		ros::spinOnce();
		publishLatestRobotState();
		rate.sleep();
	}
}

// ---------------------------------------------------------------------------------------------------------------------

void iiwa_sim::SimNode::publishLatestRobotState() {
	JointState jointState1 = _jointStateListener.getJointState(_jointNames[0]);

	if (jointState1.header.stamp > _jointPositionMsg.header.stamp) {
		_jointPositionMsg.header = jointState1.header;
		_jointPositionMsg.header.seq = _seq;
		_jointVelocityMsg.header = _jointPositionMsg.header;
		_jointTorqueMsg.header = _jointPositionMsg.header;

		_jointPositionMsg.position.a1 = jointState1.position;
		_jointVelocityMsg.velocity.a1 = jointState1.velocity;
		_jointTorqueMsg.torque.a1 = jointState1.effort;

		JointState jointState2 = _jointStateListener.getJointState(_jointNames[1], jointState1.header.stamp);
		_jointPositionMsg.position.a2 = jointState2.position;
		_jointVelocityMsg.velocity.a2 = jointState2.velocity;
		_jointTorqueMsg.torque.a2 = jointState2.effort;

		JointState jointState3 = _jointStateListener.getJointState(_jointNames[2], jointState1.header.stamp);
		_jointPositionMsg.position.a3 = jointState3.position;
		_jointVelocityMsg.velocity.a3 = jointState3.velocity;
		_jointTorqueMsg.torque.a3 = jointState3.effort;

		JointState jointState4 = _jointStateListener.getJointState(_jointNames[3], jointState1.header.stamp);
		_jointPositionMsg.position.a4 = jointState4.position;
		_jointVelocityMsg.velocity.a4 = jointState4.velocity;
		_jointTorqueMsg.torque.a4 = jointState4.effort;

		JointState jointState5 = _jointStateListener.getJointState(_jointNames[4], jointState1.header.stamp);
		_jointPositionMsg.position.a5 = jointState5.position;
		_jointVelocityMsg.velocity.a5 = jointState5.velocity;
		_jointTorqueMsg.torque.a5 = jointState5.effort;

		JointState jointState6 = _jointStateListener.getJointState(_jointNames[5], jointState1.header.stamp);
		_jointPositionMsg.position.a6 = jointState6.position;
		_jointVelocityMsg.velocity.a6 = jointState6.velocity;
		_jointTorqueMsg.torque.a6 = jointState6.effort;

		JointState jointState7 = _jointStateListener.getJointState(_jointNames[6], jointState1.header.stamp);
		_jointPositionMsg.position.a7 = jointState7.position;
		_jointVelocityMsg.velocity.a7 = jointState7.velocity;
		_jointTorqueMsg.torque.a7 = jointState7.effort;

		_jointPositionPub.publish(_jointPositionMsg);
		_jointVelocityPub.publish(_jointVelocityMsg);
		_jointTorquePub.publish(_jointTorqueMsg);

		publishCartesianPose();
		_seq++;
	}
}

// ---------------------------------------------------------------------------------------------------------------------

void iiwa_sim::SimNode::publishCartesianPose() {
	if (_cartesianPosePub.getNumSubscribers() > 0) {
		tf::StampedTransform transform;
		try {
			_tfListener.lookupTransform(_baseFrame, _eeFrame, ros::Time(0), transform);
			iiwa_msgs::CartesianPose pose;
			pose.poseStamped.header.frame_id = transform.frame_id_;
			pose.poseStamped.header.seq = _seq;
			pose.poseStamped.header.stamp = transform.stamp_;

			pose.poseStamped.pose.position.x = transform.getOrigin().x();
			pose.poseStamped.pose.position.y = transform.getOrigin().y();
			pose.poseStamped.pose.position.z = transform.getOrigin().z();
			pose.poseStamped.pose.orientation.x = transform.getRotation().x();
			pose.poseStamped.pose.orientation.y = transform.getRotation().y();
			pose.poseStamped.pose.orientation.z = transform.getRotation().z();
			pose.poseStamped.pose.orientation.w = transform.getRotation().w();

			pose.redundancy.turn = -1;
			pose.redundancy.status = -1;

			_cartesianPosePub.publish(pose);
		}
		catch (tf::TransformException ex) {
			ROS_WARN_STREAM_THROTTLE(3.0, "[iiwa_sim:" << __LINE__ << "] TF Error: " << ex.what());
		}
	}
}

// ---------------------------------------------------------------------------------------------------------------------

bool iiwa_sim::SimNode::setPTPJointLimitsServiceCB(iiwa_msgs::SetPTPJointSpeedLimits::Request& request, iiwa_msgs::SetPTPJointSpeedLimits::Response& response) {
	response.error = "Speed limits are not available in simulation!";
	response.success = true;

	return true;
}

// ---------------------------------------------------------------------------------------------------------------------

bool iiwa_sim::SimNode::setPTPCartesianLimitsServiceCB(iiwa_msgs::SetPTPCartesianSpeedLimits::Request& request, iiwa_msgs::SetPTPCartesianSpeedLimits::Response& response) {
	response.error = "Speed limits are not available in simulation!";
	response.success = true;
	return true;
}

// ---------------------------------------------------------------------------------------------------------------------

bool iiwa_sim::SimNode::setEndpointFrameServiceCB(iiwa_msgs::SetEndpointFrame::Request& request, iiwa_msgs::SetEndpointFrame::Response& response) {
	_eeFrame = request.frame_id;
	response.success = true;
	return true;
}

// ---------------------------------------------------------------------------------------------------------------------

bool iiwa_sim::SimNode::setWorkpieceServiceCB(iiwa_msgs::SetWorkpiece::Request& request, iiwa_msgs::SetWorkpiece::Response& response) {
	response.success = true;
	return true;
}


tf::Pose iiwa_sim::SimNode::interpolatePoses(const tf::Pose& p1, const tf::Pose& p2, const double w) {
	tf::Pose interpolated;
	interpolated.setOrigin((1.0-w)*p1.getOrigin() + w*p2.getOrigin());
	interpolated.setRotation(p1.getRotation().slerp(p2.getRotation(), w));
	return interpolated;
}

// ---------------------------------------------------------------------------------------------------------------------

std::vector<geometry_msgs::Pose> iiwa_sim::SimNode::interpolateLinear(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2, const double stepSize) {
	tf::Pose t1, t2;
	tf::poseMsgToTF(p1, t1);
	tf::poseMsgToTF(p2, t2);

	tf::Vector3 v = t2.getOrigin() - t1.getOrigin();
	double dist = v.length();
	int steps = dist > 0 ? std::floor(dist / stepSize) + 1 : 1;

	std::vector<geometry_msgs::Pose> waypoints;
	geometry_msgs::Pose waypointMsg;
	tf::Pose waypointTf;

	for (unsigned int i=1; i<=steps; i++) {
		waypointTf = interpolatePoses(t1, t2, ((float)i)/(float)steps);
		tf::poseTFToMsg(waypointTf, waypointMsg);
		waypoints.push_back(waypointMsg);
	}

	return waypoints;
}