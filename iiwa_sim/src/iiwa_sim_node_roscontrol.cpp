/**
 * Copyright (C) 2016-2019 Arne Peters - arne.peters@tum.de
 * Technische Universität München
 * Chair for Chair of Robotics, Artificial Intelligence and Real-time Systems
 * Fakultät für Informatik / I6, Boltzmannstraße 3, 85748 Garching bei München, Germany
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

#include <iiwa_sim/iiwa_sim_node_roscontrol.h>

iiwa_sim::SimNodeRoscontrol::SimNodeRoscontrol()
: iiwa_sim::SimNode::SimNode() {
	_commandJointPositionPub = _nh.advertise<iiwa_msgs::JointPosition>("command/JointPosition", 1);
	_commandCartesianPosePub = _nh.advertise<iiwa_msgs::CartesianPose>("command/CartesianPose", 1);
	_commandCartesianPoseLinPub = _nh.advertise<iiwa_msgs::CartesianPose>("command/CartesianPoseLin", 1);

	_jointPositionSub = _nh.subscribe("state/JointPosition", 2, &iiwa_sim::SimNodeRoscontrol::jointPositionCallback, this);
	_cartesianPoseSub = _nh.subscribe("state/CartesianPose", 2, &iiwa_sim::SimNodeRoscontrol::cartesianPoseCallback, this);
}

// ---------------------------------------------------------------------------------------------------------------------

iiwa_sim::SimNodeRoscontrol::~SimNodeRoscontrol() {
}

// ---------------------------------------------------------------------------------------------------------------------

void iiwa_sim::SimNodeRoscontrol::spin() {
	ros::spin();
}

// ---------------------------------------------------------------------------------------------------------------------

void iiwa_sim::SimNodeRoscontrol::moveToJointPositionGoalCB() {
	ROS_INFO_STREAM("[SimNodeRoscontrol] Received new joint goal");

	cancelCurrentGoal();

	_currentGoal.type = JOINT_POSITION;
	_currentGoal.moveToJointPositionGoal = _moveToJointPositionServer.acceptNewGoal();
	_commandJointPositionPub.publish(_currentGoal.moveToJointPositionGoal->joint_position);
}

// ---------------------------------------------------------------------------------------------------------------------

void iiwa_sim::SimNodeRoscontrol::moveToCartesianPoseGoalCB() {
	ROS_DEBUG_STREAM("[SimNodeRoscontrol] Received new Cartesian goal");
	cancelCurrentGoal();

	_currentGoal.type = CARTESIAN_POSE;
	_currentGoal.moveToCartesianPoseGoal = _moveToCartesianPoseServer.acceptNewGoal();
	_commandCartesianPosePub.publish(_currentGoal.moveToCartesianPoseGoal->cartesian_pose);
}

// ---------------------------------------------------------------------------------------------------------------------

void iiwa_sim::SimNodeRoscontrol::moveToCartesianPoseLinGoalCB() {
	ROS_DEBUG_STREAM("[SimNodeRoscontrol] Received new Cartesian lin goal");
	cancelCurrentGoal();

	_currentGoal.type = CARTESIAN_POSE_LIN;
	_currentGoal.moveToCartesianPoseLinGoal = _moveToCartesianPoseLinServer.acceptNewGoal();
	_commandCartesianPoseLinPub.publish(_currentGoal.moveToCartesianPoseLinGoal->cartesian_pose);
}

// ---------------------------------------------------------------------------------------------------------------------

void iiwa_sim::SimNodeRoscontrol::moveAlongSplineGoalCB() {
	cancelCurrentGoal();

	_currentGoal.type = CARTESIAN_SPLINE;
	_currentGoal.moveAlongSplineGoal = _moveAlongSplineServer.acceptNewGoal();
	_currentGoal.splineIndex = 0;

	if (_currentGoal.moveAlongSplineGoal->spline.segments.size() < 1) {
		// Spline is empty
		markActionDone<iiwa_msgs::MoveAlongSplineAction, iiwa_msgs::MoveAlongSplineResult>(_moveAlongSplineServer);
	}
	else {
		_commandCartesianPoseLinPub.publish(_currentGoal.moveAlongSplineGoal->spline.segments[_currentGoal.splineIndex].point);
	}
}

// ---------------------------------------------------------------------------------------------------------------------

void iiwa_sim::SimNodeRoscontrol::cancelCurrentGoal() {
	if (_currentGoal.type == NONE) {
		return;
	}

	_commandJointPositionPub.publish(getCurrentJointPosition());

	switch (_currentGoal.type) {
		case JOINT_POSITION: {
			markActionDone<iiwa_msgs::MoveToJointPositionAction, iiwa_msgs::MoveToJointPositionResult>(_moveToJointPositionServer, false, "Received new goal");
			break;
		}
		case CARTESIAN_POSE: {
			markActionDone<iiwa_msgs::MoveToCartesianPoseAction, iiwa_msgs::MoveToCartesianPoseResult>(_moveToCartesianPoseServer, false, "Received new goal");
			break;
		}
		case CARTESIAN_POSE_LIN: {
			markActionDone<iiwa_msgs::MoveToCartesianPoseAction, iiwa_msgs::MoveToCartesianPoseResult>(_moveToCartesianPoseLinServer, false, "Received new goal");
			break;
		}
		case CARTESIAN_SPLINE: {
			markActionDone<iiwa_msgs::MoveAlongSplineAction, iiwa_msgs::MoveAlongSplineResult>(_moveAlongSplineServer, false, "Received new goal");
			break;
		}
		case NONE: {
			break;
		}
		default: {
			ROS_ERROR_STREAM("[SimNodeRoscontrol] Unknown goal state: "<<_currentGoal.type);
		}
	}

	_currentGoal.type = NONE;
}

// ---------------------------------------------------------------------------------------------------------------------

bool iiwa_sim::SimNodeRoscontrol::setPTPJointLimitsServiceCB(iiwa_msgs::SetPTPJointSpeedLimits::Request& request, iiwa_msgs::SetPTPJointSpeedLimits::Response& response) {

}

// ---------------------------------------------------------------------------------------------------------------------

void iiwa_sim::SimNodeRoscontrol::jointPositionCallback(const iiwa_msgs::JointPosition::ConstPtr& msg) {
	if (_currentGoal.type == JOINT_POSITION) {
		if (goalReached(*msg, _currentGoal.moveToJointPositionGoal->joint_position)) {
			markActionDone<iiwa_msgs::MoveToJointPositionAction, iiwa_msgs::MoveToJointPositionResult>(_moveToJointPositionServer);
		}
	}
}

// ---------------------------------------------------------------------------------------------------------------------

void iiwa_sim::SimNodeRoscontrol::cartesianPoseCallback(const iiwa_msgs::CartesianPose::ConstPtr& msg) {
	switch (_currentGoal.type) {
		case CARTESIAN_POSE: {
			if (goalReached(*msg, _currentGoal.moveToCartesianPoseGoal->cartesian_pose)) {
				markActionDone<iiwa_msgs::MoveToCartesianPoseAction, iiwa_msgs::MoveToCartesianPoseResult>(_moveToCartesianPoseServer);
			}
			break;
		}
		case CARTESIAN_POSE_LIN: {
			if (goalReached(*msg, _currentGoal.moveToCartesianPoseLinGoal->cartesian_pose)) {
				markActionDone<iiwa_msgs::MoveToCartesianPoseAction, iiwa_msgs::MoveToCartesianPoseResult>(_moveToCartesianPoseLinServer);
			}
			break;
		}
		case CARTESIAN_SPLINE: {
			if (goalReached(*msg, _currentGoal.moveAlongSplineGoal->spline.segments[_currentGoal.splineIndex].point)) {
				_currentGoal.splineIndex++;

				if (_currentGoal.splineIndex == _currentGoal.moveAlongSplineGoal->spline.segments.size()) {
					markActionDone<iiwa_msgs::MoveToCartesianPoseAction, iiwa_msgs::MoveToCartesianPoseResult>(_moveToCartesianPoseLinServer);
				}
				else {
					_commandCartesianPoseLinPub.publish(_currentGoal.moveAlongSplineGoal->spline.segments[_currentGoal.splineIndex].point);
				}
			}
			break;
		}
	}
}

// ---------------------------------------------------------------------------------------------------------------------

iiwa_msgs::JointPosition iiwa_sim::SimNodeRoscontrol::getCurrentJointPosition() const {
	iiwa_msgs::JointPosition currentPosition;

	currentPosition.position.a1 = _jointStateListener.getJointState(_jointNames[0]).position;
	currentPosition.position.a2 = _jointStateListener.getJointState(_jointNames[1]).position;
	currentPosition.position.a3 = _jointStateListener.getJointState(_jointNames[2]).position;
	currentPosition.position.a4 = _jointStateListener.getJointState(_jointNames[3]).position;
	currentPosition.position.a5 = _jointStateListener.getJointState(_jointNames[4]).position;
	currentPosition.position.a6 = _jointStateListener.getJointState(_jointNames[5]).position;
	currentPosition.position.a7 = _jointStateListener.getJointState(_jointNames[6]).position;

	return currentPosition;
}

// ---------------------------------------------------------------------------------------------------------------------

bool iiwa_sim::SimNodeRoscontrol::goalReached(const iiwa_msgs::JointPosition& currentPosition, const iiwa_msgs::JointPosition& targetPosition) const {
	//ROS_INFO("Checking for goal reached...");
	if (
			std::fabs(currentPosition.position.a1 - targetPosition.position.a1) <= _jointAngleConstraintTolerance &&
			std::fabs(currentPosition.position.a2 - targetPosition.position.a2) <= _jointAngleConstraintTolerance &&
			std::fabs(currentPosition.position.a3 - targetPosition.position.a3) <= _jointAngleConstraintTolerance &&
			std::fabs(currentPosition.position.a4 - targetPosition.position.a4) <= _jointAngleConstraintTolerance &&
			std::fabs(currentPosition.position.a5 - targetPosition.position.a5) <= _jointAngleConstraintTolerance &&
			std::fabs(currentPosition.position.a6 - targetPosition.position.a6) <= _jointAngleConstraintTolerance &&
			std::fabs(currentPosition.position.a7 - targetPosition.position.a7) <= _jointAngleConstraintTolerance
	) {
		return true;
	}

	return false;
}

// ---------------------------------------------------------------------------------------------------------------------

bool iiwa_sim::SimNodeRoscontrol::goalReached(const iiwa_msgs::CartesianPose& currentPose, const iiwa_msgs::CartesianPose& targetPose) const {
	tf::Quaternion currentOrientation, targetOrientation;
	tf::quaternionMsgToTF(currentPose.poseStamped.pose.orientation, currentOrientation);
	tf::quaternionMsgToTF(targetPose.poseStamped.pose.orientation, targetOrientation);

	tf::Vector3 currentPosition, targetPosition;
	tf::pointMsgToTF(currentPose.poseStamped.pose.position, currentPosition);
	tf::pointMsgToTF(targetPose.poseStamped.pose.position, targetPosition);

	if (
		(currentPosition - targetPosition).length() < _positionConstraintTolerance &&
		currentOrientation.angleShortestPath(targetOrientation) < _orientationConstraintTolerance &&
		std::fabs(currentPose.redundancy.e1 - targetPose.redundancy.e1) < _redundancyAngleTolerance
	) {
		return true;
	}

	return false;
}
