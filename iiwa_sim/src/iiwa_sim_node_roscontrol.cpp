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
	ros::Publisher _commandJointPositionPub = _nh.advertise<iiwa_msgs::JointPosition>("command/JointPosition", 1);
	ros::Publisher _commandCartesianPosePub = _nh.advertise<iiwa_msgs::CartesianPose>("command/CartesianPose", 1);
	ros::Publisher _commandCartesianPoseLinPub = _nh.advertise<iiwa_msgs::CartesianPose>("command/CartesianPoseLin", 1);
}

// ---------------------------------------------------------------------------------------------------------------------

iiwa_sim::SimNodeRoscontrol::~SimNodeRoscontrol() {

}

// ---------------------------------------------------------------------------------------------------------------------

void iiwa_sim::SimNodeRoscontrol::moveToJointPositionGoalCB() {
	cancelCurrentGoal();

	iiwa_msgs::MoveToJointPositionGoal::ConstPtr goal = _moveToJointPositionServer.acceptNewGoal();
	_commandJointPositionPub.publish(goal->joint_position);
}

// ---------------------------------------------------------------------------------------------------------------------

void iiwa_sim::SimNodeRoscontrol::moveToCartesianPoseGoalCB() {
	cancelCurrentGoal();

	iiwa_msgs::MoveToCartesianPoseGoal::ConstPtr goal = _moveToCartesianPoseServer.acceptNewGoal();
	_commandCartesianPosePub.publish(goal->cartesian_pose);
}

// ---------------------------------------------------------------------------------------------------------------------

void iiwa_sim::SimNodeRoscontrol::moveToCartesianPoseLinGoalCB() {
	cancelCurrentGoal();

	iiwa_msgs::MoveToCartesianPoseGoal::ConstPtr goal = _moveToCartesianPoseLinServer.acceptNewGoal();
	_commandCartesianPoseLinPub.publish(goal->cartesian_pose);
}

// ---------------------------------------------------------------------------------------------------------------------

void iiwa_sim::SimNodeRoscontrol::moveAlongSplineGoalCB() {

}

// ---------------------------------------------------------------------------------------------------------------------

void iiwa_sim::SimNodeRoscontrol::cancelCurrentGoal() {

}

// ---------------------------------------------------------------------------------------------------------------------

bool iiwa_sim::SimNodeRoscontrol::setPTPJointLimitsServiceCB(iiwa_msgs::SetPTPJointSpeedLimits::Request& request, iiwa_msgs::SetPTPJointSpeedLimits::Response& response) {

}
