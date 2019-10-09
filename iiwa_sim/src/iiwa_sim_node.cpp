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

iiwa_sim::SimNode::SimNode() :
	_moveToJointPositionServer(_nh, "move_to_joint_position", false),
	_moveGroupClient("move_group", false) {

	ROS_INFO("[iiwa_sim] Waiting for action clients...");

	_moveGroupClient.waitForServer();

	ROS_INFO("[iiwa_sim] Starting action servers");

	_moveToJointPositionServer.registerGoalCallback(boost::bind(&SimNode::moveToJointPositionGoalCB, this));
	//_moveToJointPositionServer.registerPreemptCallback(boost::bind(&SimNode::moveToJointPositionPreemptCB, this));
	_moveToJointPositionServer.start();
}

// ---------------------------------------------------------------------------------------------------------------------

iiwa_sim::SimNode::~SimNode() {
}

// ---------------------------------------------------------------------------------------------------------------------

void iiwa_sim::SimNode::spin() {
	ros::spin();
}

// ---------------------------------------------------------------------------------------------------------------------

moveit_msgs::JointConstraint iiwa_sim::SimNode::getJointConstraint(std::string jointName, double angle) {
	moveit_msgs::JointConstraint joint_constraint;
	joint_constraint.joint_name = jointName;
	joint_constraint.position = angle;
	joint_constraint.tolerance_above = 0.0001;
	joint_constraint.tolerance_below = 0.0001;
	joint_constraint.weight = 1.0;
	return joint_constraint;
}

// ---------------------------------------------------------------------------------------------------------------------

void iiwa_sim::SimNode::cancelCurrentGoal() {
	if (_moveToJointPositionServer.isActive()) {
		iiwa_msgs::MoveToJointPositionResult result;
		result.error = "New goal received";
		result.success = false;
		_moveToJointPositionServer.setAborted(result, "New goal received");
	}
}

// ---------------------------------------------------------------------------------------------------------------------

void iiwa_sim::SimNode::moveToJointPositionGoalCB() {
	cancelCurrentGoal();

	iiwa_msgs::MoveToJointPositionGoal::ConstPtr goal = _moveToJointPositionServer.acceptNewGoal();

	moveit_msgs::MoveGroupGoal moveitGoal;
	ros::Time now = ros::Time::now();

	moveitGoal.request.workspace_parameters.header.stamp = now;
	moveitGoal.request.workspace_parameters.header.seq = _moveGroupSeq;
	moveitGoal.request.workspace_parameters.header.frame_id = '/base_link';
	moveitGoal.request.workspace_parameters.min_corner.x = -1.0;
	moveitGoal.request.workspace_parameters.min_corner.y = -1.0;
	moveitGoal.request.workspace_parameters.min_corner.z = -1.0;
	moveitGoal.request.workspace_parameters.max_corner.x = 1.0;
	moveitGoal.request.workspace_parameters.max_corner.y = 1.0;
	moveitGoal.request.workspace_parameters.max_corner.z = 1.0;
	moveitGoal.request.start_state.joint_state.header.seq = _moveGroupSeq;
	moveitGoal.request.start_state.joint_state.header.stamp = now;
	moveitGoal.request.start_state.multi_dof_joint_state.header.seq = _moveGroupSeq;
	moveitGoal.request.start_state.multi_dof_joint_state.header.stamp = now;
	moveitGoal.request.start_state.is_diff = true;

	moveitGoal.request.group_name = _moveGroup;
	moveitGoal.request.num_planning_attempts = _numPlanningAttempts;
	moveitGoal.request.allowed_planning_time = _allowedPlanningTime;
	moveitGoal.request.max_velocity_scaling_factor = _maxVelocityScalingFactor;
	moveitGoal.request.max_acceleration_scaling_factor = _maxAccelerationScalingFactor;

	moveitGoal.planning_options.planning_scene_diff.robot_state.joint_state.header.seq = _moveGroupSeq;
	moveitGoal.planning_options.planning_scene_diff.robot_state.joint_state.header.stamp = now;
	moveitGoal.planning_options.planning_scene_diff.robot_state.multi_dof_joint_state.header.seq = _moveGroupSeq;
	moveitGoal.planning_options.planning_scene_diff.robot_state.multi_dof_joint_state.header.stamp = now;
	moveitGoal.planning_options.planning_scene_diff.robot_state.is_diff = true;
	moveitGoal.planning_options.planning_scene_diff.world.octomap.header.seq = _moveGroupSeq;
	moveitGoal.planning_options.planning_scene_diff.world.octomap.header.stamp = now;
	moveitGoal.planning_options.planning_scene_diff.world.octomap.origin.position.x = 0.0;
	moveitGoal.planning_options.planning_scene_diff.world.octomap.origin.position.y = 0.0;
	moveitGoal.planning_options.planning_scene_diff.world.octomap.origin.position.z = 0.0;
	moveitGoal.planning_options.planning_scene_diff.world.octomap.origin.orientation.x = 0.0;
	moveitGoal.planning_options.planning_scene_diff.world.octomap.origin.orientation.y = 0.0;
	moveitGoal.planning_options.planning_scene_diff.world.octomap.origin.orientation.z = 0.0;
	moveitGoal.planning_options.planning_scene_diff.world.octomap.origin.orientation.w = 0.0;
	moveitGoal.planning_options.planning_scene_diff.world.octomap.octomap.header.seq = _moveGroupSeq;
	moveitGoal.planning_options.planning_scene_diff.world.octomap.octomap.header.stamp = now;
	moveitGoal.planning_options.planning_scene_diff.is_diff = true;
	moveitGoal.planning_options.plan_only = false;
	moveitGoal.planning_options.look_around = false;
	moveitGoal.planning_options.look_around_attempts = 0;
	moveitGoal.planning_options.max_safe_execution_cost = 0.0;
	moveitGoal.planning_options.replan = false;
	moveitGoal.planning_options.replan_attempts = 0;
	moveitGoal.planning_options.replan_delay = 2.0;

	moveit_msgs::Constraints constraints;
	constraints.joint_constraints.push_back(getJointConstraint("iiwa_joint_1", goal->joint_position.position.a1));
	constraints.joint_constraints.push_back(getJointConstraint("iiwa_joint_2", goal->joint_position.position.a2));
	constraints.joint_constraints.push_back(getJointConstraint("iiwa_joint_3", goal->joint_position.position.a3));
	constraints.joint_constraints.push_back(getJointConstraint("iiwa_joint_4", goal->joint_position.position.a4));
	constraints.joint_constraints.push_back(getJointConstraint("iiwa_joint_5", goal->joint_position.position.a5));
	constraints.joint_constraints.push_back(getJointConstraint("iiwa_joint_6", goal->joint_position.position.a6));
	constraints.joint_constraints.push_back(getJointConstraint("iiwa_joint_7", goal->joint_position.position.a7));
	moveitGoal.request.goal_constraints.push_back(constraints);

	_moveGroupSeq++;

	_moveGroupClient.sendGoal(
			moveitGoal,
			boost::bind(&SimNode::moveGroupResultCB, this, _1, _2),
			actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>::SimpleActiveCallback(),
			actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>::SimpleFeedbackCallback()
	);
}

void iiwa_sim::SimNode::moveGroupResultCB(const actionlib::SimpleClientGoalState& state, const moveit_msgs::MoveGroupResultConstPtr& moveitResult) {
	if (_moveToJointPositionServer.isActive()) {
		iiwa_msgs::MoveToJointPositionResult result;

		switch (state.state_) {
			case actionlib::SimpleClientGoalState::SUCCEEDED:
				if (moveitResult->error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
					result.success = false;
					result.error = "Failed with MoveIt! error code "+std::to_string(moveitResult->error_code.val)+": "+state.text_;
				}
				else {
					result.success = true;
				}

				_moveToJointPositionServer.setSucceeded(result, state.text_);
				break;
			case actionlib::SimpleClientGoalState::PREEMPTED:
				result.success = false;
				result.error = state.text_;
				_moveToJointPositionServer.setPreempted(result, state.text_);
				break;
			case actionlib::SimpleClientGoalState::ABORTED:
			case actionlib::SimpleClientGoalState::LOST:
			case actionlib::SimpleClientGoalState::RECALLED:
				result.success = false;
				result.error = state.text_;
				_moveToJointPositionServer.setAborted(result, state.text_);
				break;
			default:
				ROS_ERROR_STREAM("Invalid goal result state: "<<state.state_<<" ("<<state.text_<<")");
				break;
		}
	}
}
