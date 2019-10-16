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

iiwa_sim::SimNode::SimNode() :
	_tfListener(_nh),
	_jointStateListener(_nh),
	_moveToJointPositionServer(_nh, "move_to_joint_position", false),
	_moveToCartesianPoseServer(_nh, "move_to_cartesian_pose", false),
	_moveToCartesianPoseLinServer(_nh, "move_to_cartesian_pose_lin", false),
	_moveGroupClient("move_group", true) {

	_moveGroup = _nh.param<std::string>("move_group", _moveGroup);
	_numPlanningAttempts = _nh.param<double>("num_planning_attempts", _numPlanningAttempts);
	_allowedPlanningTime = _nh.param<double>("allowed_planning_time", _allowedPlanningTime);
	_maxVelocityScalingFactor = _nh.param<double>("max_velocity_scaling_factor", _maxVelocityScalingFactor);
	_maxAccelerationScalingFactor = _nh.param<double>("max_acceleration_scaling_factor", _maxAccelerationScalingFactor);

	ROS_INFO("[iiwa_sim] Waiting for action clients...");

	_moveGroupClient.waitForServer();

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

	ROS_INFO("[iiwa_sim] Ready.");
}

// ---------------------------------------------------------------------------------------------------------------------

iiwa_sim::SimNode::~SimNode() {
}

// ---------------------------------------------------------------------------------------------------------------------

void iiwa_sim::SimNode::spin() {
	ros::spin();
}

// ---------------------------------------------------------------------------------------------------------------------

moveit_msgs::JointConstraint iiwa_sim::SimNode::getJointConstraint(const std::string& jointName, const double angle) const{
	moveit_msgs::JointConstraint jointConstraint;
	jointConstraint.joint_name = jointName;
	jointConstraint.position = angle;
	jointConstraint.tolerance_above = 0.0001;
	jointConstraint.tolerance_below = 0.0001;
	jointConstraint.weight = 1.0;
	return jointConstraint;
}

// ---------------------------------------------------------------------------------------------------------------------

moveit_msgs::PositionConstraint iiwa_sim::SimNode::getPositionConstraint(const geometry_msgs::PoseStamped& pose) const {
	moveit_msgs::PositionConstraint positionConstraint;
	positionConstraint.header = pose.header;
	positionConstraint.link_name = _eeFrame;

	positionConstraint.target_point_offset.x = 0.0;
	positionConstraint.target_point_offset.y = 0.0;
	positionConstraint.target_point_offset.z = 0.0;

	shape_msgs::SolidPrimitive sphere;
	sphere.type = shape_msgs::SolidPrimitive::SPHERE;
	sphere.dimensions.push_back(0.001);

	positionConstraint.constraint_region.primitives.push_back(sphere);
	positionConstraint.constraint_region.primitive_poses.push_back(pose.pose);

	positionConstraint.weight = 1.0;

	return positionConstraint;
}

// ---------------------------------------------------------------------------------------------------------------------

moveit_msgs::OrientationConstraint iiwa_sim::SimNode::getOrientationConstraint(const geometry_msgs::PoseStamped& pose) const {
	moveit_msgs::OrientationConstraint orientationConstraint;
	orientationConstraint.header = pose.header;
	orientationConstraint.link_name = "iiwa_link_ee";

	orientationConstraint.orientation = pose.pose.orientation;

	orientationConstraint.absolute_x_axis_tolerance = 0.01;
	orientationConstraint.absolute_y_axis_tolerance = 0.01;
	orientationConstraint.absolute_z_axis_tolerance = 0.01;

	orientationConstraint.weight = 1.0;

	return orientationConstraint;
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

	moveit_msgs::MoveGroupGoal moveitGoal = toMoveGroupGoal(goal);

	_moveGroupClient.sendGoal(
			moveitGoal,
			boost::bind(&SimNode::moveGroupResultCB, this, _1, _2),
			actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>::SimpleActiveCallback(),
			actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>::SimpleFeedbackCallback()
	);
}

// ---------------------------------------------------------------------------------------------------------------------

void iiwa_sim::SimNode::moveToCartesianPoseGoalCB() {
	cancelCurrentGoal();

	iiwa_msgs::MoveToCartesianPoseGoal::ConstPtr goal = _moveToCartesianPoseServer.acceptNewGoal();

	moveit_msgs::MoveGroupGoal moveitGoal = toMoveGroupGoal(goal);

	_moveGroupClient.sendGoal(
			moveitGoal,
			boost::bind(&SimNode::moveGroupResultCB, this, _1, _2),
			actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>::SimpleActiveCallback(),
			actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>::SimpleFeedbackCallback()
	);
}

// ---------------------------------------------------------------------------------------------------------------------

void iiwa_sim::SimNode::moveToCartesianPoseLinGoalCB() {
	cancelCurrentGoal();

	iiwa_msgs::MoveToCartesianPoseGoal::ConstPtr goal = _moveToCartesianPoseLinServer.acceptNewGoal();

	if (!_tfListener.waitForTransform(_eeFrame, goal->cartesian_pose.poseStamped.header.frame_id, goal->cartesian_pose.poseStamped.header.stamp, ros::Duration(1.0))) {
		iiwa_msgs::MoveToCartesianPoseResult result;
		result.error = "No transformation found from "+goal->cartesian_pose.poseStamped.header.frame_id+" to "+_eeFrame;
		result.success = false;
		ROS_ERROR(result.error.c_str());
		_moveToCartesianPoseLinServer.setSucceeded(result, result.error);
		return;
	}

	tf::StampedTransform startTransform;

	try {
		_tfListener.lookupTransform(_eeFrame, goal->cartesian_pose.poseStamped.header.frame_id,
									goal->cartesian_pose.poseStamped.header.stamp, startTransform);
	}
	catch (tf::TransformException ex) {
		iiwa_msgs::MoveToCartesianPoseResult result;
		result.error = "tf::TransformException: "+std::string(ex.what());
		result.success = false;
		ROS_ERROR(result.error.c_str());
		_moveToCartesianPoseLinServer.setSucceeded(result, result.error);
		return;
	}

	geometry_msgs::PoseStamped startPose;
	startPose.header = goal->cartesian_pose.poseStamped.header;
	startPose.pose.position.x = startTransform.getOrigin().x();
	startPose.pose.position.y = startTransform.getOrigin().y();
	startPose.pose.position.z = startTransform.getOrigin().z();
	startPose.pose.orientation.w = startTransform.getRotation().w();
	startPose.pose.orientation.x = startTransform.getRotation().x();
	startPose.pose.orientation.y = startTransform.getRotation().y();
	startPose.pose.orientation.z = startTransform.getRotation().z();

	moveit_msgs::MoveGroupGoal moveitGoal = toMoveGroupGoal(goal, startPose);

	_moveGroupClient.sendGoal(
			moveitGoal,
			boost::bind(&SimNode::moveGroupResultCB, this, _1, _2),
			actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>::SimpleActiveCallback(),
			actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>::SimpleFeedbackCallback()
	);
}

// ---------------------------------------------------------------------------------------------------------------------

moveit_msgs::MoveGroupGoal iiwa_sim::SimNode::getBlankMoveItGoal(const std_msgs::Header& header) const {
	moveit_msgs::MoveGroupGoal moveitGoal;

	moveitGoal.request.workspace_parameters.header = header;
	moveitGoal.request.workspace_parameters.min_corner.x = -1.0;
	moveitGoal.request.workspace_parameters.min_corner.y = -1.0;
	moveitGoal.request.workspace_parameters.min_corner.z = -1.0;
	moveitGoal.request.workspace_parameters.max_corner.x = 1.0;
	moveitGoal.request.workspace_parameters.max_corner.y = 1.0;
	moveitGoal.request.workspace_parameters.max_corner.z = 1.0;
	moveitGoal.request.start_state.joint_state.header = header;
	moveitGoal.request.start_state.multi_dof_joint_state.header = header;
	moveitGoal.request.start_state.is_diff = true;

	moveitGoal.request.group_name = _moveGroup;
	moveitGoal.request.num_planning_attempts = _numPlanningAttempts;
	moveitGoal.request.allowed_planning_time = _allowedPlanningTime;
	moveitGoal.request.max_velocity_scaling_factor = _maxVelocityScalingFactor;
	moveitGoal.request.max_acceleration_scaling_factor = _maxAccelerationScalingFactor;

	moveitGoal.planning_options.planning_scene_diff.robot_state.joint_state.header = header;
	moveitGoal.planning_options.planning_scene_diff.robot_state.multi_dof_joint_state.header = header;
	moveitGoal.planning_options.planning_scene_diff.robot_state.is_diff = true;
	moveitGoal.planning_options.planning_scene_diff.world.octomap.header = header;
	moveitGoal.planning_options.planning_scene_diff.world.octomap.origin.position.x = 0.0;
	moveitGoal.planning_options.planning_scene_diff.world.octomap.origin.position.y = 0.0;
	moveitGoal.planning_options.planning_scene_diff.world.octomap.origin.position.z = 0.0;
	moveitGoal.planning_options.planning_scene_diff.world.octomap.origin.orientation.x = 0.0;
	moveitGoal.planning_options.planning_scene_diff.world.octomap.origin.orientation.y = 0.0;
	moveitGoal.planning_options.planning_scene_diff.world.octomap.origin.orientation.z = 0.0;
	moveitGoal.planning_options.planning_scene_diff.world.octomap.origin.orientation.w = 0.0;
	moveitGoal.planning_options.planning_scene_diff.world.octomap.octomap.header = header;
	moveitGoal.planning_options.planning_scene_diff.is_diff = true;
	moveitGoal.planning_options.plan_only = false;
	moveitGoal.planning_options.look_around = false;
	moveitGoal.planning_options.look_around_attempts = 0;
	moveitGoal.planning_options.max_safe_execution_cost = 0.0;
	moveitGoal.planning_options.replan = false;
	moveitGoal.planning_options.replan_attempts = 0;
	moveitGoal.planning_options.replan_delay = 2.0;

	return moveitGoal;
}

// ---------------------------------------------------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------------------------------------------------

moveit_msgs::MoveGroupGoal iiwa_sim::SimNode::toMoveGroupGoal(const iiwa_msgs::MoveToJointPositionGoal::ConstPtr goal) {
	std_msgs::Header header = goal->joint_position.header;
	header.seq = _moveGroupSeq++;

	moveit_msgs::MoveGroupGoal moveitGoal = getBlankMoveItGoal(header);

	moveit_msgs::Constraints constraints;
	constraints.joint_constraints.push_back(getJointConstraint("iiwa_joint_1", goal->joint_position.position.a1));
	constraints.joint_constraints.push_back(getJointConstraint("iiwa_joint_2", goal->joint_position.position.a2));
	constraints.joint_constraints.push_back(getJointConstraint("iiwa_joint_3", goal->joint_position.position.a3));
	constraints.joint_constraints.push_back(getJointConstraint("iiwa_joint_4", goal->joint_position.position.a4));
	constraints.joint_constraints.push_back(getJointConstraint("iiwa_joint_5", goal->joint_position.position.a5));
	constraints.joint_constraints.push_back(getJointConstraint("iiwa_joint_6", goal->joint_position.position.a6));
	constraints.joint_constraints.push_back(getJointConstraint("iiwa_joint_7", goal->joint_position.position.a7));
	moveitGoal.request.goal_constraints.push_back(constraints);

	return moveitGoal;
}

// ---------------------------------------------------------------------------------------------------------------------

moveit_msgs::MoveGroupGoal iiwa_sim::SimNode::toMoveGroupGoal(const iiwa_msgs::MoveToCartesianPoseGoal::ConstPtr goal) {
	std_msgs::Header header = goal->cartesian_pose.poseStamped.header;
	header.seq = _moveGroupSeq++;

	moveit_msgs::MoveGroupGoal moveitGoal = getBlankMoveItGoal(header);

	moveit_msgs::Constraints constraints;
	constraints.position_constraints.push_back(getPositionConstraint(goal->cartesian_pose.poseStamped));
	constraints.orientation_constraints.push_back(getOrientationConstraint(goal->cartesian_pose.poseStamped));

	if (goal->cartesian_pose.redundancy.status != -1 || goal->cartesian_pose.redundancy.turn != -1) {
		constraints.joint_constraints.push_back(getJointConstraint("iiwa_joint_3", goal->cartesian_pose.redundancy.e1));
	}

	moveitGoal.request.goal_constraints.push_back(constraints);



	return moveitGoal;
}

// ---------------------------------------------------------------------------------------------------------------------

moveit_msgs::MoveGroupGoal iiwa_sim::SimNode::toMoveGroupGoal(const iiwa_msgs::MoveToCartesianPoseGoal::ConstPtr goal, const geometry_msgs::PoseStamped& startPose) {
	moveit_msgs::MoveGroupGoal moveitGoal = toMoveGroupGoal(goal);

	std::vector<geometry_msgs::Pose> waypoints = interpolateLinear(startPose.pose, goal->cartesian_pose.poseStamped.pose, _stepSize);
	geometry_msgs::PoseStamped pose;
	pose.header = goal->cartesian_pose.poseStamped.header;

	for (const geometry_msgs::Pose& waypoint : waypoints) {
		pose.pose = waypoint;
		moveit_msgs::Constraints constraints;
		constraints.position_constraints.push_back(getPositionConstraint(pose));
		constraints.orientation_constraints.push_back(getOrientationConstraint(pose));
		moveitGoal.request.trajectory_constraints.constraints.push_back(constraints);
	}

	if (goal->cartesian_pose.redundancy.status != -1 || goal->cartesian_pose.redundancy.turn != -1) {
		const double startJointPosition = _jointStateListener.getJointState("iiwa_joint_3").position;
		const double targetJointPosition = goal->cartesian_pose.redundancy.e1;

		const double step = 1.0/(double)moveitGoal.request.trajectory_constraints.constraints.size();
		for (unsigned int i=0; i<moveitGoal.request.trajectory_constraints.constraints.size(); i++) {
			const double interpolatedJointPosition = (1.0-step*(i+1))*startJointPosition + step*(i+1)*targetJointPosition;
			moveit_msgs::JointConstraint jointConstraint = getJointConstraint("iiwa_joint_3", interpolatedJointPosition);
			moveitGoal.request.trajectory_constraints.constraints[i].joint_constraints.push_back(jointConstraint);
		}
	}

	return moveitGoal;
}

// ---------------------------------------------------------------------------------------------------------------------

void iiwa_sim::SimNode::moveGroupResultCB(const actionlib::SimpleClientGoalState& state, const moveit_msgs::MoveGroupResultConstPtr& moveitResult) {
	// TODO: Reduce redundant code

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
	else if (_moveToCartesianPoseServer.isActive()) {
		iiwa_msgs::MoveToCartesianPoseResult result;

		switch (state.state_) {
			case actionlib::SimpleClientGoalState::SUCCEEDED:
				if (moveitResult->error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
					result.success = false;
					result.error = "Failed with MoveIt! error code "+std::to_string(moveitResult->error_code.val)+": "+state.text_;
				}
				else {
					result.success = true;
				}

				_moveToCartesianPoseServer.setSucceeded(result, state.text_);
				break;
			case actionlib::SimpleClientGoalState::PREEMPTED:
				result.success = false;
				result.error = state.text_;
				_moveToCartesianPoseServer.setPreempted(result, state.text_);
				break;
			case actionlib::SimpleClientGoalState::ABORTED:
			case actionlib::SimpleClientGoalState::LOST:
			case actionlib::SimpleClientGoalState::RECALLED:
				result.success = false;
				result.error = state.text_;
				_moveToCartesianPoseServer.setAborted(result, state.text_);
				break;
			default:
				ROS_ERROR_STREAM("Invalid goal result state: "<<state.state_<<" ("<<state.text_<<")");
				break;
		}
	}
	else if (_moveToCartesianPoseLinServer.isActive()) {
		iiwa_msgs::MoveToCartesianPoseResult result;

		switch (state.state_) {
			case actionlib::SimpleClientGoalState::SUCCEEDED:
				if (moveitResult->error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
					result.success = false;
					result.error = "Failed with MoveIt! error code "+std::to_string(moveitResult->error_code.val)+": "+state.text_;
				}
				else {
					result.success = true;
				}

				_moveToCartesianPoseLinServer.setSucceeded(result, state.text_);
				break;
			case actionlib::SimpleClientGoalState::PREEMPTED:
				result.success = false;
				result.error = state.text_;
				_moveToCartesianPoseLinServer.setPreempted(result, state.text_);
				break;
			case actionlib::SimpleClientGoalState::ABORTED:
			case actionlib::SimpleClientGoalState::LOST:
			case actionlib::SimpleClientGoalState::RECALLED:
				result.success = false;
				result.error = state.text_;
				_moveToCartesianPoseLinServer.setAborted(result, state.text_);
				break;
			default:
				ROS_ERROR_STREAM("Invalid goal result state: "<<state.state_<<" ("<<state.text_<<")");
				break;
		}
	}
}
