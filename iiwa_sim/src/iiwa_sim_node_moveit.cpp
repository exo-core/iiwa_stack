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

#include <iiwa_sim/iiwa_sim_node_moveit.h>

iiwa_sim::SimNodeMoveit::SimNodeMoveit()
: iiwa_sim::SimNode(),
  _moveGroupClient("move_group", true) {

	_moveGroup = _nh.param<std::string>("move_group", _moveGroup);
	_planner = _nh.param<std::string>("planner", _planner);
	_numPlanningAttempts = _nh.param<double>("num_planning_attempts", _numPlanningAttempts);
	_allowedPlanningTime = _nh.param<double>("allowed_planning_time", _allowedPlanningTime);
	_maxVelocityScalingFactor = _nh.param<double>("max_velocity_scaling_factor", _maxVelocityScalingFactor);
	_maxAccelerationScalingFactor = _nh.param<double>("max_acceleration_scaling_factor", _maxAccelerationScalingFactor);

	_redundancyAngleTolerance = _nh.param<double>("redundancy_angle_tolerance", _redundancyAngleTolerance);
	_jointAngleConstraintTolerance = _nh.param<double>("joint_angle_constraint_tolerance", _jointAngleConstraintTolerance);
	_positionConstraintTolerance = _nh.param<double>("position_constraint_tolerance", _positionConstraintTolerance);
	_orientationConstraintTolerance = _nh.param<double>("orientation_constraint_tolerance", _orientationConstraintTolerance);

	ROS_INFO("[iiwa_sim] Waiting for move_group action client...");

	_moveGroupClient.waitForServer();
}

// ---------------------------------------------------------------------------------------------------------------------

iiwa_sim::SimNodeMoveit::~SimNodeMoveit() {
}

// ---------------------------------------------------------------------------------------------------------------------

void iiwa_sim::SimNodeMoveit::cancelCurrentGoal() {
	if (_moveToJointPositionServer.isActive()) {
		iiwa_msgs::MoveToJointPositionResult result;
		result.error = "New goal received";
		result.success = false;
		_moveToJointPositionServer.setAborted(result, "New goal received");
	}
}

// ---------------------------------------------------------------------------------------------------------------------

void iiwa_sim::SimNodeMoveit::moveToJointPositionGoalCB() {
	cancelCurrentGoal();

	iiwa_msgs::MoveToJointPositionGoal::ConstPtr goal = _moveToJointPositionServer.acceptNewGoal();

	_moveitGoal = toMoveGroupGoal(goal);
	_moveitRetries = 0;

	_moveGroupClient.sendGoal(
			_moveitGoal,
			boost::bind(&SimNodeMoveit::moveGroupResultCB, this, _1, _2),
			actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>::SimpleActiveCallback(),
			actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>::SimpleFeedbackCallback()
	);
}

// ---------------------------------------------------------------------------------------------------------------------

void iiwa_sim::SimNodeMoveit::moveToCartesianPoseGoalCB() {
	cancelCurrentGoal();

	iiwa_msgs::MoveToCartesianPoseGoal::ConstPtr goal = _moveToCartesianPoseServer.acceptNewGoal();

	_moveitGoal = toMoveGroupGoal(goal);
	_moveitRetries = 0;

	_moveGroupClient.sendGoal(
			_moveitGoal,
			boost::bind(&SimNodeMoveit::moveGroupResultCB, this, _1, _2),
			actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>::SimpleActiveCallback(),
			actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>::SimpleFeedbackCallback()
	);
}

// ---------------------------------------------------------------------------------------------------------------------

void iiwa_sim::SimNodeMoveit::moveToCartesianPoseLinGoalCB() {
	cancelCurrentGoal();

	iiwa_msgs::MoveToCartesianPoseGoal::ConstPtr goal = _moveToCartesianPoseLinServer.acceptNewGoal();
	ros::Time lookupTime = goal->cartesian_pose.poseStamped.header.stamp;

	if (goal->cartesian_pose.poseStamped.header.stamp != ros::Time(0) && !_tfListener.waitForTransform(_eeFrame, goal->cartesian_pose.poseStamped.header.frame_id, lookupTime, _maxTfLookupTime)) {
		ROS_WARN_STREAM("[iiwa_sim:"<<__LINE__<<"] No transformation found from "+goal->cartesian_pose.poseStamped.header.frame_id+" to "+_eeFrame<<" at time "<<goal->cartesian_pose.poseStamped.header.stamp<<". Trying latest transformation instead.");
		lookupTime = ros::Time(0);
	}

	tf::StampedTransform startTransform;

	try {
		_tfListener.lookupTransform(_eeFrame, goal->cartesian_pose.poseStamped.header.frame_id, lookupTime, startTransform);
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

	_moveitGoal = toCartesianMoveGroupGoal(goal, startPose);
	_moveitRetries = 0;

	_moveGroupClient.sendGoal(
			_moveitGoal,
			boost::bind(&SimNodeMoveit::moveGroupResultCB, this, _1, _2),
			actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>::SimpleActiveCallback(),
			actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>::SimpleFeedbackCallback()
	);
}

// ---------------------------------------------------------------------------------------------------------------------

void iiwa_sim::SimNodeMoveit::moveAlongSplineGoalCB() {
	cancelCurrentGoal();

	iiwa_msgs::MoveAlongSplineGoal::ConstPtr goal = _moveAlongSplineServer.acceptNewGoal();

	const std::vector<iiwa_msgs::SplineSegment>& splineSegments = goal->spline.segments;
	if (splineSegments.size() == 0) {
		iiwa_msgs::MoveAlongSplineResult result;
		result.error = "Spline has no segments.";
		result.success = false;
		ROS_ERROR(result.error.c_str());
		_moveAlongSplineServer.setSucceeded(result, result.error);
		return;
	}

	std_msgs::Header header;
	header.frame_id = _baseFrame;
	header.stamp = ros::Time::now();
	_moveitGoal = getBlankMoveItGoal(header);
	_moveitRetries = 0;

	ros::Time lookupTime = header.stamp;

	// Get current position of end effector
	if (!_tfListener.waitForTransform(_eeFrame, _baseFrame, header.stamp, _maxTfLookupTime)) {
		ROS_WARN_STREAM("[iiwa_sim:"<<__LINE__<<"] No transformation found from "+_baseFrame+" to "+_eeFrame<<" at time "<<header.stamp<<". Trying latest transformation instead.");
		lookupTime = ros::Time(0);
	}

	tf::StampedTransform startTransform;

	try {
		_tfListener.lookupTransform(_eeFrame, _baseFrame, lookupTime, startTransform);
	}
	catch (tf::TransformException ex) {
		iiwa_msgs::MoveAlongSplineResult result;
		result.error = "tf::TransformException: " + std::string(ex.what());
		result.success = false;
		ROS_ERROR_STREAM("[iiwa_sim:"<<__LINE__<<"] "<<result.error);
		_moveAlongSplineServer.setSucceeded(result, result.error);
		return;
	}

	geometry_msgs::PoseStamped startPoseMsg;
	startPoseMsg.header = header;
	startPoseMsg.pose.position.x = startTransform.getOrigin().x();
	startPoseMsg.pose.position.y = startTransform.getOrigin().y();
	startPoseMsg.pose.position.z = startTransform.getOrigin().z();
	startPoseMsg.pose.orientation.w = startTransform.getRotation().w();
	startPoseMsg.pose.orientation.x = startTransform.getRotation().x();
	startPoseMsg.pose.orientation.y = startTransform.getRotation().y();
	startPoseMsg.pose.orientation.z = startTransform.getRotation().z();

	double startRedundancyAngle = _jointStateListener.getJointState("iiwa_joint_3").position;

	for (const iiwa_msgs::SplineSegment& segment : splineSegments) {
		tf::Stamped<tf::Pose> rawTargetPose, targetPose;
		tf::poseStampedMsgToTF(segment.point.poseStamped, rawTargetPose);
		_tfListener.transformPose(header.frame_id, rawTargetPose, targetPose);
		geometry_msgs::PoseStamped targetPoseMsg;
		tf::poseStampedTFToMsg(targetPose, targetPoseMsg);

		double targetRedundancyAngle = startRedundancyAngle;
		if (segment.point.redundancy.turn != -1 || segment.point.redundancy.turn != -1) {
			targetRedundancyAngle = segment.point.redundancy.e1;
		}

		std::vector<moveit_msgs::Constraints> segmentConstraints;

		switch (segment.type) {
			case iiwa_msgs::SplineSegment::LIN:
				segmentConstraints = getLinearMotionSegmentConstraints(
						header,
						startPoseMsg.pose,
						targetPoseMsg.pose,
						startRedundancyAngle,
						targetRedundancyAngle
				);
				break;
			case iiwa_msgs::SplineSegment::SPL:
				ROS_WARN_THROTTLE(1.0, "iiwa_sim does not support SPL motion segments. Using results of MoveIt! motion planner instead");
				segmentConstraints.push_back(getSplineMotionSegmentConstraints(targetPoseMsg, targetRedundancyAngle));
				break;
			case iiwa_msgs::SplineSegment::CIRC:
				ROS_WARN_THROTTLE(1.0, "iiwa_sim does not support CIRC motion segments. Using results of MoveIt! motion planner instead");
				segmentConstraints.push_back(getSplineMotionSegmentConstraints(targetPoseMsg, targetRedundancyAngle));
				break;
			default:
				iiwa_msgs::MoveAlongSplineResult result;
				result.error = "Invalid spline segment type: "+std::to_string(segment.type);
				result.success = false;
				ROS_ERROR_STREAM("[iiwa_sim:"<<__LINE__<<"] "<<result.error);
				_moveAlongSplineServer.setSucceeded(result, result.error);
				return;
		}

		// append segment constraints
		_moveitGoal.request.trajectory_constraints.constraints.insert(_moveitGoal.request.trajectory_constraints.constraints.end(), segmentConstraints.begin(), segmentConstraints.end());

		startRedundancyAngle = targetRedundancyAngle;
		startPoseMsg = targetPoseMsg;
	}

	moveit_msgs::Constraints goalConstraints;
	goalConstraints.position_constraints.push_back(getPositionConstraint(startPoseMsg));
	goalConstraints.orientation_constraints.push_back(getOrientationConstraint(startPoseMsg));
	goalConstraints.joint_constraints.push_back(getJointConstraint("iiwa_joint_3", startRedundancyAngle, _redundancyAngleTolerance));
	_moveitGoal.request.goal_constraints.push_back(goalConstraints);

	_moveGroupClient.sendGoal(
			_moveitGoal,
			boost::bind(&SimNodeMoveit::moveGroupResultCB, this, _1, _2),
			actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>::SimpleActiveCallback(),
			actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>::SimpleFeedbackCallback()
	);
}

// ---------------------------------------------------------------------------------------------------------------------

bool iiwa_sim::SimNodeMoveit::setPTPJointLimitsServiceCB(iiwa_msgs::SetPTPJointSpeedLimits::Request& request, iiwa_msgs::SetPTPJointSpeedLimits::Response& response) {
	_maxAccelerationScalingFactor = request.joint_relative_acceleration;
	_maxVelocityScalingFactor = request.joint_relative_velocity;

	response.success = true;

	return true;
}

// ---------------------------------------------------------------------------------------------------------------------

moveit_msgs::MoveGroupGoal iiwa_sim::SimNodeMoveit::toMoveGroupGoal(const iiwa_msgs::MoveToJointPositionGoal::ConstPtr goal) {
	std_msgs::Header header = goal->joint_position.header;
	header.seq = _moveGroupSeq++;

	moveit_msgs::MoveGroupGoal moveitGoal = getBlankMoveItGoal(header);

	moveit_msgs::Constraints constraints;
	constraints.joint_constraints.push_back(getJointConstraint("iiwa_joint_1", goal->joint_position.position.a1, _jointAngleConstraintTolerance));
	constraints.joint_constraints.push_back(getJointConstraint("iiwa_joint_2", goal->joint_position.position.a2, _jointAngleConstraintTolerance));
	constraints.joint_constraints.push_back(getJointConstraint("iiwa_joint_3", goal->joint_position.position.a3, _jointAngleConstraintTolerance));
	constraints.joint_constraints.push_back(getJointConstraint("iiwa_joint_4", goal->joint_position.position.a4, _jointAngleConstraintTolerance));
	constraints.joint_constraints.push_back(getJointConstraint("iiwa_joint_5", goal->joint_position.position.a5, _jointAngleConstraintTolerance));
	constraints.joint_constraints.push_back(getJointConstraint("iiwa_joint_6", goal->joint_position.position.a6, _jointAngleConstraintTolerance));
	constraints.joint_constraints.push_back(getJointConstraint("iiwa_joint_7", goal->joint_position.position.a7, _jointAngleConstraintTolerance));
	moveitGoal.request.goal_constraints.push_back(constraints);

	return moveitGoal;
}

// ---------------------------------------------------------------------------------------------------------------------

moveit_msgs::MoveGroupGoal iiwa_sim::SimNodeMoveit::toMoveGroupGoal(const iiwa_msgs::MoveToCartesianPoseGoal::ConstPtr goal) {
	std_msgs::Header header = goal->cartesian_pose.poseStamped.header;
	header.seq = _moveGroupSeq++;

	moveit_msgs::MoveGroupGoal moveitGoal = getBlankMoveItGoal(header);

	moveit_msgs::Constraints constraints;
	constraints.position_constraints.push_back(getPositionConstraint(goal->cartesian_pose.poseStamped));
	constraints.orientation_constraints.push_back(getOrientationConstraint(goal->cartesian_pose.poseStamped));

	if (goal->cartesian_pose.redundancy.status != -1 || goal->cartesian_pose.redundancy.turn != -1) {
		constraints.joint_constraints.push_back(getJointConstraint("iiwa_joint_3", goal->cartesian_pose.redundancy.e1, _redundancyAngleTolerance));
	}

	moveitGoal.request.goal_constraints.push_back(constraints);



	return moveitGoal;
}

// ---------------------------------------------------------------------------------------------------------------------

moveit_msgs::MoveGroupGoal iiwa_sim::SimNodeMoveit::toCartesianMoveGroupGoal(const iiwa_msgs::MoveToCartesianPoseGoal::ConstPtr goal, const geometry_msgs::PoseStamped& startPose) {
	moveit_msgs::MoveGroupGoal moveitGoal = toMoveGroupGoal(goal);

	if (goal->cartesian_pose.redundancy.status == -1 && goal->cartesian_pose.redundancy.turn == -1) {
		moveitGoal.request.trajectory_constraints.constraints = getLinearMotionSegmentConstraints(
				goal->cartesian_pose.poseStamped.header,
				startPose.pose,
				goal->cartesian_pose.poseStamped.pose
		);
	}
	else {
		moveitGoal.request.trajectory_constraints.constraints = getLinearMotionSegmentConstraints(
				goal->cartesian_pose.poseStamped.header,
				startPose.pose,
				goal->cartesian_pose.poseStamped.pose,
				_jointStateListener.getJointState("iiwa_joint_3").position,
				goal->cartesian_pose.redundancy.e1
		);
	}

	return moveitGoal;
}

// ---------------------------------------------------------------------------------------------------------------------

void iiwa_sim::SimNodeMoveit::moveGroupResultCB(const actionlib::SimpleClientGoalState& state, const moveit_msgs::MoveGroupResultConstPtr& moveitResult) {
	if (_moveToJointPositionServer.isActive()) {
		processMoveGroupGoal<iiwa_msgs::MoveToJointPositionAction, iiwa_msgs::MoveToJointPositionResult>(_moveToJointPositionServer, state, moveitResult);
	}
	else if (_moveToCartesianPoseServer.isActive()) {
		processMoveGroupGoal<iiwa_msgs::MoveToCartesianPoseAction, iiwa_msgs::MoveToCartesianPoseResult>(_moveToCartesianPoseServer, state, moveitResult);
	}
	else if (_moveToCartesianPoseLinServer.isActive()) {
		processMoveGroupGoal<iiwa_msgs::MoveToCartesianPoseAction, iiwa_msgs::MoveToCartesianPoseResult>(_moveToCartesianPoseLinServer, state, moveitResult);
	}
	else if (_moveAlongSplineServer.isActive()) {
		processMoveGroupGoal<iiwa_msgs::MoveAlongSplineAction, iiwa_msgs::MoveAlongSplineResult>(_moveAlongSplineServer, state, moveitResult);
	}
}

// ---------------------------------------------------------------------------------------------------------------------

moveit_msgs::JointConstraint iiwa_sim::SimNodeMoveit::getJointConstraint(const std::string& jointName, const double angle, const double tolerance) const{
	moveit_msgs::JointConstraint jointConstraint;
	jointConstraint.joint_name = jointName;
	jointConstraint.position = angle;
	jointConstraint.tolerance_above = tolerance;
	jointConstraint.tolerance_below = tolerance;
	jointConstraint.weight = 1.0;
	return jointConstraint;
}

// ---------------------------------------------------------------------------------------------------------------------

moveit_msgs::PositionConstraint iiwa_sim::SimNodeMoveit::getPositionConstraint(const geometry_msgs::PoseStamped& pose) const {
	moveit_msgs::PositionConstraint positionConstraint;
	positionConstraint.header = pose.header;
	positionConstraint.link_name = _eeFrame;

	positionConstraint.target_point_offset.x = 0.0;
	positionConstraint.target_point_offset.y = 0.0;
	positionConstraint.target_point_offset.z = 0.0;

	shape_msgs::SolidPrimitive sphere;
	sphere.type = shape_msgs::SolidPrimitive::SPHERE;
	sphere.dimensions.push_back(_positionConstraintTolerance);

	positionConstraint.constraint_region.primitives.push_back(sphere);
	positionConstraint.constraint_region.primitive_poses.push_back(pose.pose);

	positionConstraint.weight = 1.0;

	return positionConstraint;
}

// ---------------------------------------------------------------------------------------------------------------------

moveit_msgs::OrientationConstraint iiwa_sim::SimNodeMoveit::getOrientationConstraint(const geometry_msgs::PoseStamped& pose) const {
	moveit_msgs::OrientationConstraint orientationConstraint;
	orientationConstraint.header = pose.header;
	orientationConstraint.link_name = "iiwa_link_ee";

	orientationConstraint.orientation = pose.pose.orientation;

	orientationConstraint.absolute_x_axis_tolerance = _orientationConstraintTolerance;
	orientationConstraint.absolute_y_axis_tolerance = _orientationConstraintTolerance;
	orientationConstraint.absolute_z_axis_tolerance = _orientationConstraintTolerance;

	orientationConstraint.weight = 1.0;

	return orientationConstraint;
}

// ---------------------------------------------------------------------------------------------------------------------

moveit_msgs::MoveGroupGoal iiwa_sim::SimNodeMoveit::getBlankMoveItGoal(const std_msgs::Header& header) const {
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
	moveitGoal.request.planner_id = _planner;

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

std::vector<moveit_msgs::Constraints> iiwa_sim::SimNodeMoveit::getLinearMotionSegmentConstraints(const std_msgs::Header& header, const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2) const {
	std::vector<moveit_msgs::Constraints> constraints;
	std::vector<geometry_msgs::Pose> waypoints = interpolateLinear(p1, p2, _stepSize);
	geometry_msgs::PoseStamped pose;
	pose.header = header;

	for (const geometry_msgs::Pose& waypoint : waypoints) {
		pose.pose = waypoint;
		moveit_msgs::Constraints waypointConstraints;
		waypointConstraints.position_constraints.push_back(getPositionConstraint(pose));
		waypointConstraints.orientation_constraints.push_back(getOrientationConstraint(pose));
		constraints.push_back(waypointConstraints);
	}

	return constraints;
}

// ---------------------------------------------------------------------------------------------------------------------

std::vector<moveit_msgs::Constraints> iiwa_sim::SimNodeMoveit::getLinearMotionSegmentConstraints(const std_msgs::Header& header, const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2, const double j1, const double j2) const {
	std::vector<moveit_msgs::Constraints> constraints = getLinearMotionSegmentConstraints(header, p1, p2);

	const double step = 1.0/(double)constraints.size();
	for (unsigned int i=0; i<constraints.size(); i++) {
		const double interpolatedJointPosition = (1.0-step*(i+1))*j1 + step*(i+1)*j2;
		moveit_msgs::JointConstraint jointConstraint = getJointConstraint("iiwa_joint_3", interpolatedJointPosition, _redundancyAngleTolerance);
		constraints[i].joint_constraints.push_back(jointConstraint);
	}

	return constraints;
}

// ---------------------------------------------------------------------------------------------------------------------

moveit_msgs::Constraints iiwa_sim::SimNodeMoveit::getSplineMotionSegmentConstraints(const geometry_msgs::PoseStamped& p, const double j) const {
	moveit_msgs::Constraints constraints;
	constraints.position_constraints.push_back(getPositionConstraint(p));
	constraints.orientation_constraints.push_back(getOrientationConstraint(p));
	constraints.joint_constraints.push_back(getJointConstraint("iiwa_joint_3", j, _redundancyAngleTolerance));
	return constraints;
}

// ---------------------------------------------------------------------------------------------------------------------

