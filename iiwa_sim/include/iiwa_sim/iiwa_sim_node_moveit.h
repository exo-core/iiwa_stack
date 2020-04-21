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

#ifndef SRC_IIWA_SIM_NODE_MOVEIT_H
#define SRC_IIWA_SIM_NODE_MOVEIT_H

#include <iiwa_sim/iiwa_sim_node.h>
#include <moveit_msgs/MoveGroupAction.h>

namespace iiwa_sim {
	class SimNodeMoveit : public SimNode {
		public:
			/**
			 * Constructor
			 */
			SimNodeMoveit();

			/**
			 * Destructor
			 */
			virtual ~SimNodeMoveit();

			/**
			 * Goal callback for move_to_joint_position action
			 */
			virtual void moveToJointPositionGoalCB() override;

			/**
			 * Goal callback for move_to_cartesian_pose action
			 */
			virtual void moveToCartesianPoseGoalCB() override;

			/**
			 * Goal callback for move_to_cartesian_pose_lin action
			 */
			virtual void moveToCartesianPoseLinGoalCB() override;

			/**
			 * Goal callback for move_along_spline action
			 */
			virtual void moveAlongSplineGoalCB() override;

			/**
			 * Stops the current goal, if any.
			 */
			virtual void cancelCurrentGoal();

			/**
			 * Callback for configuration/setPTPJointLimits service
			 * @param request
			 * @param response
			 * @return
			 */
			virtual bool setPTPJointLimitsServiceCB(iiwa_msgs::SetPTPJointSpeedLimits::Request& request, iiwa_msgs::SetPTPJointSpeedLimits::Response& response) override;

			/**
			 * Goal reached callback for MoveIt!'s move_group action
			 * @param state
			 * @param result
			 */
			void moveGroupResultCB(const actionlib::SimpleClientGoalState& state, const moveit_msgs::MoveGroupResultConstPtr& result);

			/**
			 * Create a MoveIt! constraint message for a specific axis
			 * @param jointName
			 * @param angle
			 * @param tolerance
			 * @return
			 */
			moveit_msgs::JointConstraint getJointConstraint(const std::string& jointName, const double angle, const double tolerance) const;

			/**
			 * Create a MoveIt! constraint message for a specific target position
			 * @param pose
			 * @return
			 */
			moveit_msgs::PositionConstraint getPositionConstraint(const geometry_msgs::PoseStamped& pose) const;

			/**
			 * Create a MoveIt! constraint message for a specific target orientation
			 * @param pose
			 * @return
			 */
			moveit_msgs::OrientationConstraint getOrientationConstraint(const geometry_msgs::PoseStamped& pose) const;

			/**
			 * Creates a MoveIt! move_group goal based on a target joint configuration
			 * @param goal
			 * @return
			 */
			moveit_msgs::MoveGroupGoal toMoveGroupGoal(const iiwa_msgs::MoveToJointPositionGoal::ConstPtr goal);

			/**
			 * Creates a MoveIt! move_group goal based on a Cartesian target pose
			 * @param goal
			 * @return
			 */
			moveit_msgs::MoveGroupGoal toMoveGroupGoal(const iiwa_msgs::MoveToCartesianPoseGoal::ConstPtr goal);

			/**
			 * Creates a MoveIt! move_group goal for a linear motion based on a Cartesian target pose
			 * @param goal
			 * @param startPose: Current pose of the end effector
			 * @return
			 */
			moveit_msgs::MoveGroupGoal toCartesianMoveGroupGoal(const iiwa_msgs::MoveToCartesianPoseGoal::ConstPtr goal, const geometry_msgs::PoseStamped& startPose);

			/**
			 * Initializes a goal for a move_group action without goal or trajectory constraints
			 * @param header
			 * @return
			 */
			moveit_msgs::MoveGroupGoal getBlankMoveItGoal(const std_msgs::Header& header) const;


			/**
			 * Creates a list of waypoint constraint for a motion along a Cartesian line
			 * @param header: Header to be used for creating stamped poses
			 * @param p1: start pose
			 * @param p2: target pose
			 * @return
			 */
			std::vector<moveit_msgs::Constraints> getLinearMotionSegmentConstraints(const std_msgs::Header& header, const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2) const;

			/**
			 * Creates a list of waypoint constraint for a motion along a Cartesian line with redundancy parameters
			 * @param header: Header to be used for creating stamped poses
			 * @param p1: start pose
			 * @param p2: target pose
			 * @param j1: start value for iiwa_joint_3
			 * @param j2: target value for iiwa_joint_3
			 * @return
			 */
			std::vector<moveit_msgs::Constraints> getLinearMotionSegmentConstraints(const std_msgs::Header& header, const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2, const double j1, const double j2) const;

			/**
			 *
			 * @param p
			 * @param j
			 * @return
			 */
			moveit_msgs::Constraints getSplineMotionSegmentConstraints(const geometry_msgs::PoseStamped& p, const double j) const;

		protected:
			template <typename ACTIVE_ACTION_TYPE, typename ACTIVE_ACTION_TYPE_RESULT> void processMoveGroupGoal(actionlib::SimpleActionServer<ACTIVE_ACTION_TYPE>& activeActionServer, const actionlib::SimpleClientGoalState& state, const moveit_msgs::MoveGroupResultConstPtr& moveitResult) {
				ACTIVE_ACTION_TYPE_RESULT result;

				switch (state.state_) {
					case actionlib::SimpleClientGoalState::SUCCEEDED:
						if (moveitResult->error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
							result.success = false;
							result.error = "Failed with MoveIt! error code "+std::to_string(moveitResult->error_code.val)+": "+state.text_;
						}
						else {
							result.success = true;
						}

						activeActionServer.setSucceeded(result, state.text_);
						break;
					case actionlib::SimpleClientGoalState::PREEMPTED:
						result.success = false;
						result.error = state.text_;
						activeActionServer.setPreempted(result, state.text_);
						break;
					case actionlib::SimpleClientGoalState::ABORTED:
						if (moveitResult->error_code.val == moveit_msgs::MoveItErrorCodes::CONTROL_FAILED && _moveitRetries < _maxMoveitRetries) {
							ROS_DEBUG_STREAM("[iiwa_sim] "<<state.text_<<" - Resending goal");
							ros::Duration(0.1).sleep();
							_moveGroupClient.sendGoal(
									_moveitGoal,
									boost::bind(&SimNodeMoveit::moveGroupResultCB, this, _1, _2),
									actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>::SimpleActiveCallback(),
									actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>::SimpleFeedbackCallback()
							);
							_moveitRetries++;
							return;
						}
					case actionlib::SimpleClientGoalState::LOST:
					case actionlib::SimpleClientGoalState::RECALLED:
						result.success = false;
						result.error = state.text_;
						activeActionServer.setAborted(result, state.text_);
						break;
					default:
						ROS_ERROR_STREAM("[iiwa_sim] Invalid goal result state: "<<state.state_<<" ("<<state.text_<<")");
						break;
				}
			}

			// Action client
			actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> _moveGroupClient;
			moveit_msgs::MoveGroupGoal _moveitGoal;
			int _moveGroupSeq = 1;
			int _moveitRetries;
			int _maxMoveitRetries = 4;

			// MoveIt! parameters
			std::string _moveGroup = "manipulator";
			std::string _planner = "RRTStar";
			int _numPlanningAttempts = 10;
			double _allowedPlanningTime = 10.0;
			double _maxVelocityScalingFactor = 1.0;
			double _maxAccelerationScalingFactor = 1.0;
			double _redundancyAngleTolerance = 0.05;
			double _jointAngleConstraintTolerance = 0.005;
			double _positionConstraintTolerance = 0.001;
			double _orientationConstraintTolerance = 0.01;
	};
}

#endif //SRC_IIWA_SIM_NODE_MOVEIT_H
