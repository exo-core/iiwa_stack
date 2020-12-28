/**
 * Copyright (C) 2016-2020 Arne Peters - arne.peters@tum.de
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

#ifndef SRC_IIWA_SIM_NODE_ROSCONTROL_H
#define SRC_IIWA_SIM_NODE_ROSCONTROL_H

#include <iiwa_sim/iiwa_sim_node.h>

namespace iiwa_sim {
	enum GoalType {
		NONE,
		JOINT_POSITION,
		CARTESIAN_POSE,
		CARTESIAN_POSE_LIN,
		CARTESIAN_SPLINE
	};

	struct MotionGoal {
		GoalType type = NONE;

		iiwa_msgs::MoveToJointPositionGoal::ConstPtr moveToJointPositionGoal;
		iiwa_msgs::MoveToCartesianPoseGoal::ConstPtr moveToCartesianPoseGoal;
		iiwa_msgs::MoveToCartesianPoseGoal::ConstPtr moveToCartesianPoseLinGoal;
		iiwa_msgs::MoveAlongSplineGoal::ConstPtr moveAlongSplineGoal;
		int splineIndex = 0;
	};

	class SimNodeRoscontrol : public SimNode {
		public:
			/**
			 * Constructor
			 */
			SimNodeRoscontrol();

			/**
			 * Destructor
			 */
			virtual ~SimNodeRoscontrol();

			/**
			 * The nodes idle spin loop
			 */
			virtual void spin() override;

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
			 * Callback for iiwa joint position
			 * @param msg
			 */
			void jointPositionCallback(const iiwa_msgs::JointPosition::ConstPtr& msg);

			/**
			 * Callback for iiwa Cartesian pose
			 * @param msg
			 */
			void cartesianPoseCallback(const iiwa_msgs::CartesianPose::ConstPtr& msg);

		protected:
			iiwa_msgs::JointPosition getCurrentJointPosition() const;

			bool goalReached(const iiwa_msgs::JointPosition& currentPosition, const iiwa_msgs::JointPosition& targetPosition) const;
			bool goalReached(const iiwa_msgs::CartesianPose& currentPose, const iiwa_msgs::CartesianPose& targetPose) const;

			template <typename ACTIVE_ACTION_TYPE, typename ACTIVE_ACTION_TYPE_RESULT> void markActionDone(actionlib::SimpleActionServer<ACTIVE_ACTION_TYPE>& activeActionServer, bool success = true, const std::string& error = "") {
				if (success) {
					ROS_DEBUG("[SimNodeRoscontrol] Marking current goal reached.");
				}
				else {
					ROS_DEBUG("[SimNodeRoscontrol] Marking current goal failed.");
				}

				ACTIVE_ACTION_TYPE_RESULT result;
				result.success = success;
				result.error = error;
				activeActionServer.setSucceeded(result);

				_currentGoal.type = NONE;
			}

			MotionGoal _currentGoal;

			ros::Publisher _commandJointPositionPub;
			ros::Publisher _commandCartesianPosePub;
			ros::Publisher _commandCartesianPoseLinPub;

			ros::Subscriber _jointPositionSub;
			ros::Subscriber _cartesianPoseSub;

			ros::ServiceClient _setSmartServoLimitsClient;
			ros::ServiceClient _setSmartServoLinLimitsClient;

			double _redundancyAngleTolerance = 0.005;
			double _jointAngleConstraintTolerance = 0.005;
			double _positionConstraintTolerance = 0.001;
			double _orientationConstraintTolerance = 0.05;
	};
}


#endif //SRC_IIWA_SIM_NODE_ROSCONTROL_H
