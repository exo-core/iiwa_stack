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

#ifndef SRC_IIWA_SIM_NODE_ROSCONTROL_H
#define SRC_IIWA_SIM_NODE_ROSCONTROL_H

#include <iiwa_sim/iiwa_sim_node.h>

namespace iiwa_sim {
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

		protected:
			ros::Publisher _commandJointPositionPub;
			ros::Publisher _commandCartesianPosePub;
			ros::Publisher _commandCartesianPoseLinPub;
	};
}


#endif //SRC_IIWA_SIM_NODE_ROSCONTROL_H
