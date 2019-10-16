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

#ifndef SRC_JOINT_STATE_LISTENER_H
#define SRC_JOINT_STATE_LISTENER_H

#include <list>
#include <queue>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace iiwa_sim {
	struct JointState {
		std_msgs::Header header;
		std::string name;
		double position;
		double velocity;
		double effort;
	};

	/**
	 * The RobotStateListener subscribes to the "joint_states" topic and buffers the joint states for a robot
	 */
	class JointStateListener {
		public:
			/**
			 * Constructor
			 * @param nh
			 * @param robotName
			 * @param bufferTime
			 */
			JointStateListener(ros::NodeHandle& nh, const ros::Duration& bufferTime = ros::Duration(10));

			/**
			 * Destructor
			 */
			virtual ~JointStateListener();

			/**
			 * Request the state of a specific joint at a given time
			 * @param jointName
			 * @param time
			 * @return
			 */
			iiwa_sim::JointState getJointState(const std::string& jointName, const ros::Time& time = ros::Time(0));

			/**
			 * Callback for joint state messages.
			 * @param msg
			 */
			void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

		protected:
			std::map <std::string, std::list<iiwa_sim::JointState>> _jointStates;
			ros::Duration _bufferTime;
			ros::Subscriber _jointStatesSub;
	};
}


#endif //SRC_JOINT_STATE_LISTENER_H
