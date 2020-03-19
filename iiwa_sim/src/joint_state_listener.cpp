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

#include "iiwa_sim/joint_state_listener.h"

iiwa_sim::JointStateListener::JointStateListener(ros::NodeHandle& nh, const ros::Duration& bufferTime)
: _bufferTime(bufferTime) {
	_jointStatesSub = nh.subscribe("joint_states", 10, &iiwa_sim::JointStateListener::jointStateCallback, this);
}

// ---------------------------------------------------------------------------------------------------------------------

iiwa_sim::JointStateListener::~JointStateListener() {
	_jointStatesSub.shutdown();
}

// ---------------------------------------------------------------------------------------------------------------------

iiwa_sim::JointState iiwa_sim::JointStateListener::getJointState(const std::string& jointName, const ros::Time& time) {
	iiwa_sim::JointState result;
	result.header.frame_id = jointName;
	result.header.stamp = time;
	result.name = jointName;
	result.position = NAN;
	result.effort = NAN;
	result.velocity = NAN;

	if (_jointStates.find(jointName) == _jointStates.end() ) {
		// not entry found for jointState.name
		return result;
	}

	const std::list<iiwa_sim::JointState>& jointStates = _jointStates[jointName];

	if (jointStates.empty()) {
		return result;
	}

	if (time == ros::Time(0)) {
		result.header = jointStates.back().header;
		result.position = jointStates.back().position;
		result.effort = jointStates.back().effort;
		result.velocity = jointStates.back().velocity;
		return result;
	}

	std::list<iiwa_sim::JointState>::const_iterator iterator1 = jointStates.begin();
	while (iterator1 != jointStates.end() && time > iterator1->header.stamp) {
		// just keep iterating until target time is reached
		iterator1++;
	}

	if (iterator1 == jointStates.end()) {
		// requested time is greater than oldest joint data in current dataset
		ROS_WARN_THROTTLE(1.0, "Requested time is greater than latest joint position in dataset");
		return result;
	}
	else if (iterator1->header.stamp == time) {
		// there is an exact match with the requested time and the stored data, so no interpolation is needed
		result.header = jointStates.back().header;
		result.position = iterator1->position;
		result.effort = iterator1->effort;
		result.velocity = iterator1->velocity;
		return result;
	}
	else if (iterator1 == jointStates.begin()) {
		// requested time is lower than oldest joint data in current dataset
		ROS_WARN_THROTTLE(1.0, "Requested time is lower than oldest joint position in dataset");
		return result;
	}

	const iiwa_sim::JointState& js2 = *iterator1;
	const iiwa_sim::JointState& js1 = *(--iterator1);

	// Do interpolation
	ros::Duration interval = js2.header.stamp - js1.header.stamp;
	ros::Duration localTime = time - js1.header.stamp;

	double fractionJ1 = localTime.toSec() / interval.toSec();
	result.position = fractionJ1*js1.position + (1.0 - fractionJ1)*js2.position;
	result.effort = fractionJ1*js1.effort + (1.0 - fractionJ1)*js2.effort;
	result.velocity = fractionJ1*js1.effort + (1.0 - fractionJ1)*js2.velocity;

	return result;
}

// ---------------------------------------------------------------------------------------------------------------------

void iiwa_sim::JointStateListener::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
	for (unsigned int i=0; i<msg->name.size(); i++) {
		iiwa_sim::JointState jointState;
		jointState.header = msg->header;
		jointState.name = msg->name[i];

		if (msg->position.size() <= 0) {
			jointState.position = NAN;
		}
		else {
			jointState.position = msg->position[i];
		}

		if (msg->effort.size() <= 0) {
			jointState.effort = NAN;
		}
		else {
			jointState.effort = msg->effort[i];
		}

		if (msg->velocity.size() <= 0) {
			jointState.velocity = NAN;
		}
		else {
			jointState.velocity = msg->velocity[i];
		}

		if (_jointStates.find(jointState.name) == _jointStates.end() ) {
			// not entry found for jointState.name
			_jointStates[jointState.name] = std::list<iiwa_sim::JointState>();
			_jointStates[jointState.name].push_back(jointState);
		}
		else {
			// found
			std::list<iiwa_sim::JointState>& list = _jointStates[jointState.name];
			ros::Time now = ros::Time::now();

			// remove outdated entries from list
			while (!list.empty() && now - list.front().header.stamp > _bufferTime) {
				list.pop_front();
			}

			// push new joint state to list
			list.push_back(jointState);
		}
	}
}