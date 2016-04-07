/*
 * LeapObserver.cpp
 *
 *  Created on: Nov 26, 2015
 *      Author: steve
 */

#include <drone_control/LeapObserver.h>
#include <geometry_msgs/Pose.h>

LeapObserver::LeapObserver() :
		orientation( { 0.0, 0.0, 0.0, 0.0 }), hand_position( { 0.0, 0.0, 0.0 }) {
}

LeapObserver::~LeapObserver() {
}

void LeapObserver::leap_callback_pose(
		const geometry_msgs::Pose::ConstPtr& pose_data) {
	orientation[0] = pose_data->orientation.x;
	orientation[1] = pose_data->orientation.y;
	orientation[2] = pose_data->orientation.z;
	orientation[3] = pose_data->orientation.w;

	hand_position[0] = pose_data->position.x;
	hand_position[1] = pose_data->position.y;
	hand_position[2] = pose_data->position.z;
}

const float* LeapObserver::getHandPosition() const {
	return hand_position;
}

const float LeapObserver::getPitch() const {
	return orientation[0];
}

const float LeapObserver::getYaw() const {
	return orientation[1];
}

const float LeapObserver::getRoll() const {
	return orientation[2];
}
