/*
 * LeapObserver.cpp
 *
 *  Created on: Nov 26, 2015
 *      Author: steve
 */

#include <ardrone_gesture_control/LeapObserver.h>
#include <geometry_msgs/Pose.h>

LeapObserver::LeapObserver() :
		orientation( { -10.0, -10.0, -10.0, -10.0 }), hand_position( { 0.0, 0.0,
				0.0 }), grab(-1.0) {
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

void LeapObserver::leap_callback_dir(
		const geometry_msgs::Vector3::ConstPtr& dir_data) {
	direction[0] = dir_data->x;
	direction[1] = dir_data->y;
	direction[2] = dir_data->z;
}

void LeapObserver::leap_callback_normal(
		const geometry_msgs::Vector3::ConstPtr& normal_data) {
	normal[0] = normal_data->x;
	normal[1] = normal_data->y;
	normal[2] = normal_data->z;
}

void LeapObserver::leap_callback_grab(
		const std_msgs::Float32::ConstPtr& grab_data) {
	grab = grab_data->data;
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

const float* LeapObserver::getDirection() const {
	return direction;
}

const float* LeapObserver::getNormal() const {
	return normal;
}

const float LeapObserver::getGrab() const {
	return grab;
}

const LeapObserver::Leap_Hand_Gestures LeapObserver::getHandPose() const {
	if (grab > GESTURE_FIST_GRAB) {
		return FIST;
	} else if (orientation[1] < GESTURE_MYO_RIGHT_SWIPE_YAW
			&& orientation[1] > GESTURE_MYO_LEFT_SWIPE_YAW
			&& orientation[2] > GESTURE_MYO_ROLL_MIN
			&& orientation[2] < GESTURE_MYO_ROLL_MAX
			&& orientation[0] > GESTURE_MYO_PITCH_MIN
			&& orientation[0] < GESTURE_MYO_PITCH_MAX) {
		return MYO_INITIAL;
	} else if (orientation[1] > GESTURE_MYO_RIGHT_SWIPE_YAW
			&& orientation[2] > GESTURE_MYO_ROLL_MIN
			&& orientation[2] < GESTURE_MYO_ROLL_MAX
			&& orientation[0] > GESTURE_MYO_PITCH_MIN
			&& orientation[0] < GESTURE_MYO_PITCH_MAX) {
		return MYO_RIGHT_SWIPE;
	} else if (orientation[1] < GESTURE_MYO_LEFT_SWIPE_YAW
			&& orientation[2] > GESTURE_MYO_ROLL_MIN
			&& orientation[2] < GESTURE_MYO_ROLL_MAX
			&& orientation[0] > GESTURE_MYO_PITCH_MIN
			&& orientation[0] < GESTURE_MYO_PITCH_MAX) {
		return MYO_LEFT_SWIPE;
	} else if (orientation[1] < GESTURE_FLAT_HAND_YAW
			&& orientation[1] > -GESTURE_FLAT_HAND_YAW
			&& orientation[2] > -GESTURE_FLAT_HAND_ROLL
			&& orientation[2] < GESTURE_FLAT_HAND_ROLL
			&& orientation[0] > GESTURE_FLAT_HAND_PITCH_MIN
			&& orientation[0] < GESTURE_FLAT_HAND_PITCH_MAX) {
		return FLAT_HAND;
	} else if (orientation[1] > GESTURE_FLAT_HAND_RIGHT_SWIPE_YAW
			&& orientation[2] > -GESTURE_FLAT_HAND_ROLL
			&& orientation[2] < GESTURE_FLAT_HAND_ROLL
			&& orientation[0] > GESTURE_FLAT_HAND_PITCH_MIN
			&& orientation[0] < GESTURE_FLAT_HAND_PITCH_MAX) {
		return FLAT_HAND_SWIPE_RIGHT;
	} else if (grab < 0.0 || orientation[0] == -10.0) {
		return NO_HAND;
	}
	return NOTHING_DETECTED;
}

