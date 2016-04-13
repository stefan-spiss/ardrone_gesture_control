/*
 * MyoObserver.cpp
 *
 *  Created on: Nov 10, 2015
 *      Author: steve
 */

#include <ardrone_gesture_control/MyoObserver.h>
#include <std_msgs/Int32.h>

MyoObserver::MyoObserver() :
		hand_pose(MyoObserver::NOT_CONNECTED), self_classified_pose(-1) {
}

MyoObserver::~MyoObserver() {
}

void MyoObserver::myo_callback(const std_msgs::Int32::ConstPtr& command) {
	hand_pose = MyoObserver::Myo_Gesture(command->data);
}

void MyoObserver::myo_self_classified_callback(const std_msgs::Int32::ConstPtr& command) {
	self_classified_pose = command->data;
}

MyoObserver::Myo_Gesture MyoObserver::getHandPose() const {
	return hand_pose;
}

int MyoObserver::getSelfClassifiedPose() const {
	return self_classified_pose;
}
