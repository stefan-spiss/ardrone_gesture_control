/*
 * MyoObserver.cpp
 *
 *  Created on: Nov 10, 2015
 *      Author: steve
 */

#include <drone_control/MyoObserver.h>
#include <std_msgs/UInt8.h>

MyoObserver::MyoObserver() :
		hand_pose(MyoObserver::NOT_CONNECTED), changed(false) {
}

MyoObserver::~MyoObserver() {
}

void MyoObserver::myo_callback(const std_msgs::UInt8::ConstPtr& command) {
	hand_pose = MyoObserver::Myo_Gesture(command->data);
	changed = true;
	//printf("myo subscriber called, %d\n", hand_pose);
}

MyoObserver::Myo_Gesture MyoObserver::getHandPose() {
	changed = false;
	return hand_pose;
}

bool MyoObserver::isChanged() const {
	return changed;
}
