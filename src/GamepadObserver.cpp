/*
 * GamepadObserver.cpp
 *
 *  Created on: Mar 1, 2016
 *      Author: steve
 */

#include <ardrone_gesture_control/GamepadObserver.h>

GamepadObserver::GamepadObserver() :
		aButton(false), bButton(false), xButton(false), yButton(false) {
	leftStick[0] = leftStick[1] = rightStick[0] = rightStick[1] = 0.0;
}

GamepadObserver::~GamepadObserver() {

}

bool GamepadObserver::aButtonPressed() {
	if (aButton) {
		aButton = false;
		return true;
	}
	return false;
}

bool GamepadObserver::bButtonPressed() {
	if (bButton) {
		bButton = false;
		return true;
	}
	return false;
}

bool GamepadObserver::xButtonPressed() {
	if (xButton) {
		xButton = false;
		return true;
	}
	return false;
}

bool GamepadObserver::yButtonPressed() {
	if (yButton) {
		yButton = false;
		return true;
	}
	return false;
}

void GamepadObserver::gamepadAxisCallback(
		const std_msgs::Float64MultiArray::ConstPtr& axisData) {
	leftStick[0] = axisData->data[0];
	leftStick[1] = axisData->data[1];
	rightStick[0] = axisData->data[2];
	rightStick[1] = axisData->data[3];
}

void GamepadObserver::gamepadBtnCallback(
		const std_msgs::Int16MultiArray::ConstPtr& btnData) {
	if (btnData->data[0] == 1) {
		aButton = true;
	} else {
		aButton = false;
	}

	if (btnData->data[1] == 1) {
		bButton = true;
	} else {
		bButton = false;
	}

	if (btnData->data[2] == 1) {
		xButton = true;
	} else {
		xButton = false;
	}

	if (btnData->data[3] == 1) {
		yButton = true;
	} else {
		yButton = false;
	}
}

const float* GamepadObserver::getLeftStick() const {
	return leftStick;
}

const float* GamepadObserver::getRightStick() const {
	return rightStick;
}
