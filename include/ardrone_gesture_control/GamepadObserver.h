/*
 * GamepadObserver.h
 *
 *  Created on: Mar 1, 2016
 *      Author: steve
 */

#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

class GamepadObserver {
public:
	GamepadObserver();
	~GamepadObserver();

	bool aButtonPressed();
	bool bButtonPressed();
	bool xButtonPressed();
	bool yButtonPressed();
	const float* getLeftStick() const;
	const float* getRightStick() const;
	void gamepadAxisCallback(
			const std_msgs::Float64MultiArray::ConstPtr& axisData);
	void gamepadBtnCallback(const std_msgs::Int16MultiArray::ConstPtr& btnData);

private:

	bool aButton, bButton, xButton, yButton;

	float leftStick[2];
	float rightStick[2];
};
