/*
 * JoypadControl.h
 *
 *  Created on: Jul 12, 2016
 *      Author: steve
 */
#pragma once

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>
#include <ardrone_gesture_control/GamepadObserver.h>


#define JOYPAD_THRESHOLD 0.4
#define JOYPAD_MAX_THRESHOLD 0.95

#define DRONE_SPEED_PITCH_ROLL 0.1
#define DRONE_SPEED_PITCH_ROLL_JOYPAD 0.5
#define DRONE_SPEED_YAW_UPDOWN 0.5



#define EPSILON 0.000001

class JoypadControl {
public:
	JoypadControl(ros::NodeHandle &node, std::string &takeoffTopic,
			std::string &landTopic, std::string &resetTopic,
			std::string &steerTopic, std::string &droneStateTopic,
			std::string &flatTrimSrv, std::string &joypadAxisTopic, std::string &joypadBtnTopic);
	virtual ~JoypadControl();

	void controlDrone();

private:

	enum DroneState {
		Emergency = 0,
		Inited = 1,
		Landed = 2,
		Flying = 3,
		Hovering = 4,
		Test = 5,
		TakingOff = 6,
		GotoHover = 7,
		Landing = 8,
		Looping = 9
	};

	ros::NodeHandle node;
	ros::Rate loopRate;
	GamepadObserver gamepadObserver;

	ros::Publisher takeoff_pub;
	ros::Publisher land_pub;
	ros::Publisher reset_pub;
	ros::Publisher steer_pub;
	ros::Subscriber droneState_sub;
	ros::ServiceClient flattrim_srv;
	ros::Subscriber joypadAxis_sub;
	ros::Subscriber joypadBtn_sub;

	DroneState droneState;

	void quadrotorInfoCallback(
			const ardrone_autonomy::Navdata::ConstPtr& navdata);

	bool isFlying();
	double calculateSpeed(double minAngle, double maxAngle, double currentAngle, double maxSpeed);

	void send_control();
};
