/*
 * JoypadOculusControl.h
 *
 *  Created on: Apr 20, 2016
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
#include <ardrone_gesture_control/OculusObserver.h>

// All thresholds for yaw pitch roll in radiant
//Turn head left or right (negative)
#define OCULUS_YAW_THRESHOLD 0.2//0.25
#define OCULUS_MAX_YAW_THRESHOLD 0.5
//Turn head up or down (negative)
#define OCULUS_PITCH_THRESHOLD 0.15 //0.25
#define OCULUS_MAX_PITCH_THRESHOLD 0.4 //0.5
//Tilt head to the left or right (negative)
#define OCULUS_ROLL_THRESHOLD 0.1 //0.25
#define OCULUS_MAX_ROLL_THRESHOLD 0.35 //0.5

#define JOYPAD_THRESHOLD 0.5
#define JOYPAD_MAX_THRESHOLD 0.95

//#define DRONE_SPEED_PITCH_ROLL 0.1
//#define DRONE_SPEED_PITCH_ROLL_JOYPAD 0.5
//#define DRONE_SPEED_YAW_UPDOWN 0.5


#define EPSILON 0.000001

class JoypadOculusControl {
public:
	JoypadOculusControl(ros::NodeHandle &node, std::string &takeoffTopic,
	                    std::string &landTopic, std::string &resetTopic,
	                    std::string &steerTopic, std::string &droneStateTopic,
	                    std::string &flatTrimSrv, std::string &joypadAxisTopic, std::string &joypadBtnTopic,
	                    std::string &oculusTopic, bool travelAllowed = true, float maxDroneSpeedPitch = 0.25,
	                    float maxDroneSpeedRoll = 0.25, float maxDroneSpeedYaw = 0.5, float maxDroneSpeedUpDown = 0.5);

	virtual ~JoypadOculusControl();

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
	OculusObserver oculusObserver;
	GamepadObserver gamepadObserver;

	ros::Publisher takeoff_pub;
	ros::Publisher land_pub;
	ros::Publisher reset_pub;
	ros::Publisher steer_pub;
	ros::Subscriber droneState_sub;
	ros::ServiceClient flattrim_srv;
	ros::Subscriber oculus_sub;
	ros::Subscriber joypadAxis_sub;
	ros::Subscriber joypadBtn_sub;

	DroneState droneState;

	const bool travelAllowed;

	const float maxDroneSpeedPitch, maxDroneSpeedRoll, maxDroneSpeedYaw, maxDroneSpeedUpDown;

	void quadrotorInfoCallback(
			const ardrone_autonomy::Navdata::ConstPtr &navdata);

	bool isFlying();

	double calculateSpeed(double minAngle, double maxAngle, double currentAngle, double maxSpeed);

	void send_control();
};
