/*
 * LeapMyoOculusAdvancedControl.h
 *
 *  Created on: Apr 13, 2016
 *      Author: steve
 */

#pragma once

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>
#include <ardrone_gesture_control/MyoObserver.h>
#include <ardrone_gesture_control/LeapObserver.h>
#include <ardrone_gesture_control/OculusObserver.h>

// All thresholds for yaw pitch roll in radiant
//Turn head left or right (negative)
#define OCULUS_YAW_THRESHOLD 0.25
#define OCULUS_MAX_YAW_THRESHOLD 0.5
//Turn head up or down (negative)
#define OCULUS_PITCH_THRESHOLD 0.25
#define OCULUS_MAX_PITCH_THRESHOLD 0.5
//Tilt head to the left or right (negative)
#define OCULUS_ROLL_THRESHOLD 0.25
#define OCULUS_MAX_ROLL_THRESHOLD 0.5

//#define DRONE_SPEED_PITCH_ROLL 0.25
//#define DRONE_SPEED_YAW_UPDOWN 0.5

#define EPSILON 0.000001

class LeapMyoOculusDroneControl {
public:
	LeapMyoOculusDroneControl(ros::NodeHandle &node, std::string &takeoffTopic,
	                          std::string &landTopic, std::string &resetTopic,
	                          std::string &steerTopic, std::string &droneStateTopic,
	                          std::string &flatTrimSrv, std::string &myoTopic,
	                          std::string &leapTopicPose, std::string &leapGrabTopic,
	                          std::string &oculusTopic, bool travelAllowed = true, float maxDroneSpeedPitch = 0.25,
	                          float maxDroneSpeedRoll = 0.25, float maxDroneSpeedYaw = 0.5,
	                          float maxDroneSpeedUpDown = 0.5);

	virtual ~LeapMyoOculusDroneControl();

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
	MyoObserver myoObserver;
	LeapObserver leapObserver;
	OculusObserver oculusObserver;

	ros::Publisher takeoff_pub;
	ros::Publisher land_pub;
	ros::Publisher reset_pub;
	ros::Publisher steer_pub;
	ros::Subscriber droneState_sub;
	ros::ServiceClient flattrim_srv;
	ros::Subscriber oculus_sub;
	ros::Subscriber myo_sub;
	ros::Subscriber leapPose_sub;
	ros::Subscriber leapGrab_sub;

	DroneState droneState;

	const bool travelAllowed;

	const float maxDroneSpeedPitch, maxDroneSpeedRoll, maxDroneSpeedYaw, maxDroneSpeedUpDown;

	MyoObserver::Myo_Gesture myoGestureBefore;

	void quadrotorInfoCallback(
			const ardrone_autonomy::Navdata::ConstPtr &navdata);

	bool isFlying();

	double calculateSpeed(double minAngle, double maxAngle, double currentAngle, double maxSpeed);

	void send_control();
};
