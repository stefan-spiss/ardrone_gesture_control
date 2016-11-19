/*
 * JoypadControl.cpp
 *
 *  Created on: Jul 12, 2016
 *      Author: steve
 */

#include <ardrone_gesture_control/JoypadControl.h>

JoypadControl::JoypadControl(ros::NodeHandle &node, std::string &takeoffTopic,
                             std::string &landTopic, std::string &resetTopic,
                             std::string &steerTopic, std::string &droneStateTopic,
                             std::string &flatTrimSrv, std::string &joypadAxisTopic,
                             std::string &joypadBtnTopic, float maxDroneSpeedPitch, float maxDroneSpeedRoll,
                             float maxDroneSpeedYaw, float maxDroneSpeedUpDown) :
		node(node), gamepadObserver(), droneState(Emergency), loopRate(10), maxDroneSpeedPitch(maxDroneSpeedPitch),
		maxDroneSpeedRoll(maxDroneSpeedRoll), maxDroneSpeedYaw(maxDroneSpeedYaw),
		maxDroneSpeedUpDown(maxDroneSpeedUpDown) {
	takeoff_pub = node.advertise<std_msgs::Empty>(takeoffTopic, 1, true);
	land_pub = node.advertise<std_msgs::Empty>(landTopic, 1, true);
	reset_pub = node.advertise<std_msgs::Empty>(resetTopic, 1, true);
	steer_pub = node.advertise<geometry_msgs::Twist>(steerTopic, 1);
	droneState_sub = node.subscribe(droneStateTopic, 1,
	                                &JoypadControl::quadrotorInfoCallback, this);

	flattrim_srv = node.serviceClient<std_srvs::Empty>(flatTrimSrv, 1);

	joypadAxis_sub = node.subscribe<std_msgs::Float64MultiArray>(
			joypadAxisTopic, 1, &GamepadObserver::gamepadAxisCallback,
			&gamepadObserver);
	joypadBtn_sub = node.subscribe<std_msgs::Int16MultiArray>(joypadBtnTopic, 1,
	                                                          &GamepadObserver::gamepadBtnCallback, &gamepadObserver);
}

JoypadControl::~JoypadControl() {
}

void JoypadControl::controlDrone() {
	while (ros::ok()) {
		send_control();
		ros::spinOnce();
		loopRate.sleep();
	}

	std_msgs::Empty empty;
	land_pub.publish(empty);
	ros::shutdown();
}

void JoypadControl::quadrotorInfoCallback(
		const ardrone_autonomy::Navdata::ConstPtr &navdata) {
	switch (navdata->state) {
		case 0:
			droneState = Emergency;
			break;
		case 1:
			droneState = Inited;
			break;
		case 2:
			droneState = Landed;
			break;
		case 3:
			droneState = Flying;
			break;
		case 4:
			droneState = Hovering;
			break;
		case 5:
			droneState = Test;
			break;
		case 6:
			droneState = TakingOff;
			break;
		case 7:
			droneState = GotoHover;
			break;
		case 8:
			droneState = Landing;
			break;
		case 9:
			droneState = Looping;
			break;
	}
}

bool JoypadControl::isFlying() {
	switch (droneState) {
		case Flying:
		case Hovering:
		case GotoHover:
			return true;
	}
	return false;
}

double JoypadControl::calculateSpeed(double minAngle, double maxAngle,
                                     double currentAngle, double maxSpeed) {
	if (fabs(currentAngle) > fabs(maxAngle)) {
		currentAngle = maxAngle;
	}
	return (currentAngle - minAngle) * maxSpeed / (maxAngle - minAngle);
}

void JoypadControl::send_control() {
	/** Service for flattrim */
	std_srvs::Empty flattrim_srv_srvs;

	/** Messages for publishing: */
	std_msgs::Empty emptyMsg;
	geometry_msgs::Twist twist;

	twist.linear.x = twist.linear.y = twist.linear.z = 0;
	twist.angular.x = twist.angular.y = twist.angular.z = 0;

	if (isFlying()) {
		// Steer quadcopter
		const float *leftStick = gamepadObserver.getLeftStick();
		if (leftStick[0] < -JOYPAD_THRESHOLD) {
			printf("Fly left\n");
			twist.linear.y = calculateSpeed(-JOYPAD_THRESHOLD,
			                                -JOYPAD_MAX_THRESHOLD, leftStick[0],
			                                maxDroneSpeedRoll);
		} else if (leftStick[0] > JOYPAD_THRESHOLD) {
			printf("Fly right\n");
			twist.linear.y = -calculateSpeed(JOYPAD_THRESHOLD,
			                                 JOYPAD_MAX_THRESHOLD, leftStick[0],
			                                 maxDroneSpeedRoll);
		}
		if (leftStick[1] < -JOYPAD_THRESHOLD) {
			printf("Fly front\n");
			twist.linear.x = calculateSpeed(-JOYPAD_THRESHOLD,
			                                -JOYPAD_MAX_THRESHOLD, leftStick[1],
			                                maxDroneSpeedPitch);
		} else if (leftStick[1] > JOYPAD_THRESHOLD) {
			printf("Fly back\n");
			twist.linear.x = -calculateSpeed(JOYPAD_THRESHOLD,
			                                 JOYPAD_MAX_THRESHOLD, leftStick[1],
			                                 maxDroneSpeedPitch);
		}

		const float *rightStick = gamepadObserver.getRightStick();
		if (rightStick[0] < -JOYPAD_THRESHOLD) {
			std::cout << "Rotate counterclockwise" << std::endl;
			twist.angular.z = calculateSpeed(-JOYPAD_THRESHOLD,
			                                 -JOYPAD_MAX_THRESHOLD, rightStick[0],
			                                 maxDroneSpeedYaw);
		} else if (rightStick[0] > JOYPAD_THRESHOLD) {
			printf("Rotate clockwise\n");
			twist.angular.z = -calculateSpeed(JOYPAD_THRESHOLD,
			                                  JOYPAD_MAX_THRESHOLD, rightStick[0],
			                                  maxDroneSpeedYaw);
		}

		if (rightStick[1] < -JOYPAD_THRESHOLD) {
			printf("Fly up\n");
			twist.linear.z = calculateSpeed(-JOYPAD_THRESHOLD,
			                                -JOYPAD_MAX_THRESHOLD, rightStick[1],
			                                maxDroneSpeedUpDown);
		} else if (rightStick[1] > JOYPAD_THRESHOLD) {
			printf("Fly down\n");
			twist.linear.z = -calculateSpeed(JOYPAD_THRESHOLD,
			                                 JOYPAD_MAX_THRESHOLD, rightStick[1],
			                                 maxDroneSpeedUpDown);
		}

		steer_pub.publish(twist);

		// Land
		if (gamepadObserver.bButtonPressed()) {
			printf("Land\n");
			land_pub.publish(emptyMsg);
		}
		//	if (gamepadObserver.xButtonPressed()) {
		//
		//	}

		// Emergency
		if (gamepadObserver.yButtonPressed()) {
			printf("Emergency\n");
			reset_pub.publish(emptyMsg);
		}
	} else {
		// Takeoff
		if (gamepadObserver.aButtonPressed()) {
			printf("Start\n");
			takeoff_pub.publish(emptyMsg);
		}

		// Flattrim
		if (gamepadObserver.bButtonPressed()) {
			printf("Flattrim\n");
			flattrim_srv.call(flattrim_srv_srvs);
		}

		// Reset
		if (gamepadObserver.yButtonPressed()) {
			if (droneState == Emergency) {
				printf("Reset\n");
				reset_pub.publish(emptyMsg);
			}
		}
	}
}
