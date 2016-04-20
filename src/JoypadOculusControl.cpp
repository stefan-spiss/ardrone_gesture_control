/*
 * JoypadOculusControl.cpp
 *
 *  Created on: Apr 20, 2016
 *      Author: steve
 */

#include <ardrone_gesture_control/JoypadOculusControl.h>

JoypadOculusControl::JoypadOculusControl(ros::NodeHandle& node,
		std::string& takeoffTopic, std::string& landTopic,
		std::string& resetTopic, std::string& steerTopic,
		std::string& droneStateTopic, std::string& flatTrimSrv,
		std::string& joypadAxisTopic, std::string& joypadBtnTopic,
		std::string& oculusTopic) :
		node(node), oculusObserver(), gamepadObserver(), droneState(Emergency), loopRate(
				10) {
	takeoff_pub = node.advertise<std_msgs::Empty>(takeoffTopic, 1, true);
	land_pub = node.advertise<std_msgs::Empty>(landTopic, 1, true);
	reset_pub = node.advertise<std_msgs::Empty>(resetTopic, 1, true);
	steer_pub = node.advertise<geometry_msgs::Twist>(steerTopic, 1);
	droneState_sub = node.subscribe(droneStateTopic, 1,
			&JoypadOculusControl::quadrotorInfoCallback, this);

	flattrim_srv = node.serviceClient<std_srvs::Empty>(flatTrimSrv, 1);

	oculus_sub = node.subscribe("/oculus/orientation", 1,
			&OculusObserver::oculus_callback, &oculusObserver);
	joypadAxis_sub = node.subscribe<std_msgs::Float64MultiArray>(
			joypadAxisTopic, 1, &GamepadObserver::gamepadAxisCallback,
			&gamepadObserver);
	joypadBtn_sub = node.subscribe<std_msgs::Int16MultiArray>(
			joypadBtnTopic, 1, &GamepadObserver::gamepadBtnCallback,
			&gamepadObserver);
}

JoypadOculusControl::~JoypadOculusControl() {
}

void JoypadOculusControl::controlDrone() {
	while (ros::ok()) {
		send_control();
		ros::spinOnce();
		loopRate.sleep();
	}

	std_msgs::Empty empty;
	land_pub.publish(empty);
	ros::shutdown();

}

void JoypadOculusControl::quadrotorInfoCallback(
		const ardrone_autonomy::Navdata::ConstPtr& navdata) {
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

bool JoypadOculusControl::isFlying() {
	switch (droneState) {
	case Flying:
	case Hovering:
	case GotoHover:
		return true;
	}
	return false;
}

double JoypadOculusControl::calculateSpeed(double minAngle, double maxAngle,
		double currentAngle, double maxSpeed) {
	if (fabs(currentAngle) > fabs(maxAngle)) {
		currentAngle = maxAngle;
	}
	return (currentAngle - minAngle) * maxSpeed / (maxAngle - minAngle);
}

void JoypadOculusControl::send_control() {
	/** Service for flattrim */
	std_srvs::Empty flattrim_srv_srvs;

	/** Messages for publishing: */
	std_msgs::Empty emptyMsg;
	geometry_msgs::Twist twist;

	twist.linear.x = twist.linear.y = twist.linear.z = 0;
	twist.angular.x = twist.angular.y = twist.angular.z = 0;

	if (isFlying()) {
		// Steer quadcopter
		if (gamepadObserver.aButtonPressed()) {
			if (oculusObserver.getY() > OCULUS_YAW_THRESHOLD) {
				std::cout << "Rotate counterclockwise" << std::endl;
				twist.angular.z = calculateSpeed(OCULUS_YAW_THRESHOLD,
				OCULUS_MAX_YAW_THRESHOLD, oculusObserver.getY(),
				DRONE_SPEED_YAW_UPDOWN);
			} else if (oculusObserver.getY() < -OCULUS_YAW_THRESHOLD) {
				printf("Rotate clockwise\n");
				twist.angular.z = -calculateSpeed(-OCULUS_YAW_THRESHOLD,
						-OCULUS_MAX_YAW_THRESHOLD, oculusObserver.getY(),
						DRONE_SPEED_YAW_UPDOWN);
			}
#if TRAVEL!=0
			// Control drone speed front and back
			if (oculusObserver.getX() < -OCULUS_PITCH_THRESHOLD) {
				printf("Fly front\n");
				twist.linear.x = DRONE_SPEED_PITCH_ROLL;
			} else if (oculusObserver.getX() > OCULUS_PITCH_THRESHOLD) {
				printf("Fly back\n");
				twist.linear.x = -DRONE_SPEED_PITCH_ROLL;
			}
			// Control drone speed left and right
			if (oculusObserver.getZ() > OCULUS_ROLL_THRESHOLD) {
				printf("Fly left\n");
				twist.linear.y = DRONE_SPEED_PITCH_ROLL;
			} else if (oculusObserver.getZ() < -OCULUS_ROLL_THRESHOLD) {
				printf("Fly right\n");
				twist.linear.y = -DRONE_SPEED_PITCH_ROLL;
			}
#endif
		}
#if TRAVEL==0
		const float* leftStick = gamepadObserver.getLeftStick();
		if (leftStick[0] < -JOYPAD_THRESHOLD) {
			printf("Fly left\n");
			twist.linear.y = calculateSpeed(-JOYPAD_THRESHOLD,
					-JOYPAD_MAX_THRESHOLD, leftStick[0],
					DRONE_SPEED_PITCH_ROLL_JOYPAD);
		} else if (leftStick[0] > JOYPAD_THRESHOLD) {
			printf("Fly right\n");
			twist.linear.y = -calculateSpeed(JOYPAD_THRESHOLD,
			JOYPAD_MAX_THRESHOLD, leftStick[0],
			DRONE_SPEED_PITCH_ROLL_JOYPAD);
		}
		if (leftStick[1] < -JOYPAD_THRESHOLD) {
			printf("Fly front\n");
			twist.linear.x = calculateSpeed(-JOYPAD_THRESHOLD,
					-JOYPAD_MAX_THRESHOLD, leftStick[1],
					DRONE_SPEED_PITCH_ROLL_JOYPAD);
		} else if (leftStick[1] > JOYPAD_THRESHOLD) {
			printf("Fly back\n");
			twist.linear.x = -calculateSpeed(JOYPAD_THRESHOLD,
			JOYPAD_MAX_THRESHOLD, leftStick[1],
			DRONE_SPEED_PITCH_ROLL_JOYPAD);
		}
#endif

		const float* rightStick = gamepadObserver.getRightStick();
		if (rightStick[1] < -JOYPAD_THRESHOLD) {
			printf("Fly up\n");
			twist.linear.z = calculateSpeed(-JOYPAD_THRESHOLD,
					-JOYPAD_MAX_THRESHOLD, rightStick[1],
					DRONE_SPEED_PITCH_ROLL_JOYPAD);
		} else if (rightStick[1] > JOYPAD_THRESHOLD) {
			printf("Fly down\n");
			twist.linear.z = -calculateSpeed(JOYPAD_THRESHOLD,
			JOYPAD_MAX_THRESHOLD, rightStick[1],
			DRONE_SPEED_PITCH_ROLL_JOYPAD);
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
