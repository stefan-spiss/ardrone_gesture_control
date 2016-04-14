/*
 * LeapMyoOculusAdvancedControl.cpp
 *
 *  Created on: Apr 13, 2016
 *      Author: steve
 */

#include <ardrone_gesture_control/LeapMyoOculusDroneControl.h>

LeapMyoOculusDroneControl::LeapMyoOculusDroneControl(ros::NodeHandle &node,
		std::string &takeoffTopic, std::string &landTopic,
		std::string &resetTopic, std::string &steerTopic,
		std::string &droneStateTopic, std::string &flatTrimSrv,
		std::string &myoTopic, std::string &leapTopicPose,
		std::string &leapGrabTopic, std::string &oculusTopic) :
		node(node), myoObserver(), leapObserver(), oculusObserver(), droneState(
				Emergency), myoGestureBefore(MyoObserver::NOT_CONNECTED), loopRate(
				10) {
	takeoff_pub = node.advertise<std_msgs::Empty>(takeoffTopic, 1, true);
	land_pub = node.advertise<std_msgs::Empty>(landTopic, 1, true);
	reset_pub = node.advertise<std_msgs::Empty>(resetTopic, 1, true);
	steer_pub = node.advertise<geometry_msgs::Twist>(steerTopic, 1);
	droneState_sub = node.subscribe(droneStateTopic, 1,
			&LeapMyoOculusDroneControl::quadrotorInfoCallback, this);

	flattrim_srv = node.serviceClient<std_srvs::Empty>(flatTrimSrv, 1);

	oculus_sub = node.subscribe("/oculus/orientation", 1,
			&OculusObserver::oculus_callback, &oculusObserver);
	myo_sub = node.subscribe("/myo/data/thalmic_gesture", 1,
			&MyoObserver::myo_callback, &myoObserver);
	leapPose_sub = node.subscribe(leapTopicPose, 1,
			&LeapObserver::leap_callback_pose, &leapObserver);
	leapGrab_sub = node.subscribe(leapGrabTopic, 1,
			&LeapObserver::leap_callback_grab, &leapObserver);
}

LeapMyoOculusDroneControl::~LeapMyoOculusDroneControl() {
}

void LeapMyoOculusDroneControl::controlDrone() {
	while (ros::ok) {
		send_control();
		ros::spinOnce();
		loopRate.sleep();
	}

	std_msgs::Empty empty;
	land_pub.publish(empty);
	ros::shutdown();
}

void LeapMyoOculusDroneControl::quadrotorInfoCallback(
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

bool LeapMyoOculusDroneControl::isFlying() {
	switch (droneState) {
	case Flying:
	case Hovering:
	case GotoHover:
		return true;
	}
	return false;
}

double LeapMyoOculusDroneControl::calculateSpeed(double minAngle,
		double maxAngle, double currentAngle, double maxSpeed) {
	return (currentAngle - minAngle) * maxSpeed / (maxAngle - minAngle);
}

void LeapMyoOculusDroneControl::send_control() {
	/** Service for flattrim */
	std_srvs::Empty flattrim_srv_srvs;

	/** Messages for publishing: */
	std_msgs::Empty emptyMsg;
	geometry_msgs::Twist twist;

	twist.linear.x = twist.linear.y = twist.linear.z = 0;
	twist.angular.x = twist.angular.y = twist.angular.z = 0;

	if (isFlying()) {

		// Land
		if (leapObserver.getHandPose() == LeapObserver::FLAT_HAND) {
			printf("Land\n");
			land_pub.publish(emptyMsg);
		}

		switch (myoObserver.getHandPose()) {
		case MyoObserver::INITIAL:
			twist.linear.x = twist.linear.y = twist.linear.z = 0;
			twist.angular.x = twist.angular.y = twist.angular.z = 0;
			steer_pub.publish(twist);
			break;

			// If FIST, Oculus Position is captured
		case MyoObserver::FIST:
			if (leapObserver.getHandPose() == LeapObserver::FIST) {
				// Control drone rotation
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
			steer_pub.publish(twist);

			break;
			
			// Fly up
		case MyoObserver::RIGHT:
			if (leapObserver.getHandPose() == LeapObserver::MYO_RIGHT_SWIPE) {
				printf("Fly up\n");
				twist.linear.z = DRONE_SPEED_YAW_UPDOWN;
				steer_pub.publish(twist);
			}
			break;

			// Fly down
		case MyoObserver::LEFT:
			if (leapObserver.getHandPose() == LeapObserver::MYO_LEFT_SWIPE) {
				printf("Fly down\n");
				twist.linear.z = -DRONE_SPEED_YAW_UPDOWN;
				steer_pub.publish(twist);
			}
			break;

			// Emergency
		case MyoObserver::SPREAD:
			// if Myo gesture is different than last captured gesture
			// and if the leap motion device doesn't recognize a hand,
			// emergency message is published
			if (myoGestureBefore != myoObserver.getHandPose()
					&& leapObserver.getYaw() <= -10
					&& leapObserver.getPitch() <= -10
					&& leapObserver.getRoll() <= -10) {
				printf("Emergency\n");
				reset_pub.publish(emptyMsg);
			}
			break;
		}
	} else {
		switch (myoObserver.getHandPose()) {
		case MyoObserver::INITIAL:
			break;
		case MyoObserver::FIST:
			break;

			// Flattrim
		case MyoObserver::LEFT:
			if (leapObserver.getHandPose() == LeapObserver::MYO_LEFT_SWIPE) {
				if (myoGestureBefore != myoObserver.getHandPose()) {
					printf("Flattrim\n");
					flattrim_srv.call(flattrim_srv_srvs);
				}
			}
			break;

			// Takeoff
		case MyoObserver::RIGHT:
			if (leapObserver.getHandPose() == LeapObserver::MYO_RIGHT_SWIPE) {
				printf("Start\n");
				takeoff_pub.publish(emptyMsg);
			}
			break;

			// Emergency
		case MyoObserver::SPREAD:
			if (droneState == Emergency) {
				if (myoGestureBefore != myoObserver.getHandPose()) {
					printf("Reset\n");
					reset_pub.publish(emptyMsg);
				}

			}
			break;
		case MyoObserver::TIP:
			;
		}
	}
	myoGestureBefore = myoObserver.getHandPose();
}
