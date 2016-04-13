/*
 * leap_myo_oculus_control.cpp
 *
 *  Created on: Nov 26, 2015
 *      Author: steve
 */

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_gesture_control/OculusObserver.h>
#include <ardrone_gesture_control/MyoObserver.h>
#include <ardrone_gesture_control/LeapObserver.h>

// All thresholds for yaw pitch roll in radiant
#define OCULUS_YAW_THRESHOLD 0.25 //Turn head left or right (negative)
#define OCULUS_PITCH_THRESHOLD 0.25 //Turn head up or down (negative)
#define OCULUS_ROLL_THRESHOLD 0.25 //Tilt head to the left or right (negative)

#define LEAP_YAW_THRESHOLD 0.5 //Turn hand left (negative) or right
#define LEAP_PITCH_BACK_THRESHOLD 0.8 //Bend wrist back
#define LEAP_PITCH_FRONT_THRESHOLD -0.5	//Bend wrist to the front (negative)
#define LEAP_ROLL_THRESHOLD 0.8 //Rotate wrist to the left or right (negative)

#define DRONE_SPEED_PITCH_ROLL 1.0	//0.1
#define DRONE_SPEED_YAW_UPDOWN 1.0	//0.5

#define EPSILON 0.000001

bool flying = false;
MyoObserver::Myo_Gesture myo_gesture_before;
int emergency_count = 0;

/**
 * Function is called in the ros loop. Publishes the control commands for the quadcopter according to the captured gestures.
 * @param myo_gesture		Captured gesture from the Myo device
 * @param oculus_observer	OculusObserver object which is used to get the oculussensor data
 * @param leap_observer		LeapObserver object which is used to get the leapmotion sensor data
 * @param flattrim_srv		Service which is used to flattrim the quadcopter
 * @param takeoff_pub		Publisher which is used to start the quadcopter
 * @param land_pub			Publisher which is used to land the quadcopter
 * @param reset_pub			Publisher which is used for emergency and to reset the quadcopter
 * @param steer_pub			Publisher which is used for the steering commands of the quadcopter
 */
void send_control(MyoObserver::Myo_Gesture myo_gesture,
		OculusObserver oculus_observer, LeapObserver leap_observer,
		ros::ServiceClient flattrim_srv, ros::Publisher takeoff_pub,
		ros::Publisher land_pub, ros::Publisher reset_pub,
		ros::Publisher steer_pub) {
	/** Service for flattrim */
	std_srvs::Empty flattrim_srv_srvs;

	/** Messages for publishing: */
	std_msgs::Empty emptyMsg;
	geometry_msgs::Twist twist;

	twist.linear.x = twist.linear.y = twist.linear.z = 0;
	twist.angular.x = twist.angular.y = twist.angular.z = 0;

	if (flying) {
		switch (myo_gesture) {
		case MyoObserver::INITIAL:
			twist.linear.x = twist.linear.y = twist.linear.z = 0;
			twist.angular.x = twist.angular.y = twist.angular.z = 0;
			steer_pub.publish(twist);
			break;
			// If FIST, Oculus Position is captured
		case MyoObserver::FIST:
			// Control drone rotation
			if (oculus_observer.getY() > OCULUS_YAW_THRESHOLD
					&& leap_observer.getYaw() < -LEAP_YAW_THRESHOLD) {
				printf("Rotate counterclockwise\n");
				twist.angular.z = DRONE_SPEED_YAW_UPDOWN;
			} else if (oculus_observer.getY() < -OCULUS_YAW_THRESHOLD
					&& leap_observer.getYaw() > LEAP_YAW_THRESHOLD) {
				printf("Rotate clockwise\n");
				twist.angular.z = -DRONE_SPEED_YAW_UPDOWN;
			}
			// Control drone speed front and back
			if ((oculus_observer.getX() < -OCULUS_PITCH_THRESHOLD)
					&& (leap_observer.getPitch() < LEAP_PITCH_FRONT_THRESHOLD)) {
				printf("Fly front\n");
				twist.linear.x = DRONE_SPEED_PITCH_ROLL;
			} else if ((oculus_observer.getX() > OCULUS_PITCH_THRESHOLD)
					&& (leap_observer.getPitch() > LEAP_PITCH_BACK_THRESHOLD)) {
				printf("Fly back\n");
				twist.linear.x = -DRONE_SPEED_PITCH_ROLL;
			}
			// Control drone speed left and right
			if ((oculus_observer.getZ() > OCULUS_ROLL_THRESHOLD)
					&& (leap_observer.getRoll() > LEAP_ROLL_THRESHOLD)) {
				printf("Fly left\n");
				twist.linear.y = DRONE_SPEED_PITCH_ROLL;
			} else if ((oculus_observer.getZ() < -OCULUS_ROLL_THRESHOLD)
					&& (leap_observer.getRoll() < -LEAP_ROLL_THRESHOLD)) {
				printf("Fly right\n");
				twist.linear.y = -DRONE_SPEED_PITCH_ROLL;
			}

			steer_pub.publish(twist);

			break;
			// Fly up
		case MyoObserver::LEFT:
			printf("Fly up\n");
			twist.linear.z = DRONE_SPEED_YAW_UPDOWN;
			steer_pub.publish(twist);
			break;
			// Fly left
		case MyoObserver::RIGHT:
			printf("Fly down\n");
			twist.linear.z = -DRONE_SPEED_YAW_UPDOWN;
			steer_pub.publish(twist);
			break;
		case MyoObserver::SPREAD:
			// if Myo gesture is different than last captured gesture
			// and if the leap motion device doesn't recognize a hand,
			// emergency message is published
			if (myo_gesture_before != myo_gesture
					&& leap_observer.getHandPosition()[0] <= EPSILON
					&& leap_observer.getHandPosition()[1] <= EPSILON
					&& leap_observer.getHandPosition()[3] <= EPSILON) {
				printf("Emergency\n");
				reset_pub.publish(emptyMsg);
				flying = false;
			}
			break;
			// Land
		case MyoObserver::TIP:
			if (myo_gesture_before != myo_gesture) {
				printf("Land\n");
				land_pub.publish(emptyMsg);
				flying = false;
			}
			break;
		}
	} else {
		switch (myo_gesture) {
		case MyoObserver::INITIAL:
			break;
		case MyoObserver::FIST:
			break;
			// Flattrim
		case MyoObserver::LEFT:
			if (myo_gesture_before != myo_gesture) {
				printf("Flattrim\n");
				flattrim_srv.call(flattrim_srv_srvs);
			}
			break;
		case MyoObserver::RIGHT:
			break;
			// Reset
		case MyoObserver::SPREAD:
			if (myo_gesture_before != myo_gesture) {
				printf("Reset\n");
				reset_pub.publish(emptyMsg);
			}
			break;
			// Take off
		case MyoObserver::TIP:
			if (myo_gesture_before != myo_gesture) {
				printf("Start\n");
				takeoff_pub.publish(emptyMsg);
				flying = true;
			}
			break;
		}
	}
	myo_gesture_before = myo_gesture;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "myo_pose");

	ros::NodeHandle nh;

	ros::Publisher takeoff_pub = nh.advertise<std_msgs::Empty>(
			"/ardrone/takeoff", 1, true);
	ros::Publisher land_pub = nh.advertise<std_msgs::Empty>("/ardrone/land", 1,
			true);
	ros::Publisher reset_pub = nh.advertise<std_msgs::Empty>("/ardrone/reset",
			1, true);
	ros::Publisher steer_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",
			1);

	ros::ServiceClient flattrim_srv = nh.serviceClient<std_srvs::Empty>(
			"ardrone/flattrim", 1);

	OculusObserver oculus_observer;
	MyoObserver myo_observer;
	LeapObserver leap_observer;

	ros::Subscriber sub_oculus = nh.subscribe("/oculus/orientation", 50,
			&OculusObserver::oculus_callback, &oculus_observer);
	ros::Subscriber sub_myo = nh.subscribe("/myo/data/thalmic_gesture", 1,
			&MyoObserver::myo_callback, &myo_observer);
	ros::Subscriber sub_leap = nh.subscribe("/leapmotion/pose", 1,
			&LeapObserver::leap_callback_pose, &leap_observer);
	/*	ros::Subscriber myo_sub = nh.subscribe<std_msgs::UInt8>("myo_gest", 50,
	 boost::bind(myo_callback, _1, oculus_observer, flattrim_srv,
	 takeoff_pub, land_pub, reset_pub, steer_pub));
	 */

	ros::Rate loop_rate(10);

	while (myo_observer.getHandPose() == MyoObserver::NOT_CONNECTED) {
		ros::spinOnce();
	}

	while (ros::ok) {
		send_control(myo_observer.getHandPose(), oculus_observer, leap_observer,
				flattrim_srv, takeoff_pub, land_pub, reset_pub, steer_pub);
		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::shutdown();
	/*	while(ros::ok()) {
	 ros::spinOnce();
	 printf("in while: y: %f\n", oculus_observer.getY());
	 loop_rate.sleep();
	 }*/

	return 0;
}
