//
// Created by steve on 11/6/16.
//

#include <ardrone_gesture_control/LeapObserver.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "leapmotion_test_node");
	ros::NodeHandle node;

	LeapObserver leapObserver;

	ros::Subscriber leapPose_sub;
	leapPose_sub = node.subscribe("/leapmotion/pose", 1,
	                                                   &LeapObserver::leap_callback_pose, &leapObserver);
	ros::Subscriber leapGrab_sub;
	leapGrab_sub = node.subscribe("/leapmotion/grab", 1,
	                                                  &LeapObserver::leap_callback_grab, &leapObserver);

	ros::Rate rate(1);
	while(ros::ok()) {
		ros::spinOnce();
		std::cout << leapObserver.getHandPose() << std::endl;
		rate.sleep();
	}
}