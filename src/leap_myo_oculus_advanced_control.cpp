/*
 * leap_myo_oculus_advanced_control.cpp
 *
 *  Created on: Apr 13, 2016
 *      Author: steve
 */

#include <ardrone_gesture_control/LeapMyoOculusDroneControl.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "leap_myo_oculus_drone_control_node");
	ros::NodeHandle node;
	ros::NodeHandle local_node("~");

	std::string takeoffTopic = "/ardrone/takeoff";
	std::string landTopic = "/ardrone/land";
	std::string resetTopic = "/ardrone/reset";
	std::string steerTopic = "/cmd_vel";
	std::string droneStateTopic = "/ardrone/navdata";
	std::string flatTrimSrv = "ardrone/flattrim";
	std::string myoTopic = "/myo/data/thalmic_gesture";
	std::string leapTopicPose = "/leapmotion/pose";
	std::string leapGrabTopic = "/leapmotion/grab";
	std::string oculusTopic = "oculus/orientation";
	bool travelAllowed = true;

	if (local_node.getParam("takeoff_topic", takeoffTopic)) {
		std::cout << "Set parameter takeoff_topic to " << takeoffTopic
				<< std::endl;
	}
	if (local_node.getParam("land_topic", landTopic)) {
		std::cout << "Set parameter land_topic to " << landTopic << std::endl;
	}
	if (local_node.getParam("reset_topic", resetTopic)) {
		std::cout << "Set parameter reset_topic to " << resetTopic << std::endl;
	}
	if (local_node.getParam("steer_topic", steerTopic)) {
		std::cout << "Set parameter steer_topic to " << steerTopic << std::endl;
	}
	if (local_node.getParam("droneState_topic", droneStateTopic)) {
		std::cout << "Set parameter droneState_topic to " << droneStateTopic
				<< std::endl;
	}
	if (local_node.getParam("flatTrim_srv", flatTrimSrv)) {
		std::cout << "Set parameter flatTrim_srv to " << flatTrimSrv
				<< std::endl;
	}
	if (local_node.getParam("myo_topic", myoTopic)) {
		std::cout << "Set parameter myo_topic to " << myoTopic << std::endl;
	}
	if (local_node.getParam("leapPose_topic", leapTopicPose)) {
		std::cout << "Set parameter leapPose_topic to " << leapTopicPose
				<< std::endl;
	}
	if (local_node.getParam("leapGrab_topic", leapGrabTopic)) {
		std::cout << "Set parameter leapGrab_topic to " << leapGrabTopic
				<< std::endl;
	}
	if (local_node.getParam("oculus_topic", oculusTopic)) {
		std::cout << "Set parameter oculus_topic to " << oculusTopic
				<< std::endl;
	}
	if(local_node.getParam("travel_allowed", travelAllowed)) {
		std::cout << "Set parameter travelAllowed to " << travelAllowed << std::endl;
	}
	LeapMyoOculusDroneControl droneController(node, takeoffTopic, landTopic,
			resetTopic, steerTopic, droneStateTopic, flatTrimSrv, myoTopic,
			leapTopicPose, leapGrabTopic, oculusTopic, travelAllowed);

	droneController.controlDrone();

}
