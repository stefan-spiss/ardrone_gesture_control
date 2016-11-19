#include <ardrone_gesture_control/JoypadOculusControlTactileFeedback.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "joypad_oculus_control_node");
	ros::NodeHandle node;
	ros::NodeHandle local_node("~");

	std::string takeoffTopic = "/ardrone/takeoff";
	std::string landTopic = "/ardrone/land";
	std::string resetTopic = "/ardrone/reset";
	std::string steerTopic = "/cmd_vel";
	std::string droneStateTopic = "/ardrone/navdata";
	std::string flatTrimSrv = "ardrone/flattrim";
	std::string joypadAxisTopic = "/Gamepad/Axis";
	std::string joypadBtnTopic = "/Gamepad/Buttons";
	std::string oculusTopic = "oculus/orientation";
	std::string tactileMethodChangeSrv = "/tactile_feedback/changeMethod";
	bool travelAllowed = true;
	float maxDroneSpeedPitch = 0.25;
	float maxDroneSpeedRoll = 0.25;
	float maxDroneSpeedYaw = 0.5;
	float maxDroneSpeedUpDown = 0.5;

	if (local_node.getParam("takeoff_topic", takeoffTopic)) {
		std::cout << "Set parameter takeoff_topic to " << takeoffTopic << std::endl;
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
		std::cout << "Set parameter droneState_topic to " << droneStateTopic << std::endl;
	}
	if (local_node.getParam("flatTrim_srv", flatTrimSrv)) {
		std::cout << "Set parameter flatTrim_srv to " << flatTrimSrv << std::endl;
	}
	if (local_node.getParam("joypadAxis_topic", joypadAxisTopic)) {
		std::cout << "Set parameter myo_topic to " << joypadAxisTopic << std::endl;
	}
	if (local_node.getParam("joypadBtn_topic", joypadBtnTopic)) {
		std::cout << "Set parameter leapPose_topic to " << joypadBtnTopic << std::endl;
	}
	if (local_node.getParam("oculus_topic", oculusTopic)) {
		std::cout << "Set parameter oculus_topic to " << oculusTopic << std::endl;
	}
	if (local_node.getParam("travel_allowed", travelAllowed)) {
		std::cout << "Set parameter travel_allowed to " << travelAllowed << std::endl;
	}
	if (local_node.getParam("max_drone_speed_pitch", maxDroneSpeedPitch)) {
		std::cout << "Set parameter max_drone_speed_pitch to " << maxDroneSpeedPitch << std::endl;
	}
	if (local_node.getParam("max_drone_speed_roll", maxDroneSpeedRoll)) {
		std::cout << "Set parameter max_drone_speed_roll to " << maxDroneSpeedRoll << std::endl;
	}
	if (local_node.getParam("max_drone_speed_yaw", maxDroneSpeedYaw)) {
		std::cout << "Set parameter max_drone_speed_yaw to " << maxDroneSpeedYaw << std::endl;
	}
	if (local_node.getParam("max_drone_speed_updown", maxDroneSpeedUpDown)) {
		std::cout << "Set parameter max_drone_speed_updown to " << maxDroneSpeedUpDown << std::endl;
	}
	if (local_node.getParam("tactile_method_change_srv", tactileMethodChangeSrv)) {
		std::cout << "Set parameter tactile_method_change_srv to " << tactileMethodChangeSrv << std::endl;
	}

	JoypadOculusControlTactileFeedback droneController(node, takeoffTopic, landTopic, resetTopic, steerTopic,
	                                                   droneStateTopic, flatTrimSrv, joypadAxisTopic, joypadBtnTopic,
	                                                   oculusTopic, tactileMethodChangeSrv, travelAllowed,
	                                                   maxDroneSpeedPitch, maxDroneSpeedRoll, maxDroneSpeedYaw,
	                                                   maxDroneSpeedUpDown);

	droneController.controlDrone();

}

