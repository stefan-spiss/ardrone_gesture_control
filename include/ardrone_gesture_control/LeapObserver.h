#pragma once
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>

class LeapObserver {
public:
	/**
	 * Leap Hand Gestures
	 */
	enum Leap_Hand_Gestures {
		NO_HAND = -2,			//!< not connected
		NOTHING_DETECTED = -1, 		//!< no valid gesture detected
		MYO_INITIAL = 0,     //!< hold vertical hand (myo standard initial pose)
		FIST = 1,          			//!< hold fist
		MYO_LEFT_SWIPE = 2,
		MYO_RIGHT_SWIPE = 3,  //!< swipe right with vertical hand (myo standard)
		FLAT_HAND = 10,        		//!< hold just a flat hand
		FLAT_HAND_SWIPE_RIGHT = 13	//!< swipe right with flat hand
	};
	LeapObserver();
	~LeapObserver();
	void leap_callback_pose(const geometry_msgs::Pose::ConstPtr& pose_data);
	void leap_callback_dir(const geometry_msgs::Vector3::ConstPtr& dir_data);
	void leap_callback_normal(
			const geometry_msgs::Vector3::ConstPtr& normal_data);
	void leap_callback_grab(const std_msgs::Float32::ConstPtr& grab_data);
	const float* getHandPosition() const;
	const float getPitch() const;
	const float getYaw() const;
	const float getRoll() const;
	const float* getDirection() const;
	const float* getNormal() const;
	const float getGrab() const;
	const Leap_Hand_Gestures getHandPose() const;

private:
	// LeapMotion MYO gestures
#define GESTURE_FIST_GRAB 0.9

#define GESTURE_MYO_PITCH_MIN -0.4
#define GESTURE_MYO_PITCH_MAX 0.7
#define GESTURE_MYO_LEFT_SWIPE_YAW -0.7
#define GESTURE_MYO_RIGHT_SWIPE_YAW 0.7
#define GESTURE_MYO_ROLL_MIN -2.1
#define GESTURE_MYO_ROLL_MAX -0.6

	// LeapMotion flat hand gestures
#define GESTURE_FLAT_HAND_PITCH_MIN -0.4
#define GESTURE_FLAT_HAND_PITCH_MAX 0.7
#define GESTURE_FLAT_HAND_YAW 0.4
#define GESTURE_FLAT_HAND_RIGHT_SWIPE_YAW 0.45
#define GESTURE_FLAT_HAND_ROLL 0.4

	// LeapMotion Gesture D
	// same as Gesture A
	float orientation[4];
	float hand_position[3];
	float direction[3];
	float normal[3];
	float grab;
};
