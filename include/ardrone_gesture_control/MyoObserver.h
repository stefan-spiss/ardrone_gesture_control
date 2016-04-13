#pragma once
#include <std_msgs/Int32.h>

/**
 * Class used to get myo gestures from ros topic.
 */
class MyoObserver {
public:
	/**
	 * MYO standard gestures
	 */
	enum Myo_Gesture {
		NOT_CONNECTED = -1,//!< NOT_CONNECTED
		INITIAL = 0,       //!< INITIAL
		FIST	= 1,          //!< FIST
		LEFT	= 2,          //!< LEFT
		RIGHT	= 3,         //!< RIGHT
		SPREAD	= 4,        //!< SPREAD
		TIP		= 5           //!< TIP
	};
	MyoObserver();
	~MyoObserver();
	/**
	 * Ros callback function for myo standard gestures.
	 * @param command
	 */
	void myo_callback(const std_msgs::Int32::ConstPtr& command);

	/**
	 * Ros callback function for self trained myo gestures.
	 * @param command
	 */
	void myo_self_classified_callback(const std_msgs::Int32::ConstPtr& command);

	/**
	 * Return hand_pose which is one of the standard gestures
	 * @return myo standard gesture
	 */
	Myo_Gesture getHandPose() const;

	/**
	 * Return self_classified_pose which is one of the self trained gestures
	 * @return self classified gesture
	 */
	int getSelfClassifiedPose() const;

private:
	Myo_Gesture hand_pose;
	int self_classified_pose;
};
