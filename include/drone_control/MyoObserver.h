#pragma once
#include <std_msgs/UInt8.h>

class MyoObserver {
public:
	enum Myo_Gesture {
		NOT_CONNECTED = -1,
		INITIAL = 0,
		FIST	= 1,
		LEFT	= 2,
		RIGHT	= 3,
		SPREAD	= 4,
		TIP		= 5
	};
	MyoObserver();
	~MyoObserver();
	void myo_callback(const std_msgs::UInt8::ConstPtr& command);
	Myo_Gesture getHandPose();
	bool isChanged() const;

private:
	Myo_Gesture hand_pose;
	bool changed;
};
