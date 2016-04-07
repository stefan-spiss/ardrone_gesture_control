#pragma once
#include <geometry_msgs/Pose.h>

class LeapObserver {
public:
	LeapObserver();
	~LeapObserver();
	void leap_callback_pose(const geometry_msgs::Pose::ConstPtr& pose_data);
	const float* getHandPosition() const;
	const float getPitch() const;
	const float getYaw() const;
	const float getRoll() const;

private:
	float orientation[4];
	float hand_position[3];
};
