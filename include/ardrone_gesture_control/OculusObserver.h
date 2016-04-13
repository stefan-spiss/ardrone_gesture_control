/*
 * OculusObserver.h
 *
 *  Created on: Nov 10, 2015
 *      Author: steve
 */

#pragma once
#include <deque>
#include <vector>
#include <geometry_msgs/Quaternion.h>

struct Sample {
	unsigned long long timestamp;
	float x, y, z, w;

//	Sample(geometry_msgs::Quaternion::ConstPtr& sensor_data,
//			unsigned long long time) {
//		x = sensor_data->x;
//		y = sensor_data->y;
//		z = sensor_data->z;
//		w = sensor_data->w;
//		timestamp = time;
//	}
	Sample(float x, float y, float z, float w, unsigned long long time) {
			this->x = x;
			this->y = y;
			this->z = z;
			this->w = w;
			this->timestamp = time;
		}
};

class OculusObserver {
public:
	OculusObserver();
	~OculusObserver();
	void oculus_callback(
			const geometry_msgs::Quaternion::ConstPtr& sensor_data);
	float getW() const;
	float getX() const;
	float getY() const;
	float getZ() const;
	int getGesture() const;

private:
	float x, y, z, w;
	std::deque<Sample> oculus_data;
	int gesture;
	unsigned long long start_time_wait;
	bool detecteNod();
	bool detecteHeadShake();
	std::vector<int> getRange(unsigned long long start_time,
			unsigned long long end_time) const;
};
