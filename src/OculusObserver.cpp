/*
 * OculusObserver.cpp
 *
 *  Created on: Nov 10, 2015
 *      Author: steve
 *
 *      For the detection of nodding and head shaking the work from Katsuomi Kobayashi
 *      which can be found at https://github.com/KatsuomiK/RiftGesture helped a lot.
 */

#include <algorithm>
#include <cstdlib>

#include <ardrone_gesture_control/OculusObserver.h>
#include <ardrone_gesture_control/time_ms.h>
#include <geometry_msgs/Quaternion.h>

#define NOD_THRESHOLD 0.2
#define SHAKE_THRESHOLD 0.2
#define BASE_POSITION_THRESHOLD 0.1
#define YAW_THRESHOLD 0.4 //Turn head left or right (negative)

OculusObserver::OculusObserver() :
		x(0), y(0), z(0), w(0), gesture(0) {
}

OculusObserver::~OculusObserver() {
}

void OculusObserver::oculus_callback(
		const geometry_msgs::Quaternion::ConstPtr& sensor_data) {
	x = sensor_data->x;
	y = sensor_data->y;
	z = sensor_data->z;
	w = sensor_data->w;

	Sample tmp(x, y, z, w, time_ms());

	oculus_data.push_front(tmp);
	if (oculus_data.size() > 120) {
		oculus_data.pop_back();
	}

	// Detect gestures
	if (time_ms() - start_time_wait > 500) {
		if (detecteNod()) {
			gesture = 1;
		} else if (detecteHeadShake()) {
			gesture = 2;
		} else if (y > YAW_THRESHOLD) {
			gesture = 3;
		} else if (y < -YAW_THRESHOLD) {
			gesture = 4;
		} else {
			gesture = 0;
		}
	}

//	std::cout << "Gesture: " << gesture << std::endl;
}

float OculusObserver::getW() const {
	return w;
}

float OculusObserver::getX() const {
	return x;
}

float OculusObserver::getY() const {
	return y;
}

float OculusObserver::getZ() const {
	return z;
}

int OculusObserver::getGesture() const {
	return gesture;
}

bool OculusObserver::detecteNod() {
	bool nod = false;

	std::vector<int> range = getRange(200, 400);
	float basePos = 0.0;
	for (int i = range[0]; i <= range[1]; ++i) {
		basePos += oculus_data[i].x;
	}
	basePos /= (range[1] - range[0] + 1);

	range = getRange(10, 200);
	float xMax = -10.0;
	for (int i = range[0]; i <= range[1]; ++i) {
		if (oculus_data[i].x > xMax) {
			xMax = oculus_data[i].x;
		}
	}

	float current = oculus_data.front().x;

	if (xMax - basePos > NOD_THRESHOLD
			&& fabs(current - basePos) < BASE_POSITION_THRESHOLD) {
		nod = true;
		start_time_wait = time_ms();
	}
	return nod;
}

bool OculusObserver::detecteHeadShake() {
	bool headShake = false;

	std::vector<int> range = getRange(200, 400);
	float basePos = 0.0;
	for (int i = range[0]; i <= range[1]; ++i) {
		basePos += oculus_data[i].y;
	}
	basePos /= (range[1] - range[0] + 1);

//	std::cout << "range: " << range[0] << " " << range[1] << ", basePos: " << basePos << std::endl;

	range = getRange(10, 200);
	float yMax = -10.0;
	for (int i = range[0]; i <= range[1]; ++i) {
		if (oculus_data[i].y > yMax) {
			yMax = oculus_data[i].y;
		}
	}

	float yMin = 10.0;
	for (int i = range[0]; i <= range[1]; ++i) {
		if (oculus_data[i].y < yMin) {
			yMin = oculus_data[i].y;
		}
	}

	float current = oculus_data.front().y;

	if ((yMax - basePos > SHAKE_THRESHOLD || basePos - yMin > SHAKE_THRESHOLD)
			&& fabs(current - basePos) < BASE_POSITION_THRESHOLD) {
		headShake = true;
		start_time_wait = time_ms();
	}
	return headShake;
}

std::vector<int> OculusObserver::getRange(unsigned long long start_time,
		unsigned long long end_time) const {
	int begin = 0;
	int end = 0;
	for (int i = 0; i < oculus_data.size(); ++i) {
		if (oculus_data[i].timestamp < (time_ms() - start_time) && begin == 0) {
			begin = i;
		} else if (oculus_data[i].timestamp >= (time_ms() - end_time)) {
			end = i;
		}
	}
	return std::vector<int>( { begin, end });
}
