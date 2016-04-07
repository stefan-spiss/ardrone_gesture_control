/*
 * OculusObserver.cpp
 *
 *  Created on: Nov 10, 2015
 *      Author: steve
 */
#include <drone_control/OculusObserver.h>
#include <geometry_msgs/Quaternion.h>

OculusObserver::OculusObserver() :
		x(0), y(0), z(0), w(0) {
}

OculusObserver::~OculusObserver() {
}

void OculusObserver::oculus_callback(
		const geometry_msgs::Quaternion::ConstPtr& sensor_data) {
	x = sensor_data->x;
	y = sensor_data->y;
	z = sensor_data->z;
	w = sensor_data->w;
	//printf("x = %f y = %f z = %f\n", x, y, z);
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
