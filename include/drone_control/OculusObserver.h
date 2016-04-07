/*
 * OculusObserver.h
 *
 *  Created on: Nov 10, 2015
 *      Author: steve
 */

#pragma once
#include <geometry_msgs/Quaternion.h>

class OculusObserver {
public:
	OculusObserver();
	~OculusObserver();
	void oculus_callback(const geometry_msgs::Quaternion::ConstPtr& sensor_data);
	float getW() const;
	float getX() const;
	float getY() const;
	float getZ() const;

private:
	float x, y, z, w;
};
