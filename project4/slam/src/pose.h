#ifndef _pose_h
#define _pose_h

#include <ros/ros.h>
#include "CovMatrix.h"
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;

class Pose {
	public:
		float x;
		float y;
		float t;

		float weight;
};

#endif
