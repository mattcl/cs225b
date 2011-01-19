#ifndef _graph_h
#define _graph_h
#include <ros/ros.h>
#include <vector>
#include "CovMatrix.h"
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include "pose.h"
using namespace std;

class Graph {
	public:
	vector<Pose> poses;
	
	void getLastNScans(vector<Pose> &scans, int n);
	int getNumNodes();

	void PreparePublish(geometry_msgs::PoseArray poseArray, visualization_msgs::Marker lineList, vector<sensor_msgs::LaserScan> laserScans);
	void InsertPose(float x, float y, float t, double *mean, CovMatrix variance, int index, sensor_msgs::LaserScan scan);

};

#endif
