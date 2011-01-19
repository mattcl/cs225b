#include <ros/ros.h>
#include <stdlib.h>
#include "graph.h"
#include "pose.h"
#include "CovMatrix.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>

using namespace std;

void getLastNScans(vector<Pose> & scans, int n) {
	for(int i = poses.size() - 1; i >= 0 && i >= poses.size() - n; i--) {
		scans.push_back(poses[i]);
	}
}

int Graph::getNumNodes() {
	return poses.size();
}



void Graph::PreparePublish(geometry_msgs::PoseArray poseArray, visualization_msgs::Marker lineList, vector<sensor_msgs::LaserScan> laserScans) {
	poseArray.poses.clear();
	poseArray.header.frame_id = "/odom";
	lineList.points.clear();
	lineList.header.frame_id = "/odom";
	lineList.type = visualization_msgs::Marker::LINE_LIST;
	lineList.ns = "graph_edges";
	lineList.scale.x = 0.2;
	lineList.color.b = 1.0;
	lineList.color.a = 1.0;


	for(int i = 0; i < poses.size(); ++i) {
		geometry_msgs::Pose insert;
		insert.position.x = poses[i].x;
		insert.position.y = poses[i].y;
		insert.orientation.w = (sqrt(2+2*cos(poses[i].t)))/2;
		insert.orientation.z = (1*sin(poses[i].t))/(2*insert.orientation.w);
		poseArray.poses.push_back(insert);

		//After drawing the nodes, draw the edges
		//LineList takes 2 points per line
		//Edges goes from current node to a connected node
		for(int j = 0; j < index.size(); ++j) {
			geometry_msgs::Point p;
			p.x = poses[i].x;
			p.y = poses[i].y;
			lineList.points.push_back(p);
			p.x = poses[j].x;
			p.y = poses[j].y;
			lineList.points.push_back(p);
		}

		//Now hand over the laser scan for that node
		laserScans.push_back(poses[i].scan);
	}
}

void Graph::InsertPose(float x, float y, float t, double *mean, CovMatrix variance, int index, sensor_msgs::LaserScan scan) {
	Pose insert;
	insert.x = x;
	insert.y = y;
	insert.t = t;
	pose_link ln;
	ln.mean = mean;
	ln.variance = variance;
	ln.index = index;
	insert.links.push_back(ln);
	insert.scan = scan;

	poses.push_back(insert);
}
