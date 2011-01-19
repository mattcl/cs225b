#ifndef _graph_h
#define _graph_h

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>

#include "pose.h"
#include "CovMatrix.h"

using namespace std;

struct node_t;

typedef struct link_t {
	node_t *parent;
	node_t *child;
	int parent_i;
	int child_i;
	double mean[3];
	double variance[3][3];
} link_t;

typedef struct node_t {
	Pose pose;
	sensor_msgs::LaserScan scan;
	vector<link_t*> links;
} node_t;

typedef struct bounds_t {
	int max_x, min_x, max_y, min_y;
} bounds_t;

class Graph {
	public:
	Graph();
	~Graph();

	int getNumNodes();
	int getNumLinks();
	void insertPose(Pose pose, sensor_msgs::LaserScan::ConstPtr &msg);
	void insertNode(node_t &node, node_t &parent, double mean[3], double variance[][3]);
	void getLastNNodes(int n, vector<node_t> &retnodes);
	bounds_t getSuggestedBounds();
	void drawGraph(ros::Publisher &graph_pub);
	void drawScans(ros::Publisher &scan_pub);

	private:
	node_t* root;
	vector<node_t> nodes;
	vector<link_t> links;
};

#endif
