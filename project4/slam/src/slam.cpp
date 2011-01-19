#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include "math.h"

#include "graph.h"
#include "pose.h"
//#include "nlsq.h"
#include "scan_matching.h"


using namespace std;

Graph *graph;
//NLSQ *solver;

bool pose_set, scan_set, pose_ok, origin_set;

Pose curPose;
sensor_msgs::LaserScanConstPtr scan;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
void loadParams(ros::NodeHandle &node);

int main(int argc, char** argv) {
	ros::init(argc, argv, "proj4_slam");
	ros::NodeHandle node;

	// load parameters
	loadParams(node);

	ros::Rate r(50);	
	ros::Subscriber odom_sub = node.subscribe("odom", 100, odomCallback);
	ros::Subscriber laser_sub = node.subscribe("base_scan", 100, laserCallback);

	ros::Publisher graph_pub = node.advertise<visualization_msgs::Marker>("graph_visualizer", 10);
	
	pose_set = scan_set = pose_ok = origin_set = false;

	graph = new Graph;

	while(ros::ok()) {
		ros::spinOnce();
		if(pose_set && scan_set && pose_ok) {
			ROS_INFO("graphing");
			graph->insertPose(curPose, scan);
			graph->drawGraph(graph_pub);
			graph->drawScans(graph_pub);
			origin_set = true; 
			pose_ok = false;
			scan_set = false;
		}
		r.sleep();
	}
	return 0;
}

void loadParams(ros::NodeHandle &node) {
	
}

float ComputePoseDistance(Pose* p1, Pose* p2) {
                if(p1 == NULL || p2 == NULL)
                        return 0;
                return (sqrt((p2->x-p1->x)*(p2->x-p1->x) + (p2->y-p1->y) * (p2->y-p1->y)));
        }


void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
	Pose nPos;
	nPos.x = msg->pose.pose.position.x;
	nPos.y = msg->pose.pose.position.y;
	nPos.t = atan2(2*msg->pose.pose.orientation.w*msg->pose.pose.orientation.z, 1-(2*msg->pose.pose.orientation.z*msg->pose.pose.orientation.z));

	if(pose_set && scan_set && origin_set && (ComputePoseDistance(&nPos, &curPose) < 0.3 || (curPose.x == nPos.x && curPose.y == nPos.y))) {
		pose_ok = false;
		return;
	}
	curPose = nPos;
	pose_ok = true;
	pose_set = true;
	ROS_DEBUG("using odom");
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
	ROS_DEBUG("got scan");
	scan = msg;
	scan_set = true;
}
