#include "graph.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include "pose.h"
#include "CovMatrix.h"
#include "scan_matching.h"
Graph::Graph() {
	root = NULL;
}

Graph::~Graph() {

}

int Graph::getNumNodes() {
	return nodes.size();
}

int Graph::getNumLinks() {
	return links.size();
}

void Graph::insertPose(Pose pose, sensor_msgs::LaserScan::ConstPtr &msg) {
	node_t node;
	node.pose = pose;
	node.scan.header = msg->header;
	node.scan.angle_min = msg->angle_min;
	node.scan.angle_max = msg->angle_max;
	node.scan.angle_increment = msg->angle_increment;
	node.scan.scan_time = msg->scan_time;
	node.scan.range_min = msg->range_min;
	node.scan.range_max = msg->range_max;
	node.scan.ranges = msg->ranges;
	node.scan.intensities = msg->intensities;
	
	nodes.push_back(node);
	if(nodes.size() > 1) {
		vector<node_t> referenceNodes;
		getLastNNodes(1, referenceNodes);
		ScanMatching matcher;
		double mean[3];
		double covariance[3][3];
		//matcher.match(nodes[nodes.size() - 1], referenceNodes, getSuggestedBounds(), mean, covariance); 
		insertNode(nodes[nodes.size() - 1], nodes[nodes.size() - 2], mean, covariance);
	}
}

void Graph::insertNode(node_t &node, node_t &parent, double mean[3], double variance[][3]) {
	link_t link;
	link.parent_i = nodes.size() - 2;
	link.child_i = nodes.size() - 1;
	links.push_back(link);
	for(int i = 0; i < 3; i++) {
		links[links.size() - 1].mean[i] = mean[i];
		for(int j = 0; j < 3; j++) {
			links[links.size() -1].variance[i][j] = variance[i][j];
		}
	}
	link = links[links.size() - 1];
	//ROS_INFO("adding link %f, %f, %f, %f", link.parent->pose.x, link.parent->pose.y, link.child->pose.x, link.child->pose.y);
}

void Graph::getLastNNodes(int n, vector<node_t> &retnodes) {
	for(int i = nodes.size() - 2; i >= nodes.size() - 1 - n && i >= 0; i--) {
		retnodes.push_back(nodes[i]);
	}
}

bounds_t Graph::getSuggestedBounds() {
	float max_x, min_x, max_y, min_y;
	max_x = min_x = max_y = min_y = 0;
	for(int i = 0; i < nodes.size(); i++) {
		Pose pose = nodes[i].pose;
		if(pose.x > max_x) max_x = pose.x;
		if(pose.y > max_y) max_y = pose.y;
		if(pose.x < min_x) min_x = pose.x;
		if(pose.y < min_y) min_y = pose.y;
	}
	
	min_x -= 20;
	min_y -= 20;
	max_x += 20;
	max_y += 20;

	bounds_t bounds;
	bounds.min_x = (int) (min_x * 10);
	bounds.max_x = (int) (max_x * 10);
	bounds.min_y = (int) (min_y * 10);
	bounds.max_y = (int) (max_y * 10);
	return bounds;
}

void Graph::drawGraph(ros::Publisher &graph_pub) {
	geometry_msgs::PoseArray poseArray;
	visualization_msgs::Marker nodes_m;
	visualization_msgs::Marker links_m;
	poseArray.header.frame_id = nodes_m.header.frame_id = links_m.header.frame_id = "/odom";
	nodes_m.scale.x = 0.05;
	nodes_m.scale.y = 0.05;
	links_m.scale.x = 0.05;
	links_m.scale.y = 0.05;
	nodes_m.color.r = 1.0;
	links_m.color.b = 1.0;
	nodes_m.color.a = links_m.color.a = 1.0;
	nodes_m.type = visualization_msgs::Marker::POINTS;
	links_m.type = visualization_msgs::Marker::LINE_LIST;
	nodes_m.ns = "graph_nodes";
	links_m.ns = "graph_edges";

	for(int i = 0; i < nodes.size(); i++) {
		geometry_msgs::Pose insert;
		insert.position.x = nodes[i].pose.x;
		insert.position.y = nodes[i].pose.y;
		insert.orientation.w = (sqrt(2+2*cos(nodes[i].pose.t)))/2;
                insert.orientation.z = (1*sin(nodes[i].pose.t))/(2*insert.orientation.w);
                poseArray.poses.push_back(insert);
		geometry_msgs::Point point;
		point = insert.position;
		nodes_m.points.push_back(point);
	}

	for(int i = 0; i < links.size(); i++) {
		geometry_msgs::Point start;
		geometry_msgs::Point end;
		//Pose pose = links[i].parent->pose;
		Pose pose = nodes[links[i].parent_i].pose;
		start.x = pose.x;
		start.y = pose.y;
		//pose = links[i].child->pose;
		pose = nodes[links[i].child_i].pose;
		end.x = pose.x;
		end.y = pose.y;
		links_m.points.push_back(start);
		links_m.points.push_back(end);
//		ROS_INFO("graph %f, %f, %f, %f", links[i].parent->pose.x, links[i].parent->pose.y, links[i].child->pose.x, links[i].child->pose.y);
	}

	graph_pub.publish(nodes_m);
	graph_pub.publish(links_m);
	//graph_pub.publish(poseArray);
}

void Graph::drawScans(ros::Publisher &scan_pub) {
	visualization_msgs::Marker scans;
	scans.header.frame_id = "/odom";
	scans.scale.x = 0.005;
	scans.scale.y = 0.005;
	scans.color.g = 1.0;
	scans.color.a = 1.0;
	scans.ns = "scans";
	scans.type = visualization_msgs::Marker::POINTS;
	for(int i = 0; i < nodes.size(); i++) {
		float angle_min = nodes[i].pose.t + nodes[i].scan.angle_min;
		float angle_max = nodes[i].pose.t + nodes[i].scan.angle_max;
		float angle_increment = nodes[i].scan.angle_increment;
		float cur_angle = angle_min;

		for(int j = 0; cur_angle <= angle_max - angle_increment && j < 180; j = j + 1, cur_angle = cur_angle + angle_increment) {
			float dist = nodes[i].scan.ranges[j];

			if(dist == 12.0)
				continue;

			float x = nodes[i].pose.x + cos(cur_angle) * dist;
			float y = nodes[i].pose.y + sin(cur_angle) * dist;
			geometry_msgs::Point pt;
			pt.x = x;
			pt.y = y;
			scans.points.push_back(pt);
		}

	}
	scan_pub.publish(scans);
}
