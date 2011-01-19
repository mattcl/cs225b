#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <gradient_planner/GPCostmapM.h>
#include <stdlib.h>
#include <vector>
#include <map>
#include "math.h"

using namespace std;

const float MAX_RGB = 1.0f;
const float MAX_HUE_VALUE = 270.0f;
struct g_hsv {
	float h;
	float s;
	float v;
};

map<string, visualization_msgs::Marker> markers;
visualization_msgs::Marker path;
bool received_path = false;
int grid_spacing = 1;

// Callback functions for obstacle distane and costmap
void subscribeCostmapCallback(const gradient_planner::GPCostmapMConstPtr &msg);
void subscribePathCallback(const sensor_msgs::PointCloudConstPtr &msg);

// Colour conversion from HSV to RGB, setting RGB value and computing a colour for a given value
void HSVToRGB(struct g_hsv & hsv, std_msgs::ColorRGBA & rgb);
void setRGB(std_msgs::ColorRGBA & rgb, float r, float g, float b);
void computeColorForValue(std_msgs::ColorRGBA & color, float gradient_value, float max_value);

int main(int argc, char** argv) {
	ros::init(argc, argv, "proj4_visualizer");
	ros::NodeHandle n;
	ros::Rate r(1);
	
	ros::Subscriber costmapSub = n.subscribe("costmap_visualizer", 100, &subscribeCostmapCallback);
	ros::Subscriber pathSub    = n.subscribe("path_visualizer", 100, &subscribePathCallback);	
	ros::Publisher costmapPub  = n.advertise<visualization_msgs::Marker>("costmap_markers", 10);
	ros::Publisher pathPub     = n.advertise<visualization_msgs::Marker>("path_markers", 10);

	n.getParam("proj4_visualizer", grid_spacing);

	while(ros::ok()) {
		map<string, visualization_msgs::Marker>::iterator end = markers.end();
		for(map<string, visualization_msgs::Marker>::iterator it = markers.begin(); it != end; it++) {
			it->second.header.stamp = ros::Time::now();
			costmapPub.publish(it->second);
		}
		if(received_path) {
			path.header.stamp = ros::Time::now();
			pathPub.publish(path);
		}
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}

void subscribeCostmapCallback(const gradient_planner::GPCostmapMConstPtr &msg) {
	int rows = msg->rows;
	int cols = msg->cols;
	int size = rows * cols;
	float max = -1.0;
	ROS_DEBUG("%s costmap msg received: %d rows %d cols",msg->name, rows, cols);
	for(int i = 0; i < size; i++)
		if(msg->data[i] > max)
			max = msg->data[i];
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/map";
	marker.ns = msg->name;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation.w = 1.0;
	marker.color.a = 1.0;
	marker.type = visualization_msgs::Marker::POINTS;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;

	for(int i = 0; i < size; i++) {
		std_msgs::ColorRGBA color;
		float value = msg->data[i];
		if(value < 0) {
			continue;
		}
		computeColorForValue(color, value, max);
		geometry_msgs::Point point;
		int row = i % rows;
		int col = i % cols;
		if(col % grid_spacing != 0 || row % grid_spacing != 0)
			continue;
		point.y = (float) (i / rows) / 10.0;
		point.x = (float) (i % rows) / 10.0;
		point.z = 0;
		marker.points.push_back(point);
		marker.colors.push_back(color);
	}
	markers[msg->name] = marker;
	ROS_DEBUG("Sending %d of %d points, max value %f", marker.points.size(), size, max);
}

void subscribePathCallback(const sensor_msgs::PointCloudConstPtr &msg) {
	ROS_INFO("path msg received with length: %d", msg->points.size());
	path.points.clear();	
	for(int i = 0; i < msg->points.size() - 1; i++) {
		geometry_msgs::Point point1;
		point1.y = msg->points[i].y;
		point1.x = msg->points[i].x;
		point1.z = 0;

		geometry_msgs::Point point2;
		point2.y = msg->points[i+1].y;
		point2.x = msg->points[i+1].x;
		point2.z = 0;
		path.points.push_back(point1);
		path.points.push_back(point2);	 
	}
	ROS_INFO("sending path");
	received_path = true;
}

// Functions to set colour of points while displaying obstacle map and cost map
void HSVToRGB(struct g_hsv & hsv, std_msgs::ColorRGBA & rgb) {
	float c = hsv.s * hsv.v;
	float hp = hsv.h / 60.0;
	float x = c * (1.0f - fabs(fmod(hp, 2.0f) - 1.0f));
	float m = hsv.v - c;
	if(hp < 0 || hp >= 6) {
		setRGB(rgb, 0, 0, 0);
	} else if (hp < 1) {
		setRGB(rgb, c, x, 0);
	} else if (hp < 2) {
		setRGB(rgb, x, c, 0);
	} else if (hp < 3) {
		setRGB(rgb, 0, c, x);
	} else if (hp < 4) {
		setRGB(rgb, 0, x, c);
	} else if (hp < 5) {
		setRGB(rgb, x, 0, c);
	} else {
		setRGB(rgb, c, 0, x);
	}
	rgb.r = (rgb.r + m) * MAX_RGB;
	rgb.g = (rgb.g + m) * MAX_RGB;
	rgb.b = (rgb.b + m) * MAX_RGB;
	//ROS_INFO("HSV %f, %f, %f, C %f, hp %f x %f, m %f, RGB %f, %f, %f", hsv.h, hsv.s, hsv.v, c, hp, x, m, rgb.r, rgb.g, rgb.b); 
}

void setRGB(std_msgs::ColorRGBA & rgb, float r, float g, float b) {
	rgb.r = r;
	rgb.g = g;
	rgb.b = b; 
}

void computeColorForValue(std_msgs::ColorRGBA & color, float gradientValue, float maxValue) { 
	color.a = 1.0;
	g_hsv hsv;
	hsv.h = (1 - gradientValue / maxValue) * MAX_HUE_VALUE;
	hsv.s = 1.0;
	hsv.v = 1.0;
	HSVToRGB(hsv, color);
}
