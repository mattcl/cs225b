#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <gradient_planner/GPCostmapM.h>
#include <stdlib.h>
#include "math.h"

const float MAX_RGB = 1.0f;
const float MAX_HUE_VALUE = 270.0f;
struct g_hsv {
	float h;
	float s;
	float v;
};

// define the percent of points to send to rviz
int nPointsDist;
int nPointsCost;
int nPointsLike;

// define messages to dispay distance, cost, path
visualization_msgs::Marker distancePoints;
visualization_msgs::Marker totalDistPoints;
visualization_msgs::Marker costPoints;
visualization_msgs::Marker likePoints;
visualization_msgs::Marker path;


// Callback functions for obstacle distane and costmap
void subscribeDistCallback(const gradient_planner::GPCostmapMConstPtr &msg);
void subscribeTotalDistCallback(const gradient_planner::GPCostmapMConstPtr &msg);
void subscribeCostCallback(const gradient_planner::GPCostmapMConstPtr &msg);
void subscribePathCallback(const sensor_msgs::PointCloudConstPtr &msg);
void subscribeLikeCallback(const gradient_planner::GPCostmapMConstPtr &msg);

// Colour conversion from HSV to RGB, setting RGB value and computing a colour for a given value
void HSVToRGB(struct g_hsv & hsv, std_msgs::ColorRGBA & rgb);
void setRGB(std_msgs::ColorRGBA & rgb, float r, float g, float b);
void computeColorForValue(std_msgs::ColorRGBA & color, float gradient_value, float max_value);

bool received_distance_map;
bool received_total_map;
bool received_cost_map;
bool received_likelihood_field;
bool received_path;

int main(int argc, char** argv) {
	nPointsDist = nPointsCost = nPointsLike = 1;
	if(argc > 1) {
		nPointsDist = atoi(argv[1]);
		if(argc > 2) {
			nPointsCost = atoi(argv[2]);
			if(argc > 3) {
				nPointsLike = atoi(argv[3]);
			}
		}
	}
	ROS_INFO("Will send every %d point(s) for the distance function to rviz", nPointsDist);	
	ROS_INFO("Will send every %d point(s) for the costmap to rviz", nPointsCost);

	ros::init(argc, argv, "gradient_visualizer");
	ros::NodeHandle n;
	ros::Rate r(1);
	ros::Subscriber distance_sub = n.subscribe("object_distance_fn", 100, &subscribeDistCallback);
	ros::Subscriber total_dist_sub = n.subscribe("total_object_distance_fn", 100, &subscribeTotalDistCallback);
	ros::Subscriber cost_sub = n.subscribe("costmap_fn", 100, &subscribeCostCallback);
	ros::Subscriber like_sub = n.subscribe("likelihood_field", 100, &subscribeLikeCallback);
	ros::Subscriber path_sub = n.subscribe("computed_path", 100, &subscribePathCallback);
	ros::Publisher gradient_pub = n.advertise<visualization_msgs::Marker>("gradient_marker", 10);
	
	received_distance_map = false;
	distancePoints.header.frame_id = "/map";
	distancePoints.ns = "distance_function";
	distancePoints.action = visualization_msgs::Marker::ADD;
	distancePoints.pose.orientation.w = 1.0;
	distancePoints.color.a = 1.0;
	distancePoints.id = 0;
	distancePoints.type = visualization_msgs::Marker::POINTS;
	distancePoints.scale.x = .1;
	distancePoints.scale.y = .1;
	
	received_total_map = false;
	totalDistPoints.header.frame_id = "/map";
	totalDistPoints.ns = "total_distance_function";
	totalDistPoints.action = visualization_msgs::Marker::ADD;
	totalDistPoints.pose.orientation.w = 1.0;
	totalDistPoints.color.a = 1.0;
	totalDistPoints.id = 0;
	totalDistPoints.type = visualization_msgs::Marker::POINTS;
	totalDistPoints.scale.x = .1;
	totalDistPoints.scale.y = .1;

	received_cost_map = false;
	costPoints.header.frame_id = "/map";
	costPoints.ns = "cost_function";
	costPoints.action = visualization_msgs::Marker::ADD;
	costPoints.pose.orientation.w = 1.0;
	costPoints.color.a = 1.0;
	costPoints.id = 0;
	costPoints.type = visualization_msgs::Marker::POINTS;
	costPoints.scale.x = .1;
	costPoints.scale.y = .1;

	received_likelihood_field= false;
	likePoints.header.frame_id = "/map";
	likePoints.ns = "likelihood_function";
	likePoints.action = visualization_msgs::Marker::ADD;
	likePoints.pose.orientation.w = 1.0;
	likePoints.color.a = 1.0;
	likePoints.id = 0;
	likePoints.type = visualization_msgs::Marker::POINTS;
	likePoints.scale.x = .1;
	likePoints.scale.y = .1;
	
	received_path = false;
	path.header.frame_id = "/map";
	path.ns = "computed_path";
	path.action = visualization_msgs::Marker::ADD;
	path.pose.orientation.w = 1.0;
	path.color.a = 1.0;
	path.color.r = 1.0;
	path.id = 0;
	path.type = visualization_msgs::Marker::LINE_LIST;
	path.scale.x = 0.1;

	while(ros::ok()) {
		if(received_distance_map) {
			distancePoints.header.stamp = ros::Time::now();
			gradient_pub.publish(distancePoints);
		}
		if(received_total_map) {
			totalDistPoints.header.stamp = ros::Time::now();
			gradient_pub.publish(totalDistPoints);
		}
		if(received_cost_map) {
			costPoints.header.stamp = ros::Time::now();
			gradient_pub.publish(costPoints);
		}
		if(received_likelihood_field) {
			likePoints.header.stamp = ros::Time::now();
			gradient_pub.publish(likePoints);
		}
		if(received_path) {
			path.header.stamp = ros::Time::now();
			gradient_pub.publish(path);
		}
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}

void subscribeDistCallback(const gradient_planner::GPCostmapMConstPtr &msg) {
	int rows = msg->rows;
	int cols = msg->cols;
	int size = rows * cols;
	float max = -1.0;
	ROS_INFO("distance function msg received: %d rows %d cols", rows, cols);
	for(int i = 0; i < size; i++)
		if(msg->data[i] > max)
			max = msg->data[i];
	distancePoints.points.clear();
	distancePoints.colors.clear();

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
		if(col % nPointsDist > 0 || row % nPointsDist > 0)
			continue;
		point.y = (float) (i / rows) / 10.0;
		point.x = (float) (i % rows) / 10.0;
		point.z = 0;
		distancePoints.points.push_back(point);
		distancePoints.colors.push_back(color);
	}
	ROS_INFO("DIST FN Sending %d of %d points, max value %f", size / nPointsDist, size, max);
	received_distance_map = true;
}

void subscribeTotalDistCallback(const gradient_planner::GPCostmapMConstPtr &msg) {
	int rows = msg->rows;
	int cols = msg->cols;
	int size = rows * cols;
	float max = -1.0;
	ROS_INFO("total distance function msg received: %d rows %d cols", rows, cols);
	for(int i = 0; i < size; i++)
		if(msg->data[i] > max)
			max = msg->data[i];
	totalDistPoints.points.clear();
	totalDistPoints.colors.clear();

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
		if(col % nPointsDist > 0 || row % nPointsDist > 0)
			continue;
		point.y = (float) (i / rows) / 10.0;
		point.x = (float) (i % rows) / 10.0;
		point.z = 0;
		totalDistPoints.points.push_back(point);
		totalDistPoints.colors.push_back(color);
	}
	ROS_INFO("TOTAL DIST FN Sending %d of %d points, max value %f", size / nPointsDist, size, max);
	received_total_map= true;
}

void subscribeCostCallback(const gradient_planner::GPCostmapMConstPtr &msg) {
	int rows = msg->rows;
	int cols = msg->cols;
	int size = rows * cols;
	float max = -1.0;
	ROS_INFO("costmap function msg received: %d rows %d cols", rows, cols);
	ROS_INFO("Changing values of points not reached to -1");

	for(int i = 0; i < size; i++)
		if(msg->data[i] > max && msg->data[i] != 100000000)
			max = msg->data[i];
	costPoints.points.clear();
	costPoints.colors.clear();
	ROS_INFO("max value = %f",max);
	int num = 0;
	for(int i = 0; i < size; i++) {
		std_msgs::ColorRGBA color;
		float value = msg->data[i];
		if(value < 0 || value == 100000000) {
			continue;
		}
		
		computeColorForValue(color, value, max);
		geometry_msgs::Point point;
		int row = i % rows;
		int col = i % cols;
		if(col % nPointsCost > 0 || row % nPointsCost > 0)
			continue;
		num++;
		point.y = (float) (i / rows) / 10.0;
		point.x = (float) (i % rows) / 10.0;
		point.z = 0;
		costPoints.points.push_back(point);
		costPoints.colors.push_back(color);
	}
	ROS_INFO("COST FN Sending %d of %d points, max value %f", num, size, max);
	received_cost_map = true;
}

void subscribeLikeCallback(const gradient_planner::GPCostmapMConstPtr &msg) {
	int rows = msg->rows;
	int cols = msg->cols;
	int size = rows * cols;
	float max = -1.0;
	ROS_INFO("likelihood field function msg received: %d rows %d cols", rows, cols);
	ROS_INFO("Changing values of points not reached to -1");

	for(int i = 0; i < size; i++)
		if(msg->data[i] > max && msg->data[i] != 100000000)
			max = msg->data[i];
	likePoints.points.clear();
	likePoints.colors.clear();
	ROS_INFO("max value = %f",max);
	int num = 0;
	for(int i = 0; i < size; i++) {
		std_msgs::ColorRGBA color;
		float value = msg->data[i];
		if(value < 0 || value == 100000000) {
			continue;
		}
		
		computeColorForValue(color, value, max);
		geometry_msgs::Point point;
		int row = i % rows;
		int col = i % cols;
		if(col % nPointsLike > 0 || row % nPointsLike > 0)
			continue;
		num++;
		point.y = (float) (i / rows) / 10.0;
		point.x = (float) (i % rows) / 10.0;
		point.z = 0;
		likePoints.points.push_back(point);
		likePoints.colors.push_back(color);
	}
	ROS_INFO("LIKELIHOOD FN Sending %d of %d points, max value %f", num, size, max);
	received_likelihood_field = true;
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
