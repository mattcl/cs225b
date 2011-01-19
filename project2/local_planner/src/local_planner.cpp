#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <gradient_planner/GPCostmapM.h>
#include <vector>
#include "math.h"

// "diamteter" is relative, it will be recangle with 
// w = 2 * WINDOW_DIAMETER and h = WINDOW_DIAMETER
const float WINDOW_DIAMETER = 1.0f;
const float BOT_WIDTH = 0.2f;

const int NUM_OFFSETS = 4;
const float OFFSETS[2][4] = {
	{0,0,1,1},
	{0,1,1,0},
};

const float MAX_VELOCITY = 0.3f;
const float MAX_ANGULAR_VELOCITY = 1;
const int NUM_ANGULAR_VELOCITIES = 13;
//const float ANGULAR_VELOCITIES[17] = {-1.0, -0.875, -0.75, -0.625, -0.5, -0.375, -0.25, -0.125, 0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1.0};
const float ANGULAR_VELOCITIES[NUM_ANGULAR_VELOCITIES] = {-0.75, -0.625, -0.5, -0.375, -0.25, -0.125, 0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75};
//Distance we want to keep from obstacles
const float CLEARANCE_DIST = 0.2;
//To get more search points, set this constant lower
const float SAMPLING_FACTOR = 0.7;

//Parameters for the cost function
//Cost = Alpha*DistanceToObstacles + Beta*DistanceToNextPoint
const float ALPHA = 1.0;
const float BETA = 5.0;

int bestTrajectory;

// local path vector
sensor_msgs::PointCloud pathMsg;
sensor_msgs::PointCloud localPath;

// markers
visualization_msgs::Marker botRect;
visualization_msgs::Marker searchWindow;

std::vector<sensor_msgs::PointCloud> computedTrajectories;

tf::TransformListener *transformListener;

//Laser message 
sensor_msgs::LaserScan laserMsg;
sensor_msgs::PointCloud localScan;
sensor_msgs::PointCloud scanCloud;
visualization_msgs::Marker debugMarker;

void setUpBotRect();
void setUpSearchWindow();
void subPathCallback(const sensor_msgs::PointCloud::ConstPtr &msg);
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
bool hasCollision(std::vector<geometry_msgs::Point32> points); // Check collision against laser data
void laserToPointCloud(const sensor_msgs::LaserScan::ConstPtr& msg);
void computeTrajectories();
float computeDistance(geometry_msgs::Point32 & p1, geometry_msgs::Point32 & p2);
float laserObstacleDistance(geometry_msgs::Point32 fromWhere);

int main(int argc, char** argv) {
	ros::init(argc, argv, "local_planner");
	ros::NodeHandle n;
	ros::Rate r(10);
	ros::Subscriber path_sub = n.subscribe("computed_path", 100, &subPathCallback);
	ros::Subscriber laser_sub = n.subscribe("base_scan", 100, &laserCallback);
	ros::Publisher local_planner_pub = n.advertise<visualization_msgs::Marker>("local_planner_marker", 10);
	ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	//DEBUG
	/*ros::Publisher debug_pub = n.advertise<visualization_msgs::Marker>("debug_channel",100);
	debugMarker.header.frame_id = "/base_footprint";
	debugMarker.ns = "debug";
	debugMarker.action = visualization_msgs::Marker::ADD;
	debugMarker.pose.orientation.w = 1.0;
	debugMarker.color.a = 1.0;
	debugMarker.color.b = 1.0;
	debugMarker.id = 0;
	debugMarker.type = visualization_msgs::Marker::POINTS;
	debugMarker.scale.x = .05;
	debugMarker.scale.y = .05; */
	//End Debug

	botRect.header.frame_id = searchWindow.header.frame_id = "/base_footprint";
	botRect.color.a = searchWindow.color.a = 1.0;
	botRect.color.r = searchWindow.color.r = 1.0;
	botRect.type = searchWindow.type = visualization_msgs::Marker::LINE_LIST;
	botRect.scale.x = searchWindow.scale.x = 0.01;
	botRect.scale.y = searchWindow.scale.y = 0.01;
	botRect.ns = "bot rectangle";
	searchWindow.ns = "search window";

	localPath.header.frame_id = "/base_footprint";
	localScan.header.frame_id = "/base_link";
	scanCloud.header.frame_id = "/base_footprint";

	transformListener = new tf::TransformListener();

	setUpBotRect();
	setUpSearchWindow();
	

	while(ros::ok()) {
		botRect.header.stamp = searchWindow.header.stamp = ros::Time::now();
		local_planner_pub.publish(botRect);
		local_planner_pub.publish(searchWindow);

		//DEBUG
		/*debugMarker.header.stamp = ros::Time::now();
		debug_pub.publish(debugMarker);
		debugMarker.points.clear(); */
		//END DEBUG
		
		if(!pathMsg.points.empty()) {
			computeTrajectories();
			for(unsigned int i = 0; i < computedTrajectories.size(); i++) {
				visualization_msgs::Marker trajectory;
				trajectory.header.frame_id = "/base_footprint";
				trajectory.header.stamp = ros::Time::now();
				trajectory.ns = "computed trajectories";
				trajectory.id = i;
				trajectory.type = visualization_msgs::Marker::LINE_LIST;
				trajectory.color.a = 1.0;
				trajectory.color.r = .20;
				trajectory.color.g = .20;
				trajectory.color.b = .20;
				if(i == bestTrajectory) {
					trajectory.color.r = 0;
					trajectory.color.g = 1.0;
					trajectory.color.b = 0;
				}
				trajectory.scale.x = 0.01;
				trajectory.scale.y = 0.01;
				for(unsigned int j = 0; j < computedTrajectories[i].points.size() - 1; j++) {
					geometry_msgs::Point pt1;
					pt1.x = computedTrajectories[i].points[j].x;
					pt1.y = computedTrajectories[i].points[j].y;
					trajectory.points.push_back(pt1);

					geometry_msgs::Point pt2;
					pt2.x = computedTrajectories[i].points[j+1].x;
					pt2.y = computedTrajectories[i].points[j+1].y;
					trajectory.points.push_back(pt2);
				}
				local_planner_pub.publish(trajectory);
			}
			if(computedTrajectories.size() > 0) {
				geometry_msgs::Twist cmdvel;
				cmdvel.linear.x = MAX_VELOCITY;
				cmdvel.angular.z = ANGULAR_VELOCITIES[bestTrajectory];
				velocity_pub.publish(cmdvel);
			} else {
				geometry_msgs::Twist cmdvel;
				cmdvel.linear.x = 0;
				cmdvel.angular.z = 0;
				velocity_pub.publish(cmdvel);
			}
		}
		
		ros::spinOnce();
		r.sleep();
	}
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
	laserToPointCloud(msg);
}

//Convert sensor_msgs::LaserScan(laserMsg) to PointCloud  (localScan)
void laserToPointCloud(const sensor_msgs::LaserScan::ConstPtr& msg) {
	int nPoints = ((msg->angle_max-msg->angle_min)/msg->angle_increment)+1;
	//ROS_INFO("LASER MIN:%f, MAX:%f\n", msg->range_min, msg->range_max);
	localScan.points.clear();
	for (int i = 0; i < nPoints; ++i) {
		geometry_msgs::Point32 temp;
		temp.x = msg->ranges[i]*cos(msg->angle_min+i*msg->angle_increment);
		temp.y = msg->ranges[i]*sin(msg->angle_min+i*msg->angle_increment);
		localScan.points.push_back(temp);
	}
}

void setUpSearchWindow() {
	std::vector<geometry_msgs::Point> points;
	for(int i = 0; i < NUM_OFFSETS; i++) {
		geometry_msgs::Point point;
		point.x = OFFSETS[0][i] * WINDOW_DIAMETER;
		point.y = -WINDOW_DIAMETER + OFFSETS[1][i] * (WINDOW_DIAMETER * 2);
		points.push_back(point);
	}
	
	for(unsigned int i = 0; i < points.size(); i++) {
		searchWindow.points.push_back(points[i]);
		searchWindow.points.push_back(points[(i + 1) % points.size()]);
	}
}

void setUpBotRect() {
	std::vector<geometry_msgs::Point> points;
	for(int i = 0; i < NUM_OFFSETS; i++) {
		geometry_msgs::Point point;
		point.x = -BOT_WIDTH + OFFSETS[0][i] * BOT_WIDTH * 2;
		point.y = -BOT_WIDTH + OFFSETS[1][i] * BOT_WIDTH * 2;
		points.push_back(point);
	}
	
	for(unsigned int i = 0; i < points.size(); i++) {
		botRect.points.push_back(points[i]);
		botRect.points.push_back(points[(i + 1) % points.size()]);
	}
}

void subPathCallback(const sensor_msgs::PointCloudConstPtr &msg) {
	//ROS_INFO("Received global path from planner");
	pathMsg.header.frame_id = msg->header.frame_id;
	pathMsg.points = msg->points;
}
void computeTrajectories() {
	//pathMsg.header.stamp = ros::Time::now();
	const sensor_msgs::PointCloud ptCloudIn = pathMsg;
	const sensor_msgs::PointCloud laserIn = localScan;
	scanCloud.points.clear();

	try {
		transformListener->transformPointCloud("/base_footprint", ptCloudIn, localPath);
		transformListener->transformPointCloud("/base_footprint", laserIn, scanCloud);

	} catch (tf::TransformException &ex) {
		ROS_ERROR("Could not transform path to base_footprint: %s\n", ex.what());
	} 
	
	computedTrajectories.clear();
	float yLimit = WINDOW_DIAMETER;

	geometry_msgs::Point32 rPos;

	if(computeDistance(rPos, localPath.points.back()) < 0.3) {
		return;
	} 

	// select the desired path point to reach for - currently on the next point on the path
	geometry_msgs::Point32 closest = localPath.points[0];
	float closestDist = closest.x * closest.x + closest.y * closest.y;
	int closestIndex = -999;
	for(int i = 0; i < localPath.points.size(); i++) {
		geometry_msgs::Point32 sample = localPath.points[i];
		float sampleDist = sample.x * sample.x + sample.y * sample.y;
		if(sampleDist < closestDist) {
			closestDist = sampleDist;
			closest = sample;
			closestIndex = i;
		} 	
	}

	ROS_INFO("closest point is (%f, %f) with index %d of %d", closest.x, closest.y, closestIndex, localPath.points.size());

	//target points to the closest goal point in the path - bug when targetIndex(thus target) goes back to lower index
	geometry_msgs::Point32 target = closest;
	int targetIndex = -999;
	for(targetIndex = closestIndex + 1; targetIndex < localPath.points.size(); targetIndex++) {
		geometry_msgs::Point32 sample = localPath.points[targetIndex];
		if(sample.x > WINDOW_DIAMETER || sample.y > WINDOW_DIAMETER || sample.y < -WINDOW_DIAMETER || sample.x < 0) {
			target = sample;		
			break;
		}
	}

	ROS_INFO("target point is (%f, %f) with index %d", target.x, target.y, targetIndex);
	
	for(int i = 0; i < NUM_ANGULAR_VELOCITIES; i++) {
		sensor_msgs::PointCloud trajectory;
		trajectory.header.frame_id = "/base_footprint";
		geometry_msgs::Point32 simLocation;
		float currentHeading = 0.0;
		while(true) {
			geometry_msgs::Point32 sample;
			currentHeading += SAMPLING_FACTOR*ANGULAR_VELOCITIES[i];
			sample.y = simLocation.y + SAMPLING_FACTOR*MAX_VELOCITY * sin(currentHeading);
			sample.x = simLocation.x + SAMPLING_FACTOR*MAX_VELOCITY * cos(currentHeading);
			trajectory.points.push_back(sample);
			if(sample.x > WINDOW_DIAMETER || sample.y > WINDOW_DIAMETER || sample.y < -WINDOW_DIAMETER || sample.x < 0)
				break;
			
			if(computeDistance(sample, target) < 0.2)
				break;

			simLocation = sample;
		}
		computedTrajectories.push_back(trajectory);
	}

	ROS_INFO("computed %d trajectories", computedTrajectories.size());

	float bestCost = -1.0;
	bestTrajectory = 0;
	for(int i = 0; i < computedTrajectories.size(); i++) {
		//float cost = computeDistance(computedTrajectories[i].points.back(), target); //Old cost fn
		float cost = 0;
		for(int j = 0; j < computedTrajectories[i].points.size(); ++j) {
			if(hasCollision(computedTrajectories[i].points)) {
				cost = 1e8;
				//ROS_INFO("Collision on Traj:%d", i);
				break;
			}  
			//No collision, then check for distances to obstacles&path
			cost += ALPHA*(1/sqrt(laserObstacleDistance(computedTrajectories[i].points[j]))) 
					+ BETA*computeDistance(computedTrajectories[i].points[j], target);
		}	
		//float cost = ALPHA*(1/sqrt(laserObstacleDistance(computedTrajectories[i].points.back()))) + BETA*computeDistance(computedTrajectories[i].points.back(), target); //Experimental cost fn
		ROS_INFO("cost of traj[%d] is %f", i, cost);		
		
		if(cost < bestCost || bestCost < 0) {
			bestTrajectory = i;
			bestCost = cost;			
		}
	}
}
int indexCol = -1;
//Seems like X-axis is forward, Y-axis toward left of robot
float laserObstacleDistance(geometry_msgs::Point32 fromWhere) {
	float lowestDist = 1e8;
	for (int i = 0; i < scanCloud.points.size(); ++i) {
		float dist = computeDistance(fromWhere, scanCloud.points[i]);
		if(dist < lowestDist) {
			lowestDist = dist;
			indexCol = i;
		}
	}

	//DEBUG
	if(lowestDist <= CLEARANCE_DIST) {
		ROS_INFO("Collision index is:%d of %d, dist:%f", indexCol, scanCloud.points.size(), lowestDist);
	}
	
	//END DEBUG
	
	return lowestDist;

}

bool hasCollision(std::vector<geometry_msgs::Point32> points) {
	for (int i = 0; i < points.size(); ++i) {
		float obstacleDist = laserObstacleDistance(points[i]);

		//Collision if obstacleDist == 0 --- May want to add security due to laser inaccuracy
		if(obstacleDist <= CLEARANCE_DIST) {
			//ROS_INFO("Collision at fragment:%d of %d, obsDist:%f", i, points.size(),  obstacleDist);
			return true;

		}
	}
	return false;
}

float computeDistance(geometry_msgs::Point32 & p1, geometry_msgs::Point32 & p2) {
	return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}
