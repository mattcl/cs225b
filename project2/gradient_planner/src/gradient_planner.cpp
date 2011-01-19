#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>

#include <queue>
#include <set>
#include <vector>
#include "GPCostmap.h"
#include <gradient_planner/GPCostmapM.h>
#include <gradient_planner/GPPath.h>

const int NUM_NEIGHBORS = 8;
const int NEIGHBOR_OFFSETS[2][8] = {
	{ 0, 0, 1, 1, 1,-1,-1,-1},
	{ 1,-1,-1, 0, 1,-1, 0, 1}
};

tf::TransformListener *transformListener;
GPCostmap *totalObstacleMap;
GPCostmap *obstacleMap;
GPCostmap *costmap;
sensor_msgs::PointCloud wayPoints;

void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg);
void computeDistanceFunction(GPCostmap *obsMap);
void createInitialPointQueue(std::priority_queue<coord_t, std::vector<coord_t>, CompareCoord> & workingPoints, GPCostmap *obsMap);

void rvizCallback(const geometry_msgs::PoseStampedConstPtr& msg);
bool transformPoseToFrame(const std::string &frame, const geometry_msgs::PoseStamped & in, geometry_msgs::PoseStamped & out);
void computeCostmap(int goalRow, int goalCol, int robotX, int robotY);
float computeTraversalCost(int row, int col);

float* computeGradient(float x, float y);
float* gradientDescent(float x, float y);

float computeCostLS(int row, int col);
void computeCostmapLS(int goalRow, int goalCol, int robotX, int robotY); 
  
int main(int argc, char** argv){
	ros::init(argc, argv, "gradient_planner");
	ros::NodeHandle node;
	ros::Rate r(10);
	ros::Subscriber map_sub = node.subscribe("map", 100, mapCallback);
	ros::Subscriber rviz_sub = node.subscribe("goal", 100, rvizCallback);
	ros::Publisher obstacle_dist_pub = node.advertise<gradient_planner::GPCostmapM>("object_distance_fn", 100);
	// this broadcasts the whole distance function, ignoring the physical constraints of the basement
	ros::Publisher total_obstacle_dist_pub = node.advertise<gradient_planner::GPCostmapM>("total_object_distance_fn", 100);
	ros::Publisher cost_fn_pub = node.advertise<gradient_planner::GPCostmapM>("costmap_fn", 100);
	ros::Publisher path_pub = node.advertise<sensor_msgs::PointCloud>("computed_path", 100);
	transformListener = new tf::TransformListener(ros::Duration(10));

	totalObstacleMap = NULL;
	obstacleMap = NULL;
	costmap = NULL;

	wayPoints.header.frame_id = "/map";	

	while(ros::ok()) {

		if(obstacleMap != NULL) {
			gradient_planner::GPCostmapM dis_fn_msg;
			dis_fn_msg.header.stamp = ros::Time::now();
			dis_fn_msg.header.frame_id = "/map";
			dis_fn_msg.rows = obstacleMap->getNumRows();
			dis_fn_msg.cols = obstacleMap->getNumCols();
			float * dataptr = obstacleMap->getData();
			for(int i = 0; i < obstacleMap->getNumElements(); i++) {
				dis_fn_msg.data.push_back(dataptr[i]);
			}
			obstacle_dist_pub.publish(dis_fn_msg);
		}

		if(totalObstacleMap != NULL) {
			gradient_planner::GPCostmapM total_dis_fn_msg;
			total_dis_fn_msg.header.stamp = ros::Time::now();
			total_dis_fn_msg.header.frame_id = "/map";
			total_dis_fn_msg.rows = totalObstacleMap->getNumRows();
			total_dis_fn_msg.cols = totalObstacleMap->getNumCols();
			float * dataptr = totalObstacleMap->getData();
			for(int i = 0; i < totalObstacleMap->getNumElements(); i++) {
				total_dis_fn_msg.data.push_back(dataptr[i]);
			}
			total_obstacle_dist_pub.publish(total_dis_fn_msg);
		}
			
		if(costmap != NULL) {
			gradient_planner::GPCostmapM cost_fn_msg;
			cost_fn_msg.header.stamp = ros::Time::now();
			cost_fn_msg.header.frame_id = "/map";
			cost_fn_msg.rows = costmap->getNumRows();
			cost_fn_msg.cols = costmap->getNumCols();
			float * dataptr = costmap->getData();
			for(int i = 0; i < costmap->getNumElements(); i++) {
				cost_fn_msg.data.push_back(dataptr[i]);
			}
			cost_fn_pub.publish(cost_fn_msg);
		}

		if(!wayPoints.points.empty()) {
			wayPoints.header.stamp = ros::Time::now();
			path_pub.publish(wayPoints);
		}

		ros::spinOnce();
		r.sleep();
	}
}

void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
	ROS_INFO("received map");
	obstacleMap = new GPCostmap(msg->info.width, msg->info.height);
	totalObstacleMap = new GPCostmap(msg->info.width, msg->info.height);
	obstacleMap->setAll(-1.0);
	totalObstacleMap->setAll(-1.0);
	int size = obstacleMap->getNumElements();
	for(int i = 0; i < size; i++) {
		if(msg->data[i] != 100) {
			ROS_INFO("data %d, %d", i, msg->data[i]);
		}
		if(msg->data[i] >= 40) {
			// set initial "distance" to obstacle as 0
			obstacleMap->set(i, 0.0);
			if(msg->data[i] >= 60) {
				totalObstacleMap->set(i, 0.0);
			}
		}	
	}
	computeDistanceFunction(obstacleMap);
	computeDistanceFunction(totalObstacleMap);
	ROS_INFO("costmap completed");
}

void computeDistanceFunction(GPCostmap *obsMap) {
	std::priority_queue<coord_t, std::vector<coord_t>, CompareCoord> workingPoints;
	createInitialPointQueue(workingPoints, obsMap);
	int itr = 0;

	while(!workingPoints.empty()) {
		coord_t coord = workingPoints.top();
		workingPoints.pop();
		float value = coord.value;
		if(coord.value > obsMap->get(coord) && obsMap->get(coord) >= 0)
			continue;
		for(int i = 0; i < NUM_NEIGHBORS; i++) {
			int row = coord.row + NEIGHBOR_OFFSETS[0][i];
			int col = coord.col + NEIGHBOR_OFFSETS[1][i];
			
			if(row < 0 || row > obsMap->getNumRows() - 1 || col < 0 || col > obsMap->getNumCols() - 1)
				continue;
				
			float cost = value + 1.0;
			//logical xor
			if((row || col) && !(row && col)) {
				cost += 0.414;
			}
			float neighborValue = obsMap->get(row, col);
			if(neighborValue == 0)
				continue;
			if(neighborValue < 0 || neighborValue > cost) {
				// set the value for this cell and add it
				// to the list of working points
				obsMap->set(row, col, cost);
				coord_t newCoord;
				newCoord.row = row;
				newCoord.col = col;
				newCoord.value = cost;
				workingPoints.push(newCoord);
			}
		}
		itr++;	
	}
	ROS_INFO("obstacle distance function completed in %d iterations", itr);
}

void createInitialPointQueue(std::priority_queue<coord_t, std::vector<coord_t>, CompareCoord> & workingPoints, GPCostmap *obsMap) {	
	for(int row = 0; row < obsMap->getNumRows(); row++) {
		for(int col = 0; col < obsMap->getNumCols(); col++) {
			if(obsMap->get(row, col) != -1.0) {
				coord_t coord;
				coord.row = row;
				coord.col = col;
				coord.value = 0.0;
				workingPoints.push(coord);
			}
		}
	}
}

void rvizCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
	ROS_INFO("received goal from rviz");

	tf::StampedTransform mapBase;
	try {
		transformListener->lookupTransform("/map", "/base_footprint", ros::Time(0), mapBase);
	} catch (tf::TransformException ex) {
		ROS_ERROR("Could not get transform from map to base");
		return;
	}
	
	geometry_msgs::PoseStamped robotPoseInMap;
	robotPoseInMap.pose.position.x = mapBase.getOrigin().x();
	robotPoseInMap.pose.position.y = mapBase.getOrigin().y();
	robotPoseInMap.pose.position.z = 0.0;
	robotPoseInMap.pose.orientation.x = mapBase.getRotation().x();
	robotPoseInMap.pose.orientation.y = mapBase.getRotation().y();
	robotPoseInMap.pose.orientation.z = mapBase.getRotation().z();
	robotPoseInMap.pose.orientation.w = mapBase.getRotation().w();

	ROS_INFO("robot location in map frame is (x, y) (%f, %f)", robotPoseInMap.pose.position.x, robotPoseInMap.pose.position.y);

	int robotX = (int) (robotPoseInMap.pose.position.x * 10.0);
	int robotY = (int) (robotPoseInMap.pose.position.y * 10.0);
	
	geometry_msgs::PoseStamped goalPoseInMap;
	geometry_msgs::PoseStamped poseIn;
	poseIn.header = msg->header;
	poseIn.pose = msg->pose;
	transformPoseToFrame("map", poseIn, goalPoseInMap);
	ROS_INFO("goal point in map frame is (x, y) (%f, %f)", goalPoseInMap.pose.position.x, goalPoseInMap.pose.position.y);

	int goalX = (int) (goalPoseInMap.pose.position.x * 10.0);
	int goalY = (int) (goalPoseInMap.pose.position.y * 10.0);

	int goalRow = goalX;
	int goalCol = goalY;
	

	if (computeTraversalCost(goalRow, goalCol) == 0) {
		ROS_INFO("This is an invalid goal!");
		return;
	}
	costmap = new GPCostmap(obstacleMap->getNumRows(), obstacleMap->getNumCols());
	//costmap->setAll(-1.0);
	costmap->setAll(100000000);

	costmap->set(goalRow, goalCol, 0.0);
	computeCostmapLS(goalRow, goalCol, robotX, robotY);	
	//computeCostmap(goalRow, goalCol, robotX, robotY);
	float *newWayPoint = gradientDescent(robotX, robotY);
	float wayPoint[2];
	
	wayPoints.points.clear();
	int i = 0;
	//for (int i = 0; i<100;i++){
		
	while (1){ 
		wayPoint[0] = newWayPoint[0];
		wayPoint[1] = newWayPoint[1];
		
		geometry_msgs::Point32 point;
		point.x = wayPoint[0] / 10.0;
		point.y = wayPoint[1] / 10.0;
		point.z = 0;
		wayPoints.points.push_back(point);	

		if (fabs(goalX - wayPoint[0]) + fabs(goalY - wayPoint[1]) <= 1.0){
			break;
		}

		newWayPoint = gradientDescent(wayPoint[0], wayPoint[1]);
		i++;
	}

	ROS_INFO("path completed with length: %d", i);
}

bool transformPoseToFrame(const std::string & frame, const geometry_msgs::PoseStamped & in, geometry_msgs::PoseStamped & out) {
	try {
		transformListener->transformPose(frame, in, out);
	} catch (tf::TransformException &ex) {
		ROS_ERROR("Could not transform robot pose to frame!");
		return false;
	}
	return true;
}

void computeCostmap(int goalRow, int goalCol, int robotX, int robotY) {
	std::priority_queue<coord_t, std::vector<coord_t>, CompareCoord> workingPoints;
	coord_t goal;
	goal.row = goalRow;
	goal.col = goalCol;
	goal.value = 0;
	workingPoints.push(goal);
	int itr = 0;

	while(!workingPoints.empty()) {
		coord_t coord = workingPoints.top();
		workingPoints.pop();
		//if(coord.row == robotX && coord.col == robotY)
		//	break;
		float value = coord.value;
		if(coord.value > costmap->get(coord) && costmap->get(coord) >= 0)
			continue;
		for(int i = 0; i < NUM_NEIGHBORS; i++) {
			int row = coord.row + NEIGHBOR_OFFSETS[0][i];
			int col = coord.col + NEIGHBOR_OFFSETS[1][i];
			
			if(row < 0 || row > costmap->getNumRows() - 1 || col < 0 || col > costmap->getNumCols() - 1)
				continue;
				
			float cost = value + 1.0;
			//logical xor
			if((row || col) && !(row && col)) {
				cost += 0.414;
			}
			
			float traversalCost = computeTraversalCost(row, col);
			if(traversalCost == 0.0)
				continue;
			cost += traversalCost;

			float neighborValue = costmap->get(row, col);
			if(neighborValue < 0 || neighborValue > cost) {
				// set the value for this cell and add it
				// to the list of working points
				costmap->set(row, col, cost);
				coord_t newCoord;
				newCoord.row = row;
				newCoord.col = col;
				newCoord.value = cost;
				workingPoints.push(newCoord);
			}
		}
		itr++;	
	}
	ROS_INFO("cost function completed in %d iterations", itr);
}

float computeTraversalCost(int row, int col) {
	float value = obstacleMap->get(row, col);
	if(value <= 0)
		return 0.0;
	if(value <= 1.5)
		return 0.0; 
	value -= 1.5;
	return 20.0 * 1.0 / (value * value);
}


float* computeGradient(float x, float y) {

coord_t NE, SE, NW, SW;

NW.row = int (x);
NW.col = int (y) + 1;
NW.value = 0;

SW.row = int (x);
SW.col = int (y);
SW.value = 0;

NE.row = int (x) + 1;
NE.col = int (y) + 1;
NE.value = 0;

SE.row = int (x) + 1;
SE.col = int (y);
SE.value = 0;

NW.value = costmap->get(NW);
SW.value = costmap->get(SW);
NE.value = costmap->get(NE);
SE.value = costmap->get(SE);

float gradient[2];
float* pointer;
pointer = gradient;
float mod_gradient;

gradient[0] = (y - int(y))*(NE.value - NW.value) + (1 + int(y) -y)*(SE.value - SW.value);
gradient[1] = (x - int(x))*(NE.value - SE.value) + (1 + int(x) -x)*(NW.value - SW.value);

mod_gradient = sqrt(gradient[0]*gradient[0] + gradient[1]*gradient[1]);
gradient[0] = gradient[0]/mod_gradient;
gradient[1] = gradient[1]/mod_gradient;
 
ROS_INFO("Gradient at point (x,y) (%f, %f) is (%f, %f) ", x, y, gradient[0], gradient[1]);

return pointer;

}


float* gradientDescent(float x, float y) {

float* gradient = computeGradient(x, y);

float wayPoint[2];
float* pointer;
pointer = wayPoint;
float stepSize = 1;

wayPoint[0] = x -stepSize*gradient[0];
wayPoint[1] = y -stepSize*gradient[1];
 
ROS_INFO("New way point (x,y) is (%f, %f)",wayPoint[0], wayPoint[1]);

return pointer;

}

void computeCostmapLS(int goalRow, int goalCol, int robotX, int robotY) {
	std::priority_queue<coord_t, std::vector<coord_t>, CompareCoord> workingPoints;
	coord_t goal;
	goal.row = goalRow;
	goal.col = goalCol;
	goal.value = 0;
	workingPoints.push(goal);
	int itr = 0;

	while(!workingPoints.empty()) {
		coord_t coord = workingPoints.top();
		workingPoints.pop();
		if(coord.row == robotX && coord.col == robotY)
			break;
		//float value = coord.value;
		if(coord.value > costmap->get(coord) && costmap->get(coord) != 100000000 && costmap->get(coord) >= 0)
			continue;
		for(int i = 0; i < NUM_NEIGHBORS; i++) {
			int row = coord.row + NEIGHBOR_OFFSETS[0][i];
			int col = coord.col + NEIGHBOR_OFFSETS[1][i];
			// check that neighbour lies within world
			if(row < 0 || row > costmap->getNumRows() - 1 || col < 0 || col > costmap->getNumCols() - 1)
				continue;
			
			
			// prevent exiting a wall
			float traversalCost = computeTraversalCost(row, col);
			if(traversalCost == 0.0)
				continue;

			float cost = computeCostLS(row, col);

			float neighborValue = costmap->get(row, col);
			if(neighborValue == 100000000 || ((neighborValue > cost) && neighborValue != 100000000)) {
				// set the value for this cell and add it
				// to the list of working points
				costmap->set(row, col, cost);
				coord_t newCoord;
				newCoord.row = row;
				newCoord.col = col;
				newCoord.value = cost;
				workingPoints.push(newCoord);
			}
		}
		itr++;	
	}
	ROS_INFO("cost function completed in %d iterations", itr);
}

float computeCostLS(int row, int col) {

float F_ij = 1 + computeTraversalCost(row, col);
	
coord_t N, S, E, W;

N.row = row;
N.col = col + 1;
N.value = 100000000;

S.row = row;
S.col = col-1;
S.value = 100000000;

E.row = row + 1;
E.col = col;
E.value = 100000000;

W.row = row - 1;
W.col = col;
W.value = 100000000;

if (!(N.row < 0 || N.row > costmap->getNumRows() - 1 || N.col < 0 || N.col > costmap->getNumCols() - 1))
	N.value = costmap->get(N);
if (!(S.row < 0 || S.row > costmap->getNumRows() - 1 || S.col < 0 || S.col > costmap->getNumCols() - 1))
	S.value = costmap->get(S);
if (!(E.row < 0 || E.row > costmap->getNumRows() - 1 || E.col < 0 || E.col > costmap->getNumCols() - 1))
	E.value = costmap->get(E);
if (!(W.row < 0 || W.row > costmap->getNumRows() - 1 || W.col < 0 || W.col > costmap->getNumCols() - 1))
	W.value = costmap->get(W);

float Ta, Tc;

// N is minimum
if (N.value <= S.value && N.value <= E.value && N.value <= W.value){
	//NE	
	if (E.value<=W.value){
		Ta = N.value;
		Tc = E.value;		
	}
	//NW
	else {
		Ta = N.value;
		Tc = W.value;
	}
}
// S is minimum
else if (S.value <= N.value && S.value <= E.value && S.value <= W.value){ 			
	//SE	
	if (E.value<=W.value){
		Ta = S.value;
		Tc = E.value;	
	}
	//SW
	else {
		Ta = S.value;
		Tc = W.value;
	}
}
//E is minimum
else if (E.value <= N.value && E.value <= S.value && E.value <= W.value){ 			
	//EN			
	if (N.value<=S.value){
		Ta = E.value;
		Tc = N.value;
	}
	//ES
	else {
		Ta = E.value;
		Tc = S.value;
	}
}
//W is minimum
else { 	
	//WN		
	if (N.value<=S.value){
		Ta = W.value;
		Tc = N.value;
	}
	//WS
	else {
		Ta = W.value;
		Tc = S.value;
	}
}


if ((Tc-Ta)*(Tc-Ta)>8*F_ij*F_ij){
	return Ta + F_ij;	
}
else {
	if ((Ta + Tc)/2 + sqrt(F_ij*F_ij/2 - (Tc-Ta)*(Tc-Ta)/16) < Ta + F_ij)
		return (Ta + Tc)/2 + sqrt(F_ij*F_ij/2 - (Tc-Ta)*(Tc-Ta)/16);
	else 
		return Ta + F_ij;
}


}

