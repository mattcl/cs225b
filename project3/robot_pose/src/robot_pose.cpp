#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point32.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <gradient_planner/GPCostmapM.h>
#include <nav_msgs/OccupancyGrid.h>
#include "GPCostmap.h"
#include <vector>
#include <queue>
#include "pose.h"
#include "sensor_model.h"
#include "math.h"
#include <string.h>
using namespace std;
int INIT_NUM_POSE = 500;
int MSG_FREQUENCY = 1; //Frequency of send/receive message

//Motion Model Parameter
float MOTION_ERROR_MEAN_LINEAR = 0;
float MOTION_ERROR_VARIANCE_LINEAR = 0.05;
float MOTION_ERROR_MEAN_ANGULAR = 0;
float MOTION_ERROR_VARIANCE_ANGULAR = 0.2;
//Threshold for updating pose if linear/angular displacement greater than this
float POSE_UPDATE_LINEAR = 0.3;
float POSE_UPDATE_ANGULAR = M_PI/18; //10 degrees
float LASER_RANGE = 12.0;

const float MAP_RESOLUTION = 10.0; //How many pixels per meter of map
tf::TransformListener *transformListener;
tf::Transform mapOdomTF;

vector<Pose> poses, scPoses;
geometry_msgs::PoseArray poseArray;
Pose lastPose, currentPose; //these are from odom
sensor_msgs::LaserScan *laserScan;
bool poseInitialized = false; //See if we have received the first pose to compare with
bool use_odom_correction = false;
bool do_uniform = false;
SensorModel *sensorModel;
int wait = 3;

//Scan Matching parameters
SensorModel *scanMatching1;
SensorModel *scanMatching2;
GPCostmap *refScan;
GPCostmap *curScan;
sensor_msgs::PointCloud localScan;
sensor_msgs::PointCloud scanCloud;
const int NUM_NEIGHBORS = 8;
const int NEIGHBOR_OFFSETS[2][8] = {
	{ 0, 0, 1, 1, 1,-1,-1,-1},
	{ 1,-1,-1, 0, 1,-1, 0, 1}
};
const float SC_INIT_VARIANCE = 0.5;
const float SC_INIT_VARIANCE_T = M_PI/18;

bool refScanInitialized = false;
bool mapReady = false;
bool runScanMatching = false;
float KMatrix [3][3] = {{0,0,0},{0,0,0},{0,0,0}};
float uMatrix [3][3] = {{0,0,0},{0,0,0},{0,0,0}};
float LMatrix [3][3];


void InitUniform(float x1, float x2, float y1, float y2, float t1, float t2);
void InitGaussian(float mx, float vx, float my, float vy, float mt, float vt);
float GenerateGaussian(float mean, float variance);
void PreparePublish(); //populate poseArray to publish poses in rviz
void MovePoses(Pose* old_pose, Pose* new_pose, float ml, float vl, float ma, float va);
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
void distanceCallback(const gradient_planner::GPCostmapMConstPtr& msg);
void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
void makeMapOdomTransform(tf::Transform& map_base);
float ComputePoseDistance(Pose* p1, Pose* p2);
void loadParamsFromConfig(ros::NodeHandle &n);

void computeNewTransform(Pose& recommendedPose); 
void Cholesky(float sigmaMatrix[][3], float L[][3]); 

void laserToPointCloud(const sensor_msgs::LaserScan::ConstPtr& msg);
void computeDistanceFunction(GPCostmap *obsMap);
void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg);
void createInitialPointQueue(std::priority_queue<coord_t, std::vector<coord_t>, CompareCoord> & workingPoints, GPCostmap *obsMap);
void InitScanMatchingGaussian(float mx, float vx, float my, float vy, float mt, float vt);


void InitUniform(float x1, float x2, float y1, float y2, float t1, float t2) {
	poses.clear();
	for(int i = 0; i < 10000; ++i) {
		Pose insert;
		insert.x = (drand48()*(x2-x1))+x1;
		insert.y = (drand48()*(y2-y1))+y1;
		insert.t = (drand48()*(t2-t1))+t1;

		poses.push_back(insert);
	}
}

/* This algorithm crunches numbers and give out a number in 
 * Gaussian distribution. Magic black box.
 */
float GenerateGaussian(float mean, float variance) {
	float x1, x2, w;
	do {
		x1 = 2.0 * drand48() - 1.0;
		x2 = 2.0 * drand48() - 1.0;
		w = x1 * x1 + x2 * x2;
	} while ( w >= 1.0 );

	w = sqrt( (-2.0 * log( w ) ) / w );
	return (x1*w*variance)+mean;

}

void Cholesky(float sigmaMatrix[][3], float L[][3]) {

	L[0][0] = sqrt(sigmaMatrix[0][0]);

	L[1][0] = sigmaMatrix[1][0]/L[0][0];
	L[0][1] = 0;

	L[2][0] = sigmaMatrix[2][0]/L[0][0];
	L[0][2] = 0;

	L[1][1] = sqrt(sigmaMatrix[1][1] - L[1][0]*L[1][0]);

	L[2][1] = (sigmaMatrix[2][1] - L[2][0]*L[1][0])/L[1][1];
	L[1][2] = 0;

	L[2][2] = sqrt(sigmaMatrix[2][2] - L[2][0]*L[2][0] - L[2][1]*L[2][1]);	

}



void PreparePublish() {
	poseArray.poses.clear();
	poseArray.header.frame_id = "/odom";
	for(int i = 0; i < poses.size(); ++i) {
		geometry_msgs::Pose insert;
		insert.position.x = poses[i].x;
		insert.position.y = poses[i].y;
		insert.orientation.w = (sqrt(2+2*cos(poses[i].t)))/2;
		insert.orientation.z = (1*sin(poses[i].t))/(2*insert.orientation.w);
		poseArray.poses.push_back(insert);
	}
}

//Mean:m, Variance:v. There is mean & var associated with x,y,theta
void InitGaussian(float mx, float vx, float my, float vy, float mt, float vt) {
	poses.clear();
	for(int i = 0; i < INIT_NUM_POSE; ++i) {
		Pose insert;
		insert.x = GenerateGaussian(mx, vx);
		insert.y = GenerateGaussian(my, vy);
		insert.t = GenerateGaussian(mt, vt);
		poses.push_back(insert);
	}
}

//Mean:m, Variance:v. There is mean & var associated with x,y,theta
void InitScanMatchingGaussian(float mx, float vx, float my, float vy, float mt, float vt) {
	scPoses.clear();
	for(int i = 0; i < INIT_NUM_POSE; ++i) {
		Pose insert;
		insert.x = GenerateGaussian(mx, vx);
		insert.y = GenerateGaussian(my, vy);
		insert.t = GenerateGaussian(mt, vt);
		scPoses.push_back(insert);
	}
}

/* MovePoses takes as arguments an initial(or last) position from odom
 * (after converted to x,y,theta from quaternion),
 * the current position, and the error function (now Gaussian)
 * Error fn: ml: mean linear, ma:mean angular, vl:variance linear, va:variance angular)
 */
void MovePoses(Pose* old_pose, Pose* new_pose, float ml, float vl, float ma, float va) {
	float deltaT = ComputePoseDistance(old_pose, new_pose);
	float thetaT = atan2(new_pose->y-old_pose->y, new_pose->x-old_pose->x);
	float delta1 = thetaT-old_pose->t;
	float delta2 = new_pose->t-thetaT;

	if(deltaT>=POSE_UPDATE_LINEAR || fabs((new_pose->t-old_pose->t))>=POSE_UPDATE_ANGULAR) {	  
		//Scan Matching Process:
		//Insert code to generate poses, etc here
		//***********
		if(runScanMatching) {
			InitScanMatchingGaussian(new_pose->x, SC_INIT_VARIANCE, new_pose->y, SC_INIT_VARIANCE, new_pose->t, SC_INIT_VARIANCE_T);
			scanMatching1->scResample(laserScan, scPoses);
			float sumX = 0;
			float sumY = 0; 
			float sumT = 0;
			for(int i = 0; i < scPoses.size(); ++i) {
				sumX += scPoses[i].x * scPoses[i].weight;
				sumY += scPoses[i].y * scPoses[i].weight;
				sumT += scPoses[i].t * scPoses[i].weight;

				KMatrix[0][0] = scPoses[i].x*scPoses[i].x*scPoses[i].weight;
				KMatrix[0][1] = scPoses[i].x*scPoses[i].y*scPoses[i].weight;
				KMatrix[0][2] = scPoses[i].x*scPoses[i].t*scPoses[i].weight;
				KMatrix[1][0] = KMatrix[0][1];
				KMatrix[1][1] = scPoses[i].y*scPoses[i].y*scPoses[i].weight;
				KMatrix[1][2] = scPoses[i].y*scPoses[i].t*scPoses[i].weight;
				KMatrix[2][0] = KMatrix[0][2];
				KMatrix[2][1] = KMatrix[1][2];
				KMatrix[2][2] = scPoses[i].t*scPoses[i].t*scPoses[i].weight;

				uMatrix[0][0] = scPoses[i].x*scPoses[i].x*scPoses[i].weight*scPoses[i].weight;
				uMatrix[0][1] = scPoses[i].x*scPoses[i].y*scPoses[i].weight*scPoses[i].weight;
				uMatrix[0][2] = scPoses[i].x*scPoses[i].t*scPoses[i].weight*scPoses[i].weight;
				uMatrix[1][0] = uMatrix[0][1];
				uMatrix[1][1] = scPoses[i].y*scPoses[i].y*scPoses[i].weight*scPoses[i].weight;
				uMatrix[1][2] = scPoses[i].y*scPoses[i].t*scPoses[i].weight*scPoses[i].weight;
				uMatrix[2][0] = uMatrix[0][2];
				uMatrix[2][1] = uMatrix[1][2];
				uMatrix[2][2] = scPoses[i].t*scPoses[i].t*scPoses[i].weight*scPoses[i].weight;
			}
			float scMeanX = sumX;//scPoses.size();
			float scMeanY = sumY;//scPoses.size();
			float scMeanT = sumT;//scPoses.size();

			ROS_INFO("Mean values (%f, %f, %f)", scMeanX, scMeanY, scMeanT);
			//calculate K-u Matrix - overwriting KMatrix
			for (int i = 0; i < 3; ++i) {
				for(int j = 0; j < 3; ++j) {
					if (i==j)
						KMatrix [i][j] = KMatrix[i][j] - uMatrix[i][j];
					else
						KMatrix[i][j] = 0;
				}
			}
			Cholesky(KMatrix, LMatrix);
			for (int i = 0;i<=2;i++)
				for (int j=0;j<=2;j++)
					ROS_INFO("L[%d][%d] = %f ", i, j, LMatrix[i][j]);

			//3 Random Gaussian based
			for(int i = 0; i < poses.size(); ++i) {
				float r1 = GenerateGaussian(0,1);
				float r2 = GenerateGaussian(0,1);
				float r3 = GenerateGaussian(0,1);

				poses[i].x = scMeanX + (LMatrix[0][0]*r1+LMatrix[0][1]*r2+LMatrix[0][2]*r3);
				poses[i].y = scMeanY + (LMatrix[1][0]*r1+LMatrix[1][1]*r2+LMatrix[1][2]*r3);
				poses[i].t = scMeanT + (LMatrix[2][0]*r1+LMatrix[2][1]*r2+LMatrix[2][2]*r3);
			}

		} else {

			for (int i = 0; i < poses.size(); ++i) {

				float deltaTHat = deltaT+GenerateGaussian(ml, vl*fabs(deltaT));
				float delta1Hat = delta1+GenerateGaussian(ma, va*fabs(delta1));
				float delta2Hat = delta2+GenerateGaussian(ma, va*fabs(delta2));

				poses[i].x += deltaTHat*cos(delta1Hat+poses[i].t);
				poses[i].y += deltaTHat*sin(delta1Hat+poses[i].t);
				poses[i].t += delta1Hat + delta2Hat;
			}
		}
		old_pose->x = new_pose->x;
		old_pose->y = new_pose->y;
		old_pose->t = new_pose->t;
		sensorModel->resample(laserScan, poses);
		refScanInitialized = false;

		// get a recommended pose from our new set of poses
		Pose recommendedPose;
		sensorModel->recommendPose(poses, recommendedPose);
		// compute the new map_odom transform
		if(use_odom_correction) {
			if(wait <= 0) {
				old_pose->x = recommendedPose.x;
				old_pose->y = recommendedPose.y;
				old_pose->t = recommendedPose.t;
				computeNewTransform(recommendedPose);
			} else {
				wait--;	
			}
		}
	}
}

void computeNewTransform(Pose& recommendedPose) {
	ROS_INFO("computing new map_odom transform");
	geometry_msgs::PoseStamped mapPose;
	try {
		geometry_msgs::PoseStamped tempIn;
		tempIn.pose.position.x = recommendedPose.x;
		tempIn.pose.position.y = recommendedPose.y;
		tempIn.pose.orientation.w = (sqrt(2+2*cos(recommendedPose.t)))/2.0; 
		tempIn.pose.orientation.z = (1*sin(recommendedPose.t))/(2*tempIn.pose.orientation.w);
		tempIn.header.frame_id = "/odom";
		tempIn.header.stamp = ros::Time(0);
		const geometry_msgs::PoseStamped poseIn = tempIn;
		transformListener->transformPose("/map", poseIn, mapPose);
	} catch (tf::TransformException ex) {
		ROS_ERROR("!error computing new transform: %s\n", ex.what());
	}	
	tf::Quaternion q;
	tf::quaternionMsgToTF(mapPose.pose.orientation, q);
	tf::Transform map_base = tf::Transform(q, tf::Vector3(mapPose.pose.position.x, mapPose.pose.position.y, 0.0));
	makeMapOdomTransform(map_base);
	ROS_INFO("new transform completed");
}

	float ComputePoseDistance(Pose* p1, Pose* p2) {
		if(p1 == NULL || p2 == NULL)
			return 0;
		return (sqrt((p2->x-p1->x)*(p2->x-p1->x) + (p2->y-p1->y) * (p2->y-p1->y)));
	}


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	if(!poseInitialized) {
		lastPose.x = msg->pose.pose.position.x;
		lastPose.y = msg->pose.pose.position.y;
		lastPose.t = atan2(2*msg->pose.pose.orientation.w*msg->pose.pose.orientation.z, 1-(2*msg->pose.pose.orientation.z*msg->pose.pose.orientation.z));
		if(do_uniform) {
			InitUniform(-30, 30, -30, 30, 0, 2*2*2*M_PI);
			do_uniform = false;
		} else {
			InitGaussian(lastPose.x, 1, lastPose.y, 1, lastPose.t, M_PI/10);
		}
		poseInitialized = true;

	} else {
		currentPose.x = msg->pose.pose.position.x;
		currentPose.y = msg->pose.pose.position.y;
		currentPose.t = atan2(2*msg->pose.pose.orientation.w*msg->pose.pose.orientation.z, 1-(2*msg->pose.pose.orientation.z*msg->pose.pose.orientation.z));
		MovePoses(&lastPose, &currentPose, MOTION_ERROR_MEAN_LINEAR, MOTION_ERROR_VARIANCE_LINEAR, MOTION_ERROR_MEAN_ANGULAR, MOTION_ERROR_VARIANCE_ANGULAR);
	}
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
	//	ROS_INFO("received laser scan");
	laserScan = new sensor_msgs::LaserScan();
	laserScan->header = msg->header;
	laserScan->header.stamp = ros::Time(0);
	laserScan->angle_min = msg->angle_min;
	laserScan->angle_max = msg->angle_max;
	laserScan->angle_increment = msg->angle_increment;
	laserScan->ranges = msg->ranges;
	LASER_RANGE = msg->range_max-msg->range_min;

	//Need to know when tranform from /base_link to /map is ready
	//Set mapReady=true when that happens. Currently after we get initial pose.
	if(mapReady && runScanMatching) {
		ROS_INFO("Readying for laser transform\n");
		//Creating a new GPCostmap based on the scan
		laserToPointCloud(msg); //Fills localScan with laser data
		const sensor_msgs::PointCloud laserIn = localScan;
		scanCloud.points.clear();
		//Transform to /odom frame - PROBLEM AS TO WHEN WE CAN TRANSFORM
		if(transformListener->canTransform("/map", "/base_link", ros::Time::now(), NULL)) {
			try {
				//transformListener->waitForTransform("/map","/base_link",ros::Time(0), ros::Duration(4.0));
				transformListener->transformPointCloud("/map", laserIn, scanCloud);
			} catch (tf::TransformException &ex) {
				ROS_ERROR("[robot_pose] Could not transform laser scan to map frame: %s\n", ex.what());
			} 
			ROS_INFO("Sucessful Transform\n");

			//Now we have ScanCloud, set of obstacle points in map frame
			//Init refscan and compute likelihoodField of scanMatching
			if(refScan != NULL && !refScanInitialized) {
				refScan->setAll(-1); //Init step of GPCostmap
				//Now set all obstacle points from laser in refScan to (zero)
				for(int i = 0; i < scanCloud.points.size(); ++i) {
					ROS_INFO("Point[%d]: x: %f, y: %f\n", i, scanCloud.points[i].x, scanCloud.points[i].y);
					refScan->set((int)(scanCloud.points[i].x*10), (int)(scanCloud.points[i].y*10), 0);
				}
				computeDistanceFunction(refScan);
				scanMatching1->setLikelihoodField(refScan);
				refScanInitialized = true;
			} /*else if (refScan != NULL && refScanInitialized) {
			    ROS_INFO("GettingCurScan\n");
			    curScan->setAll(-1);
			    for(int i = 0; i < scanCloud.points.size(); ++i) {
			//ROS_INFO("CurPoint[%d]: x: %f, y: %f\n", i, scanCloud.points[i].x, scanCloud.points[i].y);
			curScan->set((int)scanCloud.points[i].x, (int)scanCloud.points[i].y, 0);
			}
			computeDistanceFunction(curScan);
			scanMatching2->setLikelihoodField(curScan);
			ROS_INFO("Ready to find variance & mean\n");
			}*/
		} else {
			ROS_INFO("Can't transform yet\n");
		}
	}
}

//Convert sensor_msgs::LaserScan(laserMsg) to PointCloud  (localScan)
void laserToPointCloud(const sensor_msgs::LaserScan::ConstPtr& msg) {
	int nPoints = ((msg->angle_max-msg->angle_min)/msg->angle_increment)+1;
	//ROS_INFO("LASER MIN:%f, MAX:%f\n", msg->range_min, msg->range_max);
	localScan.points.clear();
	for (int i = 0; i < nPoints; ++i) {
		if(msg->ranges[i] < msg->range_max) {
			geometry_msgs::Point32 temp;
			temp.x = msg->ranges[i]*cos(msg->angle_min+i*msg->angle_increment);
			temp.y = msg->ranges[i]*sin(msg->angle_min+i*msg->angle_increment);
			localScan.points.push_back(temp);
		}
	}
	localScan.header.stamp = ros::Time(0);
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

void distanceCallback(const gradient_planner::GPCostmapMConstPtr& msg) {
	sensorModel->setLikelihoodField(msg);	
}

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {
	ROS_INFO("received initial pose from rviz");
	tf::Quaternion truthQ;
	tf::quaternionMsgToTF(msg->pose.pose.orientation, truthQ);
	tf::Transform map_base = tf::Transform(truthQ, tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0));
	makeMapOdomTransform(map_base);
	poseInitialized = false;
	mapReady = true;
} 

void makeMapOdomTransform(tf::Transform& map_base) {
	try {
		tf::StampedTransform base_odom;
		transformListener->lookupTransform("/base_footprint", "/odom", ros::Time(0), base_odom);
		mapOdomTF = map_base * base_odom;
	} catch (tf::TransformException ex) {
		ROS_ERROR("!error in initialPoseCallback: %s\n", ex.what());
	}
}

void loadParamsFromConfig(ros::NodeHandle &n) {
	ROS_INFO("Motion Model Loading Params");

	double meml, mevl, mema, meva, pul, pua;
	n.getParam("robot_pose/initial_number_of_poses", INIT_NUM_POSE);
	n.getParam("robot_pose/messaging_frequency", MSG_FREQUENCY);
	n.getParam("robot_pose/motion_error_mean_linear", meml);
	n.getParam("robot_pose/motion_error_variance_linear", mevl);
	n.getParam("robot_pose/motion_error_mean_angular", mema);
	n.getParam("robot_pose/motion_error_variance_angular", meva);
	n.getParam("robot_pose/pose_update_linear", pul);
	n.getParam("robot_pose/pose_update_angular_denominator", pua);
	n.getParam("robot_pose/use_odom_correction", use_odom_correction);
	n.getParam("robot_pose/do_uniform", do_uniform);
	MOTION_ERROR_MEAN_LINEAR = meml;
	MOTION_ERROR_VARIANCE_LINEAR = mevl;
	MOTION_ERROR_MEAN_ANGULAR = mema;
	MOTION_ERROR_VARIANCE_ANGULAR = meva;
	POSE_UPDATE_LINEAR = pul;
	POSE_UPDATE_ANGULAR = M_PI/pua; //10 degrees
	ROS_INFO("Motion Model Parameters Loaded:");
	ROS_INFO("Messaging frequency %d", MSG_FREQUENCY);
	ROS_INFO("Initial poses %d", INIT_NUM_POSE);
	ROS_INFO("error mean linear %f, variance %f", MOTION_ERROR_MEAN_LINEAR, MOTION_ERROR_VARIANCE_LINEAR);
	ROS_INFO("error mean angular %f, variance %f", MOTION_ERROR_MEAN_ANGULAR, MOTION_ERROR_VARIANCE_ANGULAR);
	ROS_INFO("pose update linear %f, angular %f", POSE_UPDATE_LINEAR, POSE_UPDATE_ANGULAR);
	ROS_INFO("using odom correction %s", (use_odom_correction ? "TRUE" : "FALSE"));
	if(do_uniform) {
		n.getParam("robot_pose/wait_on_uniform", wait);
		ROS_INFO("setting wait to %d", wait);
	}
	ROS_INFO("Parameter load complete\n\n\n");
}

void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
	refScan = new GPCostmap(msg->info.width, msg->info.height);
	curScan = new GPCostmap(msg->info.width, msg->info.height);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "robot_pose");
	ros::NodeHandle node;
	loadParamsFromConfig(node);
	ros::Rate r(MSG_FREQUENCY);
	srand48(time(NULL));
	ros::Publisher pose_pub = node.advertise<geometry_msgs::PoseArray>("robot_pose", 10);
	ros::Publisher like_pub = node.advertise<gradient_planner::GPCostmapM>("likelihood_field", 10);  
	ros::Publisher refscan_pub = node.advertise<gradient_planner::GPCostmapM> ("refscan", 10);
	ros::Subscriber odom_sub = node.subscribe("odom", 100, odomCallback);
	ros::Subscriber dist_sub = node.subscribe("total_object_distance_fn", 100, distanceCallback);
	ros::Subscriber laser_sub = node.subscribe("base_scan", 100, laserCallback);
	ros::Subscriber rviz_sub = node.subscribe("initialpose", 100, initialPoseCallback);
	ros::Subscriber map_sub = node.subscribe("map", 10, mapCallback);
	laserScan = NULL;
	refScan = NULL;
	localScan.header.frame_id = "/base_link";
	//InitUniform(-1,1,-1,1, -1*M_PI/2, M_PI/2);
	InitGaussian(0, 1, 0, 1, 0, M_PI/10);
	sensorModel = new SensorModel();
	sensorModel->setParamsFromConfig(node);

	scanMatching1 = new SensorModel();
	sensorModel->setParamsFromConfig(node);
	scanMatching2 = new SensorModel();
	sensorModel->setParamsFromConfig(node);

	transformListener = new tf::TransformListener(ros::Duration(100));
	tf::TransformBroadcaster b;

	// set the initial transform to 0, 0, 0
	mapOdomTF = tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(25, 25, 0));

	while(ros::ok()) {
		b.sendTransform(tf::StampedTransform(mapOdomTF, ros::Time::now(), "map", "odom"));
		PreparePublish();
		poseArray.header.stamp = ros::Time::now();
		pose_pub.publish(poseArray);
		GPCostmap *likelihoodField = sensorModel->getLikelihoodField();	
		if(likelihoodField != NULL) {
			gradient_planner::GPCostmapM msg;
			msg.header.stamp = ros::Time::now();
			msg.header.frame_id = "/map";
			msg.rows = likelihoodField->getNumRows();
			msg.cols = likelihoodField->getNumCols();
			float *dataptr = likelihoodField->getData();
			for(int i = 0; i < likelihoodField->getNumElements(); i++)
				msg.data.push_back(dataptr[i]);
			like_pub.publish(msg);
		}
		if(refScan!=NULL && refScanInitialized) {
			gradient_planner::GPCostmapM msg;
			msg.header.stamp = ros::Time::now();
			msg.header.frame_id = "/map";
			msg.rows = refScan->getNumRows();
			msg.cols = refScan->getNumCols();
			float *dataptr = refScan->getData();
			for(int i = 0; i < refScan->getNumElements(); i++)
				msg.data.push_back(dataptr[i]);
			refscan_pub.publish(msg);
		}
		ros::spinOnce();
		r.sleep();
	}
	ROS_INFO("EXITING.....\n");
	sleep(10000);


	return 0;
}
