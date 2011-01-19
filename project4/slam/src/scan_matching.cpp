#include "scan_matching.h"
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
#include "graph.h"
using namespace std;


/*==============================================*/
/* Function definition                         */
/*==============================================*/

ScanMatching::ScanMatching() {
}

void ScanMatching::match(node_t &node, vector<node_t> refNodes, bounds_t bounds, double mean[3], double covariance[][3]) {
	GPCostmap map(bounds.max_x - bounds.min_x, bounds.max_y - bounds.min_y);
	
	
	float translate_x = -(float)bounds.min_x / 10.0;
	float translate_y = -(float)bounds.min_y / 10.0;

	for(int i = 0; i < refNodes.size(); i++) {
		refNodes[i].pose.x += translate_x;
		refNodes[i].pose.y += translate_y;
	}

	node.pose.x += translate_x;
	node.pose.y += translate_y;

	vector<Pose> scPoses;
	InitScanMatchingGaussian(node.pose, 0.1f, 0.02f, scPoses, 500);
	for(int i = 0; i < refNodes.size(); i++) {
		AddScanToRefScans(map, refNodes[i].pose, &refNodes[i].scan);
	}
	model.setLikelihoodField(&map);
	model.scResample(&(node.scan), scPoses);

	CalculateMeanCovariance(scPoses, mean, covariance);
	ChangeFrameMeanCov(mean, covariance, refNodes[refNodes.size() - 1].pose);
	 
	for(int i = 0; i < refNodes.size(); i++) {
		refNodes[i].pose.x -= translate_x;
		refNodes[i].pose.y -= translate_y;
	}

	node.pose.x -= translate_x;
	node.pose.y -= translate_y;
}

// This algorithm crunches numbers and give out a number in Gaussian distribution. Magic black box.
float ScanMatching::GenerateGaussian(float mean, float variance) {
	float x1, x2, w;
	do {
		x1 = 2.0 * drand48() - 1.0;
		x2 = 2.0 * drand48() - 1.0;
		w = x1 * x1 + x2 * x2;
	} while ( w >= 1.0 );

	w = sqrt( (-2.0 * log( w ) ) / w );
	return (x1*w*variance)+mean;

}

// This function will initialize 'scposes' with a Gaussian distribution specified by the input parameters
void ScanMatching::InitScanMatchingGaussian(Pose &pose, float varXY, float varT, vector<Pose>& scPoses, int numPoses) {
	// Mean values are determined by the input pose

	scPoses.clear();
	for(int i = 0; i < numPoses; ++i) {
		Pose insert;
		insert.x = GenerateGaussian(pose.x, varXY);
		insert.y = GenerateGaussian(pose.y, varXY);
		insert.t = GenerateGaussian(pose.t, varT);
		scPoses.push_back(insert);
	}
}

// Given a vector of weighted poses, this function will return the mean and covariance matrix corresponding to it
void ScanMatching::CalculateMeanCovariance(vector<Pose>& scPoses, double mean[3], double covariance[][3]) {

    // First normalize weights
    double totalWeight = 0.0;
    for(int i = 0; i < scPoses.size(); ++i) {
        totalWeight += scPoses[i].weight;
    }
    for(int i = 0; i < scPoses.size(); ++i) {
        scPoses[i].weight /= totalWeight;
    }

    // Calculate the mean
    mean[0] = 0;
    mean[1] = 0;
    mean[2] = 0;

    for(int i = 0; i < scPoses.size(); ++i) {
        mean[0] += scPoses[i].x * scPoses[i].weight;
        mean[1] += scPoses[i].y * scPoses[i].weight;
        mean[2] += scPoses[i].t * scPoses[i].weight;
    }

    // Find sum of square of weights
    double sumWeightSquare = 0.0;
    for(int i = 0; i < scPoses.size(); ++i) {
        sumWeightSquare += (scPoses[i].weight)*(scPoses[i].weight);
    }

    // Calculate the covariance
    for (int i = 0; i<=2; i++){
        for (int j = 0; j<=2; j++){
            covariance[i][j] = 0;
        }
    }

    for(int i = 0; i < scPoses.size(); ++i) {
        covariance[0][0] += (scPoses[i].x - mean[0])*(scPoses[i].x - mean[0])*scPoses[i].weight/(1-sumWeightSquare);
        covariance[0][1] += (scPoses[i].x - mean[0])*(scPoses[i].y - mean[1])*scPoses[i].weight/(1-sumWeightSquare);
        covariance[0][2] += 0.0;//(scPoses[i].x - mean[0])*(scPoses[i].t - mean[2])*scPoses[i].weight/(1-sumWeightSquare);
        covariance[1][0] =  covariance[0][1];
        covariance[1][1] += (scPoses[i].y - mean[1])*(scPoses[i].y - mean[1])*scPoses[i].weight/(1-sumWeightSquare);
        covariance[1][2] += 0.0;//(scPoses[i].y - mean[1])*(scPoses[i].t - mean[2])*scPoses[i].weight/(1-sumWeightSquare);
        covariance[2][0] =  covariance[0][2];
        covariance[2][1] =  covariance[1][2];
        covariance[2][2] += (scPoses[i].t - mean[2])*(scPoses[i].t - mean[2])*scPoses[i].weight/(1-sumWeightSquare);
    }
}

// Takes a laser scan and position estimate and adds all points as obstacles in GPCostMap refScans
void ScanMatching::AddScanToRefScans(GPCostmap &refScans, Pose &pose, sensor_msgs::LaserScan *scan){

    float angle_min = pose.t + scan->angle_min;
	float angle_max = pose.t + scan->angle_max;
	float angle_increment = scan->angle_increment;
	float cur_angle = angle_min;

	for(int i = 0; cur_angle <= angle_max - angle_increment && i < 180; i = i + 1, cur_angle = cur_angle + angle_increment) {
		float dist = scan->ranges[i];

		if(dist == 12.0)
			continue;

		float x = pose.x + cos(cur_angle) * dist;
		float y = pose.y + sin(cur_angle) * dist;
		refScans.set((int)(x*10), (int)(y*10), 0);
	}

}

// Takes the current pose and scan and checks for overlap with the given refPose and refScan
bool ScanMatching::CheckScanOverlap(Pose &pose, sensor_msgs::LaserScan *scan, Pose &refPose, sensor_msgs::LaserScan *refScan){

    float angle_min = pose.t + scan->angle_min;
	float angle_max = pose.t + scan->angle_max;
	float angle_increment = scan->angle_increment;
	float cur_angle = angle_min;

    float x = 0;
    float y = 0;
    float num_scans = 0;
	for(int i = 0; cur_angle <= angle_max - angle_increment; i = i + 1, cur_angle = cur_angle + angle_increment) {
		float dist = scan->ranges[i];

		x += pose.x + cos(cur_angle) * dist;
		y += pose.y + sin(cur_angle) * dist;
		num_scans = num_scans + 1;
	}

	float x_avg = x/num_scans;
	float y_avg = y/num_scans;

    angle_min = refPose.t + refScan->angle_min;
	angle_max = refPose.t + refScan->angle_max;
	angle_increment = refScan->angle_increment;
	cur_angle = angle_min;

   x = 0;
   y = 0;
   num_scans = 0;
	for(int i = 0; cur_angle <= angle_max; i = i + 1, cur_angle = cur_angle + angle_increment) {
		float dist = refScan->ranges[i];

		x += refPose.x + cos(cur_angle) * dist;
		y += refPose.y + sin(cur_angle) * dist;
		num_scans = num_scans + 1;
	}

	float x_avg_ref = x/num_scans;
	float y_avg_ref = y/num_scans;

	if ((x_avg - x_avg_ref)*(x_avg - x_avg_ref) + (y_avg - y_avg_ref)*(y_avg - y_avg_ref) < SCAN_OVERLAP_CM){
        if ((pose.x - refPose.x)*(pose.x - refPose.x) + (pose.y - refPose.y)*(pose.y - refPose.y) < SCAN_OVERLAP){
            return true;
        }
	}

    return false;
}
// We can generate scan matching poses used the InitScanMatchingGaussian function, use AddScanToRefScans to create a reference scan GPCostmap,
// create a sensor model class and initialize the likelihood field using the the refScan GPCostmap, then use scResample to compute weights and
// get the mean and covariance

// If the mean and covariance and refPose are all in a fixed frame, this function will convert the mean and
// covariance into the frame of refPose
void ScanMatching::ChangeFrameMeanCov(double mean[3], double covariance[][3], Pose &refPose){

    mean[0] -= refPose.x;
    mean[1] -= refPose.y;
    mean[2] -= refPose.t;

    double theta = refPose.t;

    double a00 = cos(theta)*cos(theta)*covariance[0][0] + sin(theta)*sin(theta)*covariance[1][1] + cos(theta)*sin(theta)*(covariance[0][1] + covariance[1][0]);
    double a11 = cos(theta)*cos(theta)*covariance[1][1] + sin(theta)*sin(theta)*covariance[0][0] - cos(theta)*sin(theta)*(covariance[0][1] + covariance[1][0]);
    double a01 = cos(theta)*cos(theta)*covariance[0][1] - sin(theta)*sin(theta)*covariance[1][0] + cos(theta)*sin(theta)*(-covariance[0][0] + covariance[1][1]);
    double a10 = a01;

    covariance[0][0] = a00;
    covariance[0][1] = a01;
    covariance[1][0] = a10;
    covariance[1][1] = a11;

}




