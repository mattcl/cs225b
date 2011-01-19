#ifndef scan_matching_h
#define scan_matching_h

#include <vector>
#include "GPCostmap.h"
#include "pose.h"
#include <sensor_msgs/LaserScan.h>
#include "graph.h"
#include "sensor_model.h"

using namespace std;

const float SCAN_OVERLAP_CM = 100;
const float SCAN_OVERLAP = 25;
class ScanMatching {
	public:
	SensorModel model;
	/*==============================================*/
	/* Variable declaration                         */
	/*==============================================*/

	/*==============================================*/
	/* Function declaration                         */
	/*==============================================*/
	
	ScanMatching();

	void match(node_t &node, vector<node_t> refNodes, bounds_t bounds, double mean[3], double covariance[][3]);

	// This algorithm crunches numbers and give out a number in Gaussian distribution. Magic black box.
	float GenerateGaussian(float mean, float variance);
	// This function will initialize 'scposes' with a Gaussian distribution specified by the input parameters
	void InitScanMatchingGaussian(Pose &pose, float varXY, float varT, vector<Pose>& scPoses, int numPoses);
	// Given a vector of weighted poses, this function will return the mean and covariance matrix corresponding to it
	void CalculateMeanCovariance(vector<Pose>& scPoses, double mean[3], double covariance[][3]);
	// Takes a laser scan and position estimate and adds all points as obstacles in GPCostMap refScans
	void AddScanToRefScans(GPCostmap &refScans, Pose &pose, sensor_msgs::LaserScan *scan);
	// Takes the current pose and scan and checks for overlap with the given refPose and refScan
	bool CheckScanOverlap(Pose &pose, sensor_msgs::LaserScan *scan, Pose &refPose, sensor_msgs::LaserScan *refScan);
	// If the mean and covariance and refPose are all in a fixed frame, this function will convert the mean and
	// covariance into the frame of refPose

	void ChangeFrameMeanCov(double mean[3], double covariance[][3], Pose &refPose);

};

#endif
