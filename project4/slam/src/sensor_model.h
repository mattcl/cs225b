#ifndef _sensor_model_h
#define _sensor_model_h

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <gradient_planner/GPCostmapM.h>
#include "GPCostmap.h"
#include <vector>
#include <map>
#include "pose.h"
#include <math.h>
using namespace std;

const int DEFAULT_NUM_SCANS = 20;

class SensorModel {
	public:
	SensorModel();
	~SensorModel();
	void setParamsFromConfig(ros::NodeHandle &n);
	void setNumScansToUse(int num);
	void setLikelihoodParameters(float a0, float sig, float laser_r);
	void setKLDParameters(float pos_r, float th_r, float ep, float z, int minK, int maxK);
	void setLikelihoodField(const gradient_planner::GPCostmapMConstPtr& fieldMsg);
	void setLikelihoodField(GPCostmap *newmap);
	GPCostmap *getLikelihoodField();
	void resample(sensor_msgs::LaserScan *scan, vector<Pose>& poses);
	void scResample(sensor_msgs::LaserScan *scan, vector<Pose>& poses);
	void recommendPose(vector<Pose>& poses, Pose& recommended);

	private:
	int numScansToUse;
	float laserRange;
	float alpha0; 
	float sigma; // variance

	// scaling factors
	float pos_resolution;
	float theta_resolution;

	float epsilon;
	float z;

	int minK;
	int maxK;
		
	bool use_gaussian_during_resample;
	float resample_x_mean;
	float resample_y_mean;
	float resample_t_mean;

	float resample_x_var;
	float resample_y_var;
	float resample_t_var;

	tf::TransformListener *transformListener;
	GPCostmap *likelihoodField;
	void computePointCloud(sensor_msgs::PointCloud &cloud, Pose &pose, sensor_msgs::LaserScan *scan);
	bool computeWeight(Pose &pose, sensor_msgs::LaserScan *scan);
	void normalizeWeights(vector<Pose>& poses);
	void generateNewPoses(vector<Pose>& poses);
	int computeNewSize(vector<Pose>& poses);
	int computeKValue(vector<Pose>& poses);
	string poseToString(Pose& pose);
	char valueToChar(float value, float resolution);
	void binPoses(map<string, int>& binMap, vector<Pose>& poses);
	void binPoses(map<string, vector<Pose> >& binMap, vector<Pose>& poses);
	float GenerateGaussian(float mean, float variance);
};

#endif
