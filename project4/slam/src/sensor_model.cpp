#include "sensor_model.h"

#include <ros/ros.h>
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



SensorModel::SensorModel() {
	numScansToUse = DEFAULT_NUM_SCANS;
	alpha0 = 0.1;
	sigma = 5;
	laserRange = 120;
	likelihoodField = NULL;
	transformListener = new tf::TransformListener(ros::Duration(10));
}

SensorModel::~SensorModel() {

}

void SensorModel::setParamsFromConfig(ros::NodeHandle &n) {
	ROS_INFO("Sensor Model Loading Params");

	double laser_r, a0, s, p_r, t_r, ep, z0, rxm, rym, rtm, rxv, ryv, rtv;

	n.getParam("robot_pose/num_scans_to_use", numScansToUse);
	n.getParam("robot_pose/laser_range", laser_r);
	n.getParam("robot_pose/alpha0", a0);
	n.getParam("robot_pose/sigma", s);
	n.getParam("robot_pose/pos_resolution", p_r);
	n.getParam("robot_pose/theta_resolution", t_r);
	n.getParam("robot_pose/epsilon", ep);
	n.getParam("robot_pose/z", z0);	
	n.getParam("robot_pose/max_k", maxK);
	n.getParam("robot_pose/min_k", minK);
	n.getParam("robot_pose/use_gaussian_during_resample", use_gaussian_during_resample);
	n.getParam("robot_pose/resample_x_mean", rxm);
	n.getParam("robot_pose/resample_y_mean", rym);
	n.getParam("robot_pose/resample_t_mean", rym);
	n.getParam("robot_pose/resample_x_var", rxv);
	n.getParam("robot_pose/resample_y_var", ryv);
	n.getParam("robot_pose/resample_t_var", rtv);
	
	laserRange = 10 * laser_r;
	alpha0 = a0;
	sigma = s;
	pos_resolution = p_r;
	theta_resolution = t_r;
	epsilon = ep;
	z = z0;

	resample_x_mean = rxm;
        resample_y_mean = rym;
        resample_t_mean = rtm;

        resample_x_var = rxv;
        resample_y_var = ryv;
        resample_t_var = rtv;
	
	ROS_INFO("Sensor Model Parameters Loaded:");
	ROS_INFO("using %d scans with laser range %fcm", numScansToUse, laserRange);
	ROS_INFO("likelihood alpha0 %f, sigma %f", alpha0, sigma);
	ROS_INFO("KLD with pos_r %f, th_r %f, epsilon %f, z %f", pos_resolution, theta_resolution, epsilon, z);
	ROS_INFO("Max poses %d, min poses %d", maxK, minK);
	ROS_INFO("Using gaussian during resample: %s", (use_gaussian_during_resample ? "TRUE" : "FALSE"));
	ROS_INFO("Parameter load complete\n\n\n");
}

float SensorModel::GenerateGaussian(float mean, float variance) {
	float x1, x2, w;
	do {
		x1 = 2.0 * drand48() - 1.0;
		x2 = 2.0 * drand48() - 1.0;
		w = x1 * x1 + x2 * x2;
	} while ( w >= 1.0 );

	w = sqrt( (-2.0 * log( w ) ) / w );
	return (x1*w*variance)+mean;

}

void SensorModel::setNumScansToUse(int num) {
	numScansToUse = num;
}

void SensorModel::setLikelihoodParameters(float a0, float sig, float laser_r) {
	alpha0 = a0;
	sigma = sig;
	laserRange = laser_r * 10;
	likelihoodField = NULL;
}

void SensorModel::setKLDParameters(float pos_r, float th_r, float ep, float z, int minK, int maxK) {
	pos_resolution = pos_r;
	theta_resolution = th_r;
	epsilon = ep;
	this->z = z;
	this->minK = minK;
	this->maxK = maxK;
}

void SensorModel::setLikelihoodField(const gradient_planner::GPCostmapMConstPtr& fieldMsg) {
	if(likelihoodField == NULL) {
		likelihoodField = new GPCostmap(fieldMsg->rows, fieldMsg->cols);
		float a = alpha0 / laserRange;
		float b = (1.0 - alpha0) / (sigma * sqrt(2 * M_PI));
		float s2 = 2 * sigma * sigma;
		likelihoodField->setAll(a);
		for(int i = 0; i < fieldMsg->data.size(); i++) {
			if(fieldMsg->data[i] >= laserRange) {
				likelihoodField->set(i, 0.0);
			} else {
				float p = a + b *exp(-(fieldMsg->data[i] * fieldMsg->data[i]) / s2);
				likelihoodField->set(i, p);
			}
		}		
	}
}

void SensorModel::setLikelihoodField(GPCostmap *newmap) {
  likelihoodField = newmap;

  if(likelihoodField != NULL) {
    float a = alpha0 / laserRange;
    float b = (1.0 - alpha0) / (sigma * sqrt(2 * M_PI));
    float s2 = 2 * sigma * sigma;
    likelihoodField->setAll(a);
    for(int i = 0; i < newmap->getNumElements(); i++) {
      if((newmap->getData())[i] >= laserRange) {
	likelihoodField->set(i, 0.0);
      } else {
	float p = a + b *exp(-((newmap->getData())[i] * (newmap->getData())[i]) / s2);
	likelihoodField->set(i, p);
      }
    }		
  }
}

GPCostmap *SensorModel::getLikelihoodField() {
	return likelihoodField;
}

void SensorModel::resample(sensor_msgs::LaserScan *scan, vector<Pose>& poses) {
	if(likelihoodField == NULL)
	  return;
	for(int i = 0; i < poses.size(); i++) {
//		ROS_INFO("Computing weights for pose[%d]", i);
		if(!computeWeight(poses[i], scan))
			return;
	}
	normalizeWeights(poses);
	generateNewPoses(poses);
}

void SensorModel::scResample(sensor_msgs::LaserScan *scan, vector<Pose>& poses) {
	if(likelihoodField == NULL)
	  return;
	for(int i = 0; i < poses.size(); i++) {
//		ROS_INFO("Computing weights for pose[%d]", i);
		if(!computeWeight(poses[i], scan))
			return;
	}
	normalizeWeights(poses);
}

void SensorModel::computePointCloud(sensor_msgs::PointCloud &cloud, Pose &pose, sensor_msgs::LaserScan *scan) {
	cloud.header.frame_id = "/odom";
	float angle_min = pose.t + scan->angle_min;
	float angle_max = pose.t + scan->angle_max;
	float angle_increment = scan->angle_increment;
	float cur_angle = angle_min;
	numScansToUse = 24;
	int interval = (int) ((angle_max - angle_min) / (angle_increment * numScansToUse));
	for(int i = 0; cur_angle <= angle_max && i < 180; i = i + interval, cur_angle = cur_angle + interval * angle_increment) {
		float dist = scan->ranges[i];
		geometry_msgs::Point32 pt;
		pt.x = pose.x + cos(cur_angle) * dist;
		pt.y = pose.y + sin(cur_angle) * dist;
		cloud.points.push_back(pt);
	}
	//ROS_INFO("POINT CLOUD COMPUTED USING %d LASER SCANS", cloud.points.size());
}

bool SensorModel::computeWeight(Pose &pose, sensor_msgs::LaserScan *scan) {
	sensor_msgs::PointCloud cloud;
	computePointCloud(cloud, pose, scan);
	
	sensor_msgs::PointCloud mapCloud = cloud;
	pose.weight = 1.0;
	for(int i = 0; i < numScansToUse - 1; i++) {
		geometry_msgs::Point32 pt = mapCloud.points[i];
		float interpolatedWeight = likelihoodField->interpolate(pt.x * 10, pt.y * 10);
		//ROS_INFO("Interpol pt.x: %f, pt.y: %f, weight: %f\n", pt.x, pt.y, interpolatedWeight);
		
		if(interpolatedWeight == 0.0)
			pose.weight = 0.0;//ROS_ERROR("!interpolated weight is zero for (%f, %f), translated as (%f, %f)", pt.x, pt.y, pt.x * 10, pt.y * 10);
		if(interpolatedWeight > 0) {
			pose.weight *= interpolatedWeight;
		} else if(interpolatedWeight < 0) {
			pose.weight = 0.0; //*= (alpha0 / laserRange);
			break;
		}
		//ROS_INFO("Pose weight [%d] = %f", i, pose.weight);
	}
	return true;
}

void SensorModel::normalizeWeights(vector<Pose>& poses) {
	ROS_INFO("\nsensor model normalizing weights from %d poses", poses.size());
	float sum = 0;
	float pre_min = -1;
	float pre_max = -1;
	for(int i = 0; i < poses.size(); i++) {
		sum += poses[i].weight;
		if(poses[i].weight < pre_min || pre_min == -1)
			pre_min = poses[i].weight;
		if(poses[i].weight > pre_max || pre_max == -1)
			pre_max = poses[i].weight;
	}
	
	float postSum = 0;
	float post_min = -1;
	float post_max = -1;
	for(int i = 0; i < poses.size(); i++) {
		if(poses[i].weight > 0.0)
			poses[i].weight /= sum;
		postSum += poses[i].weight;
		if(poses[i].weight < post_min || post_min == -1)
			post_min = poses[i].weight;
		if(poses[i].weight > post_max || post_max == -1)
			post_max = poses[i].weight;
	}
	ROS_INFO("pre-normalized:  max weight: %f min weight: %f total sum: %f", pre_max, pre_min, sum);
	ROS_INFO("post-normalized: max weight: %f min weight: %f total sum: %f\n\n", post_max, post_min, postSum);	
}

void SensorModel::generateNewPoses(vector<Pose>& poses) {
	vector<Pose> newPoses;
	int newSize = computeNewSize(poses);
//	int newSize = poses.size();
	int curPose = 0;
	float step = 1.0 / (float) newSize;
	float cur = (float)rand()/(float)RAND_MAX * step;
	ROS_INFO("begin generating %d new poses with step size %f. Initial %f", newSize, step, cur);

	float margin = poses[0].weight;
	for(int i = 0; i < newSize; i++, cur += step) {
		// adjust the position in the count array
		while(cur > margin && curPose < poses.size()) {	
			margin += poses[++curPose].weight;
		}
		Pose pose;
		pose.x = poses[curPose].x ;
		pose.y = poses[curPose].y ;
		pose.t = poses[curPose].t ;
		
		if(use_gaussian_during_resample) {
			pose.x = poses[curPose].x + GenerateGaussian(resample_x_mean, resample_x_var);
			pose.y = poses[curPose].y + GenerateGaussian(resample_y_mean, resample_y_var);
			pose.t = poses[curPose].t + GenerateGaussian(resample_t_mean, resample_t_var);
		}	
		pose.weight = 1.0;
		newPoses.push_back(pose);
	}
	poses.clear();
	for(int i = 0; i < newPoses.size(); i++)
		poses.push_back(newPoses[i]);
}

void SensorModel::recommendPose(vector<Pose>& poses, Pose& recommended) {
	ROS_INFO("begin recommending pose");
	map<string, vector<Pose> > binMap;
	binPoses(binMap, poses);
	string largestBin = "";
	int largestCount = 0;
	ROS_INFO("bin map size %d", binMap.size());
	map<string, vector<Pose> >::iterator end = binMap.end(); 
	for (map<string, vector<Pose> >::iterator it = binMap.begin(); it != end; it++) {
		if(it->second.size() > largestCount) {
			largestCount = it->second.size();
			largestBin = it->first;
		}
	}
	if(largestCount > 0) {
		ROS_INFO("largest bin is %d poses", largestCount);
		recommended.x = 0; recommended.y = 0; recommended.t = 0;
		for(int i = 0; i < binMap[largestBin].size(); i++) {
			recommended.x += binMap[largestBin][i].x;
			recommended.y += binMap[largestBin][i].y;
			recommended.t += binMap[largestBin][i].t;
		}
		recommended.x /= (float) binMap[largestBin].size();
		recommended.y /= (float) binMap[largestBin].size();
		recommended.t /= (float) binMap[largestBin].size();
		ROS_INFO("recommending pose (x, y, th): (%f, %f ,%f)", recommended.x, recommended.y, recommended.t);
	} else {
		ROS_INFO("skipping, could not compute largest");
	}
}

int SensorModel::computeNewSize(vector<Pose> &poses) {
	int initialSize = poses.size();
	int k = computeKValue(poses);
	int newSize = initialSize;
	if(k > 1) {
		int suggested = (int) ((float) k - 1.0) / (2.0 * epsilon) * pow((1.0 - 2.0 / (9.0 * (k-1)) + sqrt(2.0 / (9.0 * (k - 1))) * z), 3);
		ROS_INFO("Suggesting using %d new poses", suggested);
		if(suggested >= minK && suggested <= maxK)
			newSize = suggested;
		else
			ROS_INFO("suggestion rejected, maintaining %d poses", newSize);
	}
	return newSize;
}

int SensorModel::computeKValue(vector<Pose>& poses) {
	map<string, int> binMap;
	binPoses(binMap, poses);
	ROS_INFO("KLD: %d poses placed in %d bins", poses.size(), binMap.size());
	return binMap.size();
}

string SensorModel::poseToString(Pose& pose) {
	char buffer[4];
	buffer[0] = valueToChar(pose.x, pos_resolution);
	buffer[1] = valueToChar(pose.y, pos_resolution);
	buffer[2] = valueToChar(pose.t, theta_resolution);
	buffer[3] = '\0';
	string str = buffer;
	return str;
}

char SensorModel::valueToChar(float value, float resolution) {
	return (char) (int) (value * resolution);
}

void SensorModel::binPoses(map<string, int>& binMap, vector<Pose>& poses) {
	for(int i = 0; i < poses.size(); i++) {
		string posStr = poseToString(poses[i]);
		if(binMap.find(posStr) == binMap.end())
			binMap.insert(pair<string, int>(posStr, 1));
		else
			binMap[posStr]++;
	}	
}

void SensorModel::binPoses(map<string, vector<Pose> >& binMap, vector<Pose>& poses) {
	for(int i = 0; i < poses.size(); i++) {
		string posStr = poseToString(poses[i]);
		if(binMap.find(posStr) == binMap.end()) {
			vector<Pose> binPoses;
			binPoses.push_back(poses[i]);
			binMap[posStr] = binPoses;
		} else {
			binMap[posStr].push_back(poses[i]);
		}
	}
}
