#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

tf::Transform mapOdomTF;
ros::Time ts;

void rvizCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
    ROS_INFO("received initial pose from rviz");
    tf::Quaternion truthQ;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, truthQ);
	ts = msg->header.stamp;

    mapOdomTF = tf::Transform(truthQ, tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0));
}



int main(int argc, char** argv){
 	ros::init(argc, argv, "project1_part_4");
	ros::NodeHandle node;
	ros::Subscriber rviz_sub = node.subscribe("initialpose", 100, &rvizCallback);
	ros::Rate r(10);

	mapOdomTF = tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));

	tf::TransformBroadcaster b;

	while(ros::ok()) {
		ros::spinOnce();
		b.sendTransform(tf::StampedTransform(mapOdomTF, ros::Time::now(), "map", "odom"));
		r.sleep();
	}
	return 0;
};

