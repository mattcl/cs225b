#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include <math.h>
ros::Publisher tracker_pub;

void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //Find the closest point
  int nPoints = ((msg->angle_max-msg->angle_min)/msg->angle_increment)+1;
  double closestPoint = msg->range_max+1;
  double closestPointAngle = 0;

  for(int i = 0; i < nPoints; ++i) {
    if((msg->ranges[i] < closestPoint) && (msg->ranges[i] >= msg->range_min)
       && (msg->ranges[i] <= msg->range_max)) {
      closestPoint = msg->ranges[i];
      closestPointAngle = msg->angle_min+(i*(msg->angle_increment));
    }
  }

  //Debug: print closest point to the screen
  if(closestPoint >= msg->range_min && closestPoint <= msg->range_max) {
    ROS_INFO("ClosestAngle: [%f]", closestPointAngle*180/(3.14159));

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = closestPoint*cos(closestPointAngle);
    marker.pose.position.y = closestPoint*sin(closestPointAngle);
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    tracker_pub.publish(marker);
  }
  ros::spinOnce();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_listener");

  ros::NodeHandle n;
  tracker_pub = n.advertise<visualization_msgs::Marker>("collisionWarning", 1000);
  ros::Subscriber laser_sub = n.subscribe("base_scan", 1000, chatterCallback);
  ros::spin();

  return 0;
}
