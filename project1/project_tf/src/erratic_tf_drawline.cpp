// Team Robot Unicorn: Arunanshu Roy, Budi Waskit, Matthew Chum-Lum
// This code draws a line from origin of /my_frame to origin of /base_footprint frame 

// This code is based on the tutorial for drawing lines in rviz and the tutorial for creating a listener in tf

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv){
  
  // initializing the node	
  ros::init(argc, argv, "my_tf_linedraw");
  ros::NodeHandle node;
  
  // publisher for drawing a line
  ros::Publisher marker_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  // declaring a transform listener to listen to transforms between frames
  tf::TransformListener listener;
  // used while getting frame transformations
  tf::StampedTransform stamped_transform, transform_my_frame_base_footprint;
  

  ros::Rate rate(10.0);

  while (node.ok()){

	//drawing a line in from origin of /my_frame to the origin of /base_footprint
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = "/my_frame";
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "points_and_lines";
        line_strip.action= visualization_msgs::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;

        line_strip.id = 0;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        // LINE_STRIP markers use only the x component of scale, for the line width
        line_strip.scale.x = 0.05;
        // Line strip is blue
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;

        geometry_msgs::Point p1, p2;
    
        p1.x = 0.0;
        p1.y = 0.0;
        p1.z = 0.0;
	
        line_strip.points.push_back(p1);
    
        try{
            listener.lookupTransform("my_frame", "base_footprint", ros::Time(0), transform_my_frame_base_footprint);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }

        p2.x = transform_my_frame_base_footprint.getOrigin().x();
        p2.y = transform_my_frame_base_footprint.getOrigin().y();
        p2.z = transform_my_frame_base_footprint.getOrigin().z();
	
        line_strip.points.push_back(p2);
    
        marker_pub.publish(line_strip);    		

        rate.sleep();
  }
  return 0;
};

