// Team Robot Unicorn: Arunanshu Roy, Budi Waskit, Matthew Chum-Lum
// This code creates a new frame fixed in the /odom frame which will move to the /base_footprint frame when 't' is pressed

// The code for polling the keyboard has been used from the teleop_erratic file.
// The rest of the code has been written by the team.

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#define KEYCODE_T 0x74

#define COMMAND_TIMEOUT_SEC 0.2


class FB_Node
{
  private:
    
    ros::NodeHandle node;
    tf::TransformBroadcaster br;
    
    // this will specify the transform between the odom and my_frame
    tf::Transform transform;

    // declaring a transform listener to listen to transforms between frames
    tf::TransformListener listener;
    
    // used while getting frame transformations
    tf::StampedTransform stamped_transform, transform_my_frame_base_footprint;

  public:
    FB_Node()
    {
      //set the intial transform between odom and my_frame
      transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0) );
      transform.setRotation( tf::Quaternion(0, 0, 0) );
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "my_frame"));
    }
    ~FB_Node() { }
    void keyboardLoop();
};

FB_Node* fb;
int kfd = 0;
struct termios cooked, raw;
bool done;

int
main(int argc, char** argv)
{
  ros::init(argc,argv,"fb", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
  FB_Node fb;

  boost::thread t = boost::thread::thread(boost::bind(&FB_Node::keyboardLoop, &fb));
  
  ros::spin();

  t.interrupt();
  t.join();
  tcsetattr(kfd, TCSANOW, &cooked);

  return(0);
}

void
FB_Node::keyboardLoop()
{
  char c;
  
  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("---------------------------");
  puts("Press t to move my_frame to the robot");
  puts("---------------------------");

  struct pollfd ufd;
  ufd.fd = kfd;
  ufd.events = POLLIN;
  for(;;)
  {
    boost::this_thread::interruption_point();
    
    // get the next event from the keyboard
    int num;
    if((num = poll(&ufd, 1, 250)) < 0)
    { 
      perror("poll():");
      return;
    }
    else if(num > 0)
    {
      if(read(kfd, &c, 1) < 0)
      {
        perror("read():");
        return;
      }
    }
    else
    {
       br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "my_frame"));
       continue;
    }
    
    if (c == KEYCODE_T)	// 't' was pressed on the keyboard. Moving /my_frame to /base_footprint
    {
     
      try{
	// Find the latest tranfrom between odom and base_footprint
        listener.lookupTransform("odom", "base_footprint", ros::Time(0), stamped_transform);
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
      }
      
      // Get the origin and rotation information from the stamped transform and broadcast it as the new transform between /odom and /my_frame	
      transform.setOrigin(stamped_transform.getOrigin());
      transform.setRotation(stamped_transform.getRotation());
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "my_frame"));

    }
  }
}
