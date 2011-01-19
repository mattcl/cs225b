#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "visualization_msgs/Marker.h"
#include <math.h>

#define V_ACCEL_LIMIT 0.3 //Linear acceleration limit in m/s^2
#define W_ACCEL_LIMIT 0.6 //Angular acceleration limit in radian/s^2
#define V_SAMPLES 100 //Number of discretization in V
#define W_SAMPLES 100 
#define PATH_SAMPLES 100//Number of discretization in calculating path cost
#define TIME_WINDOW 2.0//Length of time considered in the DWA algorithm
//parameters for objective function
#define ALPHA 0.8
#define BETA 0.1
#define GAMMA 0.1
#define DELTA 1/(ALPHA+BETA+GAMMA)


class TBK_Node
{
  private:
    geometry_msgs::Twist cmdvel;
    ros::NodeHandle n_;
    ros::Publisher pub_;
  bool hasCollision (double X, double Y);

  public:
    TBK_Node()
    {
      // publisher to publish the base velocity	
      pub_ = n_.advertise<geometry_msgs::Twist>("base_controller/command",1);
    }
    ~TBK_Node() { }
    void dwaLoop();
    void stopRobot()
    {
      cmdvel.linear.x = cmdvel.angular.z = 0.0;
      pub_.publish(cmdvel);
    }
};

void TBK_Node::dwaLoop() {
  double resultV = NULL;
  double resultW = NULL;
  double currentV = NULL;
  double currentW = NULL;
  double currentX = NULL;
  double currentY = NULL;
  double currentZ = NULL;
  
  double highestValue = 0; //0 is output if there's no possible path
  //loop through all angular speed samples
  for (int i=0; i < W_SAMPLES; ++i) {
    //loop through all linear speed samples
    for (int j=0; j < V_SAMPLES; ++j) {
      double sampleV = (((V_SAMPLES/2)-j)/(V_SAMPLES/2)*V_ACCEL_LIMIT)+currentV;
      double sampleW = (((W_SAMPLES/2)-j)/(W_SAMPLES/2)*W_ACCEL_LIMIT)+currentW;

      //Euclidean method for finding total path objective fn
      //Maximize the objective function!
      double totalValue = 0;
      double searchX = currentX;
      double searchY = currentY;
      double searchV = currentV;
      double searchW = currentW;
      double searchZ = currentZ;

      //Calculate total objective fn along path
      for (int k=0; k < PATH_SAMPLES; ++k) {
	//Current V,W,Z at that sample point
	searchV += (i*((sampleV-currentV)/PATH_SAMPLES))+currentV; 
	searchW += (i*((sampleW-currentW)/PATH_SAMPLES))+currentW;
	searchZ += ((searchW*TIME_WINDOW)/PATH_SAMPLES);
	
	searchX += ((TIME_WINDOW*searchV)/PATH_SAMPLES)*cos(searchZ);
	searchY += ((TIME_WINDOW*searchV)/PATH_SAMPLES)*sin(searchZ);
	
	//Check for collision first
	if(hasCollision(searchX, searchY)) {
	  totalValue = 0;
	  break;
	}
	totalCost += DELTA(ALPHA*Heading(searchX,searchY)+BETA*Obstacle(searchX,searchY)+GAMMA*searchV);
      }

      if(totalValue > highestValue && highestValue !=0) {
	resultV = sampleV;
	resultW = sampleW;
	//First time sample - take whatever nonzero value
      } else if (highestValue==0 && totalValue!=0) {
	resultV = sampleV;
	resultW = sampleW;
      }	  
    }
  }

}

bool TBK_Node::hasCollision (double X, double Y) {
  return true;
}

TBK_Node* tbk;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamic_window");

  TBK_Node tbk;

  boost::thread t = boost::thread::thread(boost::bind(&TBK_Node::dwaLoop, &tbk));

  ros::spin();

  t.interrupt();
  t.join();
  tbk.stopRobot();
 

  return 0;
}
