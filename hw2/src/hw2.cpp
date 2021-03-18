// include ros library
#include "ros/ros.h"

// include msg library
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

// include math 
#include <math.h>

using namespace std;

turtlesim::Pose pose;
geometry_msgs::Twist vel_msg;
geometry_msgs::Point goal_point;
//Define a data structure to 3D
struct XYZ{
  float x;
  float y;
  float z;
};

//Parameter of controller
float p_v=1;
float p_a=1;
float err_norm=0;
float err_theta=0;

//Declare a variable.Its name is pos_err with XYZ data type
struct XYZ pos_err;

// declare call back function(call back the pose of robot)
void pos_cb(const turtlesim::Pose::ConstPtr& msg)
{
  pose = *msg;
} 

void rotate2D(float &x, float &y, float theta)
{
	float x1 = x;
	float y1 = y;
	x = cos(theta) * x1 - sin(theta) * y1;
	y = sin(theta) * x1 + cos(theta) * y1;
} 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tutorial_1");
  ros::NodeHandle n;

  // declare publisher & subscriber
  ros::Subscriber pos_sub = n.subscribe<turtlesim::Pose>("turtle1/pose", 10, pos_cb);
  ros::Publisher turtlesim_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);
  //input your desired position
  ROS_INFO("Please input (x,y). x>0,y>0");
  cout<<"desired_X:";
  cin>>goal_point.x;
  cout<<"desired_Y:";
  cin>>goal_point.y;
  // setting frequency as 10 Hz
  ros::Rate loop_rate(10);
  
  //printf("atan2(1,1): %f\natan2(1,-1): %f\natan2(-1,-1): %f\natan2(-1,1):   %f\n",atan2(1,1),atan2(1,-1),atan2(-1,-1),atan2(-1,1));
  
  int count = 0;
  while (ros::ok()){

    printf("\ncount : %d\n",count);
    printf("goal x : %f \t y : %f\n",goal_point.x,goal_point.y);
    printf("pose x : %f \t y : %f\n",pose.x,pose.y);
    printf("err_theta: %f \n",err_theta);
    printf("pose theta: %f \n",pose.theta);    

    
    // Calculate position error(feedback term)
    pos_err.x = goal_point.x - pose.x;
    pos_err.y = goal_point.y - pose.y;
    // Find the goal_point position in Body(turtlesim) frame
	rotate2D(pos_err.x, pos_err.y, -pose.theta);
    err_norm = hypot(pos_err.x, pos_err.x);
    if (err_norm >= 1) err_norm = 1;
    
    // Calculate angular error(feedback term)
    err_theta = atan2(pos_err.y,pos_err.x);
    
    //control input 
    vel_msg.linear.x = p_v*err_norm;
    vel_msg.angular.z = p_a*err_theta;

    
    turtlesim_pub.publish(vel_msg);

    count ++;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}



