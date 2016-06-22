#include "ros/ros.h"                               
#include "geometry_msgs/Twist.h"                                 
#include "nav_msgs/Odometry.h" 


// Initialize a callback to get velocity values:
//

double xtarg = 0.2;
double ytarg = 0;
double thetatarg = 0;

double xcur = 0;
double ycur = 0;
double thetacur = 0;

double xcurinit= 0;
double ycurinit = 0;
double thetacurinit = 0;

double accuracy = 0.05;
double dx = 0;

double distancetraveled = 0.5;

int i=0;

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) { 
	if (i==0){
	  xcurinit = msg->pose.pose.position.x;
	  ycurinit = msg->pose.pose.position.y;
	  thetacurinit = msg->pose.pose.orientation.z;
	  ROS_INFO("Init values recieved: x = %f, y = %f, theta = %f", xcurinit, ycurinit, thetacurinit);
	  i++;
		}
	else {
	  xcur = msg->pose.pose.position.x;
	  ycur = msg->pose.pose.position.y;
	  thetacur = msg->pose.pose.orientation.z;

	  xcur -= xcurinit;
	  ycur -= ycurinit;
	  thetacur -= thetacurinit;

	  ROS_INFO("x = %f, y = %f, theta = %f", xcur, ycur, thetacur);
	}
	
}


int main(int argc, char** argv) {

  ros::init(argc, argv, "pose_nav");
  ros::NodeHandle n; 
  ros::Subscriber sub = n.subscribe<nav_msgs::Odometry>("/RosAria/pose", 1, poseCallback);
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 100);
  ros::Rate rate(5);
  ros::spinOnce();



/*
     dx = sqrt((xtarg - xcur)*(xtarg - xcur));
  
while (ros::ok()){

	if (dx/accuracy <= 1)

	{ 

	geometry_msgs::Twist msg;

	rate.sleep();	
	msg.linear.x = 0;
  	msg.linear.y = 0;
  	msg.linear.z = 0;
  	msg.angular.x = 0;
  	msg.angular.y = 0;
  	msg.angular.z = 0;
  	pub.publish(msg);
	ros::Duration(1).sleep();

  	ROS_INFO("x1 = %f, y1 = %f, theta1 = %f", xcur, ycur, thetacur);

	}

	else 
	*/
	//{ 
	
	geometry_msgs::Twist msg;

	while (ros::ok()) {
		msg.linear.x = 0.1;
	  	msg.linear.y = 0;
	  	msg.linear.z = 0;
	  	msg.angular.x = 0;
	  	msg.angular.y = 0;
	  	msg.angular.z = 0;
	  	pub.publish(msg);
	
		rate.sleep();
	
		ros::spinOnce();

	if (xcur >= 1) {
		msg.linear.x = 0;
  		msg.linear.y = 0;
  		msg.linear.z = 0;
  		msg.angular.x = 0;
 	 	msg.angular.y = 0;
 	 	msg.angular.z = 0;

 	 	pub.publish(msg);

		rate.sleep();

		break;
	}

	}

	/*msg.linear.x = 0;
  	msg.linear.y = 0;
  	msg.linear.z = 0;
  	msg.angular.x = 0;
  	msg.angular.y = 0;
  	msg.angular.z = 0;
  	pub.publish(msg);*/

}
