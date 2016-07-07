#include "ros/ros.h"                    
#include "geometry_msgs/Pose.h"                                    
#include "geometry_msgs/Twist.h"                                 
#include "nav_msgs/Odometry.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <cmath>


// Initialize a callback to get velocity values:

double xcur = 0;
double ycur = 0;
double thetacur = 0;

double xcurLast = 0;
double ycurLast = 0;
double thetacurLast = 0;

double xcurinit= 0;
double ycurinit = 0;
double thetacurinit = 0;

double xsonarVal = 0;
double ysonarVal = 0;
double xsonarVal4 = 0;
double xsonarVal5 = 0;
double distancePt;

double velocity;
double distance; // used for velocity calculation

int i=0;

double getBrakingDistance();
double Distance(double x, double y);
void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);

void sonarCallback(const sensor_msgs::PointCloud::ConstPtr& msg2);


int main(int argc, char** argv) {

	ros::init(argc, argv, "sonar_test");
	ros::NodeHandle n; 
	ros::Subscriber sub = n.subscribe<nav_msgs::Odometry>("/RosAria/pose", 1, poseCallback);
	ros::Subscriber sub2 = n.subscribe<sensor_msgs::PointCloud>("/RosAria/sonar", 1, sonarCallback);
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 100);
	ros::Rate rate(10); // times per second to check our position
	ros::spinOnce();



	
	int j = 0;
	geometry_msgs::Twist msg;
	sensor_msgs::PointCloud msg2;

	while (ros::ok()){	
	if (xsonarVal4 > 1000 && xsonarVal5 > 1000) { // sonar distance
		msg.linear.x = 0.1; // controls speed
	  	msg.linear.y = 0;
	  	msg.linear.z = 0;
	  	msg.angular.x = 0;
	  	msg.angular.y = 0;
	  	msg.angular.z = 0;
	  	pub.publish(msg);
	
		rate.sleep();
	
		ros::spinOnce();
		
	}

	else {
		msg.linear.x = 0;
		msg.linear.y = 0;
		msg.linear.z = 0;
		msg.angular.x = 0;
	 	msg.angular.y = 0;
	 	msg.angular.z = 0;
 	 	pub.publish(msg);

		rate.sleep();
		
		ros::spinOnce();
			
	}


	}
}



double getBrakingDistance() {
	double dist;
	//double arrayVelocity[10] = {0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1};
	//double brakingDistance[10] = {0.04,0.12,0.24,0.35,0.53,0.70,0.99,1.12,1.13,1.14};
	
	if (velocity <= 0.75)
		dist = 1.7463*velocity*velocity + 0.1283*velocity + 0.0212;
	else
		dist = 1.15;

		//dist = 0.1257*velocity + 1.0133;


	return dist;
}


void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) { 
	if (i == 0){
	  xcurinit = msg->pose.pose.position.x;
	  ycurinit = msg->pose.pose.position.y;
	  thetacurinit = msg->pose.pose.orientation.z;
	  ROS_INFO("Init values recieved: x = %f, y = %f, theta = %f", xcurinit, ycurinit, thetacurinit);
	  
		}
	else {

	  if (i != 1) { 
	  // if not first time running, then set the previous coordinates
		xcurLast = xcur;
		ycurLast = ycur;
		thetacurLast = thetacur;
	  }

	  xcur = msg->pose.pose.position.x;
	  ycur = msg->pose.pose.position.y;
	  thetacur = msg->pose.pose.orientation.z; // find the new position

	  xcur -= xcurinit; 
	  ycur -= ycurinit;
	  thetacur -= thetacurinit; // apply (0,0,0) correction

	  // calculate velocity = distance / time
	  // distance formula
	  distance = (double)pow((double)pow(xcur - xcurLast, 2) + (double)pow(ycur - ycurLast , 2) , 0.5); // pow(base, exponent)

	  velocity = (double)distance * 10;



	  ROS_INFO("Odometer Values: x = %f, y = %f, theta = %f, velocity = %f", xcur, ycur, thetacur, velocity);


	}

	i++;
}

void sonarCallback(const sensor_msgs::PointCloud::ConstPtr& msg2) { 
	for (int k=0; k<8; k++){
		xsonarVal = msg2->points[k].x;
		ysonarVal = msg2->points[k].y;
		xsonarVal *= 1000;
		ysonarVal *= 1000;
		distancePt = Distance(xsonarVal, ysonarVal);
		ROS_INFO("Sonar Values: x = %f, y = %f, distance from point = %f", xsonarVal, ysonarVal, distancePt);	
		

		
		
  	}
	
		xsonarVal4 = 1000*msg2->points[4].x;
		xsonarVal5 = 1000*msg2->points[5].x;

}

double Distance(double x, double y){
	double dis, res;
	res = pow(x,2) + pow(y,2);
	dis = pow(res,0.5);
	return dis;
}

