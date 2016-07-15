// Made by Jonatan Yanovsky on 7/15/2016
// Uses parts of functions and code written by group members of Robot Butler Project 2016


// This code will supplement the ROS navigation stack, which does not allow controlling
// the p3dx robot through code. Instead, the navigation stack allows the robot to move from
// point a to point b, but does not allow discrete control when we want to spin the robot 
// 180 degrees and reverse into the charging station. This code will allow us to perform
// any maneuver we want, but is NOT meant to replace the ROS navigation stack.

// Eventually, this code may be expanded to store the locations of known locations and may
// pass those coordinates to the navigation stack which will then move the robot the those
// coordinates.

// This code is based on discrete commands like "move 2 meters forward," and thus is not
// meant to avoid obstacles, as the navigation stack can do that perfectly. Also, we will not
// be using the mapped environment here, but instead the robot's sonars. I assume that 
// the navigation stack already brought the robot to a known position, such as the charging
// station or door 1, and that all that is left to do is to perform a custom maneuver that the
// navigation stack is unable to perform.

// custom maneuvers may be coded into this cpp file in the format of a function. A function
// backUpIntoChargingStation() may be called, assuming we already know that we are facing
// the correct angle to do so.


#include "ros/ros.h"             
#include "geometry_msgs/Pose.h"                    
#include "geometry_msgs/Twist.h"                                 
#include "nav_msgs/Odometry.h" 
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <cmath> // for pow()


//Variables
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


double velocity;
double distance; // used for velocity calculation


int i=0;

double getBrakingDistance();
void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);
void sonarCallback(const sensor_msgs::PointCloud::ConstPtr& msg2);
void backUpIntoChargingStation();

int main(int argc, char** argv) {

	ros::init(argc, argv, "master");

	// 1. call the navigation stack to move the robot to face the entrance of the charging
	// station.

	// 2. spin 180 degrees, and back up until our sonars detect that we are inside 
	// the charging station

	backUpIntoChargingStation();

	// 3. sleep until charged, or for a predefined amount of time, or until a new order was
	// given

	// 4. a new order was given: get the coffee

	// 5. call navigation stack to move the robot to face the front of the coffee terminal

	// 6. use the drone to get the coffee, and land it on robot

	// 7. call navigation stack to move the robot to face the user

	
return 0;

}//end int main


// assume we are already facing the charging station
void backUpIntoChargingStation() {

	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe<nav_msgs::Odometry>("/RosAria/pose", 1, poseCallback);
	ros::Subscriber sub2 = n.subscribe<sensor_msgs::PointCloud>("/RosAria/sonar", 1, sonarCallback);
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 100);
	ros::Rate rate(10); // times per second to check our position
	ros::spinOnce();

	int j = 0;
	geometry_msgs::Twist msg;
	sensor_msgs::PointCloud msg2;

	// note: don't need while(ros::ok()) since our code is linear.

	while ((sonarDistanceValues[3] > 800) && (sonarDistanceValues[4] > 800)) {
		msg.linear.x = 0.2; 
		msg.linear.y = 0;
		msg.linear.z = 0;
		msg.angular.x = 0; // go straight
		msg.angular.y = 0;
		msg.angular.z = 0;
		pub.publish(msg);
		rate.sleep();
		ros::spinOnce();
	}

	while (thetacur <= 0.98) { 
		msg.linear.x = 0;
		msg.linear.y = 0;
		msg.linear.z = 0;
		msg.angular.x = 0; // rotate 180 degrees
		msg.angular.y = 0;
		msg.angular.z = 1;
		pub.publish(msg);
		rate.sleep();
		ros::spinOnce();
	}

	while ((sonarDistanceValues[0] > 800) && (sonarDistanceValues[7] > 800)) {
		msg.linear.x = -0.1; 
		msg.linear.y = 0;
		msg.linear.z = 0;
		msg.angular.x = 0; // reverse into charging station
		msg.angular.y = 0;
		msg.angular.z = 0;
		pub.publish(msg);
		rate.sleep();
		ros::spinOnce();
	}

	msg.linear.x = 0; 
	msg.linear.y = 0;
	msg.linear.z = 0;
	msg.angular.x = 0; // stop
	msg.angular.y = 0;
	msg.angular.z = 0;
	pub.publish(msg);
	rate.sleep();
	ros::spinOnce();

}


double getBrakingDistance() {
		
	if (velocity <= 0.75)
		return 1.7463*velocity*velocity + 0.1283*velocity + 0.0212;

	return 1.15;

}


//  to init poseCallback, set i = 0; in code and call spinOnce();
double distanceFromLastInit; // used to see how far you have gone from the point at which "i" was set to 0
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

	  // find the distance since last init
	  distanceFromLastInit = sqrt(pow(xcur - xcurinit , 2) + pow(ycur - ycurinit , 2)); // distance formula

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


// gets the distance that the sonar detected something at, not just (x,y) coordinates
double sonarDistanceValues[8]; // sensors 0 to 7
void sonarCallback(const sensor_msgs::PointCloud::ConstPtr& msg2) { 

	for (int k=0; k<8; k++){
		xsonarVal = msg2->points[k].x;
		ysonarVal = msg2->points[k].y;
		xsonarVal *= 1000;
		ysonarVal *= 1000;
		
		// set distance of all sonar values
		sonarDistanceValues[k] = sqrt(pow(xsonarVal , 2) + pow(ysonarVal , 2)); // pythagorean theorem
		
		ROS_INFO("Sonar Values: x = %f, y = %f", xsonarVal, ysonarVal);	
		
  	}

}
