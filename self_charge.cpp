#include "ros/ros.h"                               
#include "geometry_msgs/Twist.h"                                 
#include "nav_msgs/Odometry.h" 
#include <cmath> // for pow()


// NOTES: must turn robot on/off every time we do [rosrun pose_nav pose_nav]

// BUG: cannot reset local position perfectly. If we turn 180 degrees, then the robot still remembers which direction it started from.
// this means that if we tell it to move backward (negative direction), it lists its x-position as positive

// Initialize a callback to get velocity values:


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

double velocity;
double distance; // used for velocity calculation

int i=0;

double getBrakingDistance();

void getOdometerData(const nav_msgs::Odometry::ConstPtr& msg); // aka callback


int main(int argc, char** argv) {

	ros::init(argc, argv, "pose_nav");
	ros::NodeHandle n; 
	ros::Subscriber sub = n.subscribe<nav_msgs::Odometry>("/RosAria/pose", 1, getOdometerData);
	//ros::Publisher pub2 = n.advertise<nav_msgs::Odometry>("/RosAria/pose", 100);
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 100);
	ros::Rate rate(10); // times per second to check our position
	rate.sleep();	
	ros::spinOnce();



	geometry_msgs::Twist msg;


	//nav_msgs::Odometry msg2;

	/*msg2.pose.pose.position.x = 0;
	msg2.pose.pose.position.y = 0;
	msg2.pose.pose.orientation.z = 0;
	pub2.publish(msg2);*/

	//while (ros::ok()) { // since we only want to run this once, we don't need a while loop
	
	while (xcur <= 0.25){
		msg.linear.x = 0.1; //go forward
	  	msg.linear.y = 0;
	  	msg.linear.z = 0;
	  	msg.angular.x = 0;
	  	msg.angular.y = 0;
	  	msg.angular.z = 0;
	  	pub.publish(msg);
		rate.sleep();
		ros::spinOnce();
	}
	
	while (thetacur <= 0.98){
		msg.linear.x = 0; 
	  	msg.linear.y = 0;
	  	msg.linear.z = 0;
	  	msg.angular.x = 0;
	  	msg.angular.y = 0;
	  	msg.angular.z = 1;	//spin 180 degrees
	  	pub.publish(msg);
		rate.sleep();
		ros::spinOnce();
	}
	
	//i = 0; // reset position to (0,0,0)

	while (xcur <= 0.70 - 0.25){
		msg.linear.x = -0.1; // controls speed
	  	msg.linear.y = 0;
	  	msg.linear.z = 0;
	  	msg.angular.x = 0;
	  	msg.angular.y = 0;
	  	msg.angular.z = 0;
	  	pub.publish(msg);
		rate.sleep();
		ros::spinOnce();
	}
	
	msg.linear.x = 0; // controls speed
  	msg.linear.y = 0;
  	msg.linear.z = 0;
  	msg.angular.x = 0;
  	msg.angular.y = 0;
  	msg.angular.z = 0;
  	pub.publish(msg);
	rate.sleep();

	//break;

	/* if (xcur >= 0.25) { // controls distance
	//if (xcur >= 1 - getBrakingDistance()) { // controls distance
	
		msg.linear.x = 0;
  		msg.linear.y = 0;
  		msg.linear.z = 0;
  		msg.angular.x = 0;
 	 	msg.angular.y = 0;
 	 	msg.angular.z = 0;
 	 	pub.publish(msg);
		rate.sleep();
		
		break;
	} */

	//}


	return 0;

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


void getOdometerData(const nav_msgs::Odometry::ConstPtr& msg) { // to reset to (0,0,0), just set i to 0

	if (i == 0){ // run this first, gets init values for correction
		xcurinit = msg->pose.pose.position.x;
		ycurinit = msg->pose.pose.position.y;
		thetacurinit = msg->pose.pose.orientation.z;

		velocity = 0; // reset velocity

		ROS_INFO("Init values recieved: x = %f, y = %f, theta = %f", xcurinit, ycurinit, thetacurinit);

	}
	else {

		if (i != 1) { 
			// run this second and so on. Sets LOCAL position aka starting from (0,0,0). NOT GLOBAL position.
			xcurLast = xcur;
			ycurLast = ycur;
			thetacurLast = thetacur;
		}

		xcur = msg->pose.pose.position.x; 
		/*if (xcur < 0)
			xcur *= -1;*/

		ycur = msg->pose.pose.position.y;
		/*if (ycur < 0)
			ycur *= -1;*/

		thetacur = msg->pose.pose.orientation.z; // find the new position

		xcur -= xcurinit; 
		ycur -= ycurinit;
		thetacur -= thetacurinit; // apply (0,0,0) correction

		// calculate velocity = distance / time
		// distance formula
		distance = (double)pow((double)pow(xcur - xcurLast, 2) + (double)pow(ycur - ycurLast , 2) , 0.5); 
		// pow(base, exponent)

		velocity = (double)distance * 10;



		ROS_INFO("x = %f, y = %f, theta = %f, velocity = %f", xcur, ycur, thetacur, velocity);


	}

	i++;
}

