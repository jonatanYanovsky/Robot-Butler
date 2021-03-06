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
double xsonarVal3 = 0;
double xsonarVal4 = 0;

double velocity;
double distance; // used for velocity calculation

double xHold = 0;

int i=0;

double getBrakingDistance();
void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);
void sonarCallback(const sensor_msgs::PointCloud::ConstPtr& msg2);

int main(int argc, char** argv) {

	ros::init(argc, argv, "master");
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
		if ((xsonarVal3 == 0) || (xsonarVal4 == 0) || (xsonarVal3 > 800 && xsonarVal4 > 800)){
				msg.linear.x = 0.2; // continue
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
				xHold = xcur;
			}
			while (xcur <= xHold + 0.7) {
				msg.linear.x = -0.1; // reverse
				msg.linear.y = 0;
				msg.linear.z = 0;
				msg.angular.x = 0;
				msg.angular.y = 0;
				msg.angular.z = 0;
				pub.publish(msg);
				rate.sleep();
				ros::spinOnce();
			}

				msg.linear.x = 0; // stop
				msg.linear.y = 0;
				msg.linear.z = 0;
				msg.angular.x = 0;
				msg.angular.y = 0;
				msg.angular.z = 0;
				pub.publish(msg);
				rate.sleep();
				ros::spinOnce();
				break;
		}//else end
	}//while ros::ok end
return 0;

}//end int main



double getBrakingDistance() {
	double dist;
		
	if (velocity <= 0.75)
		dist = 1.7463*velocity*velocity + 0.1283*velocity + 0.0212;
	else
		dist = 1.15;
		//dist = 0.1257*velocity + 1.0133;


	return dist;
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
	
		xsonarVal3 = 1000*msg2->points[3].x;
		xsonarVal4 = 1000*msg2->points[4].x;

}
