#include "ros/ros.h"                      
#include "geometry_msgs/Twist.h"                                 
#include "nav_msgs/Odometry.h" 
#include <cmath>

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

int x_target, y_target;
double dest_theta;
double dest_distance;

int i=0;

double getBrakingDistance();

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);


int main(int argc, char** argv) {

	y_target = 0;
	x_target = -1;

	dest_theta = atan2(y_target, x_target); //explicitly for angle in quadrant 1
	dest_theta /= 3.14159; //to make it go from -1 to 1
	
	dest_distance = sqrt(pow(x_target, 2) + pow(y_target, 2));

	ROS_INFO("Init dest_theta = %f, dest_distance = %f", dest_theta, dest_distance);

	ros::init(argc, argv, "master");
	ros::NodeHandle n; 
	ros::Subscriber sub = n.subscribe<nav_msgs::Odometry>("/RosAria/pose", 1, poseCallback);
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 100);
	ros::Rate rate(10); // times per second to check our position
	ros::spinOnce();




	int j = 0;
	geometry_msgs::Twist msg;

	// atan2: -1 to 1
	// robot: 0 to 1

	// assume we only run atan2 once.

	if (dest_theta < 0) { // then we go left
		dest_theta *= -1; // make it positive (assume thetacur is 0)
		while (dest_theta - thetacur >= 0.0) {
			msg.linear.x = 0; 
	  		msg.linear.y = 0;
	  		msg.linear.z = 0;
	  		msg.angular.x = 0;
	  		msg.angular.y = 0;
			msg.angular.z = 0.5;	//spin until target theta
			pub.publish(msg);
			rate.sleep();
			ros::spinOnce();
		}

		msg.linear.x = 0; //stop
	  	msg.linear.y = 0;
	  	msg.linear.z = 0;
	  	msg.angular.x = 0;
	  	msg.angular.y = 0;
	  	msg.angular.z = 0;
	  	pub.publish(msg);
	  	rate.sleep();
	  	ros::spinOnce();

	}
	else {  //we go right
		while (dest_theta + thetacur >= 0.0) {
			msg.linear.x = 0; 
	  		msg.linear.y = 0;
	  		msg.linear.z = 0;
	  		msg.angular.x = 0;
	  		msg.angular.y = 0;
			msg.angular.z = -0.5;	//spin until target theta
			pub.publish(msg);
			rate.sleep();
			ros::spinOnce();
		}
		
		msg.linear.x = 0; //stop
	  	msg.linear.y = 0;
	  	msg.linear.z = 0;
	  	msg.angular.x = 0;
	  	msg.angular.y = 0;
	  	msg.angular.z = 0;
	  	pub.publish(msg);
	  	rate.sleep();
	  	ros::spinOnce();
	}

	while(dest_distance - sqrt(pow(xcur, 2) + pow(ycur, 2)) - getBrakingDistance() >= 0.0){
		msg.linear.x = 0.5; //go forward for the calculated distance
	  	msg.linear.y = 0;
	  	msg.linear.z = 0;
	  	msg.angular.x = 0;
	  	msg.angular.y = 0;
	  	msg.angular.z = 0;
	  	pub.publish(msg);
	  	rate.sleep();
	  	ros::spinOnce();
	}

	msg.linear.x = 0; //stop
	msg.linear.y = 0;
	msg.linear.z = 0;
	msg.angular.x = 0;
	msg.angular.y = 0;
	msg.angular.z = 0;
	pub.publish(msg);
	rate.sleep();
	ros::spinOnce();

return 0;
}



double getBrakingDistance() {
	double dist;
	
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
	  distance = (double)pow((double)pow(xcur - xcurLast, 2) + (double)pow(ycur - ycurLast , 2) , 0.5); // NOT FOR DISTANCE

	  velocity = (double)distance * 10;



	  ROS_INFO("x = %f, y = %f, theta = %f, velocity = %f", xcur, ycur, thetacur, velocity);
	  
	}

	i++;
}

//END OF CODE.
