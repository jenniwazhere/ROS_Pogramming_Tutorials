#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
ros::Publisher velocity_publisher;

using namespace std;

//method to move the robot straight
void move(double speed, double distance, bool isForward);

int main(int argc, char **argv)
{
	//initiate new ROS node named "talker"
	ros::init(argc, argv, "robot_cleaner");
	ros::NodeHandle n;

	double speed, distance;
	bool isForward;

	velocity_publisher = n.adertise<geometry_msgs::Twist>("/turtle/cmd_vel",10);

	//to test the move function
	//ask the user for input
	cout << "Speed: "; cin >> speed;
	cout << "Distance: "; cin >> distance;
	cout << "Forward? "; cin >> isForward

	move (speed, distance, isForward);

}

//more the bot a certain distance linearly
void move(double speed, double distance, bool isForward){
	geometry_msgs::Twist vel_msg;



	//move in the linear direction
	if (isForward){
		vel_msg.linear.x = abs(speed);
	}
	else
		vel_msg.linear.x = - abs(speed);
	vel_msg.linear.x = 0;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;

	//random y direction
	vel_msg.linear.x = 0;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;

	//t0 = current time
	//loop 
	double t0 = ros:Time::now().toSec();
	double current_distance = 0;
	ros::Rate loop_rate(10);
	do {
		velocity_publisher.publish(vel_msg);
		doubtl t1=ros:Time::now().toSec();
		current_distance = speed * (t1-t0);
		ros::spinOnce(); //need in order to publish
		loop_rate.sleep();
	} while(current_distance < distance); //perform the loop multiple times

	//force the robot to stop!
	vel_msg.linear.x = 0; 
	velocity_publisher.publish(vel_msg);

}
