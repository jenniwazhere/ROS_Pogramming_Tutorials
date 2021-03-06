#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>
using namespace std;

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;

//method to move the robot straight
void move(double speed, double distance, bool isForward);
//method to move rotate the robot
void rotate (double angular_speed, double angle, bool clockwise);
//declare how we want the angle in which the robot will move (Here: degrees, ex: 90)
double degrees2radians(double angle_in_degrees);
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
double setAbsoluteOrientation (double desired_angle);
//double clean ();
double getDistance(double x1, double y1, double x2, double y2);
//void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance);


int main(int argc, char **argv)
{
	//initiate new ROS node named "talker"
	ros::init(argc, argv, "robot_cleaner");
	ros::NodeHandle n;
	
	double speed, angular_speed;
	double distance, angle;
	bool isForward, clockwise;
	turtlesim::pose goal_pose;

	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",10);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);
	//to test the move function
	//ask the user for input
	/*cout << "Speed: "; 
	cin >> speed;
	cout << "Distance: "; 
	cin >> distance;
	cout << "Forward? "; 
	cin >> isForward;

	move(speed, distance, isForward);*/

	while(ros::ok()){
		
		/** test your code here **/
		
		/** run the clean application afer you implement it*/
		double t0 = ros::Time::now().toSec(); //get current time before cleaning
		//clean();
		double t1 = ros::Time::now().toSec(); //get current time after cleaning
		ROS_INFO ("Cleaning execution time: %.2f", (t1-t0));
		return 0;

	}

	ros::spin();

   return 0;

}

//more the bot a certain distance linearly
void move(double speed, double distance, bool isForward){
	geometry_msgs::Twist vel_msg;

	//move in the linear direction
	if (isForward)
		vel_msg.linear.x = abs(speed);
	else
		vel_msg.linear.x = -abs(speed);
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;

	//random y direction
	vel_msg.linear.x = 0;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;

	double t0 = ros::Time::now().toSec();
	double current_distance = 0;
	ros::Rate loop_rate(100);
	do {
		velocity_publisher.publish(vel_msg);
		double t1=ros::Time::now().toSec();
		current_distance = speed * (t1-t0);
		ros::spinOnce(); //need in order to publish
		loop_rate.sleep();
	} while(current_distance < distance); //perform the loop multiple times

	//force the robot to stop!
	vel_msg.linear.x = 0; 
	velocity_publisher.publish(vel_msg);

}


/**
 *  makes the robot turn with a certain angular velocity, for 
 *  a certain distance in either clockwise or counter-clockwise direction  
 */
void rotate (double angular_speed, double relative_angle, bool clockwise){

	geometry_msgs::Twist vel_msg;
	   //set a random linear velocity in the x-axis
	   vel_msg.linear.x =0;
	   vel_msg.linear.y =0;
	   vel_msg.linear.z =0;
	   //set a random angular velocity in the y-axis
	   vel_msg.angular.x = 0;
	   vel_msg.angular.y = 0;
	   if (clockwise)
	   		   vel_msg.angular.z =-abs(angular_speed);
	   	   else
	   		   vel_msg.angular.z =abs(angular_speed);

	   double t0 = ros::Time::now().toSec();
	   double current_angle = 0.0;
	   ros::Rate loop_rate(1000);
	   do{
		   velocity_publisher.publish(vel_msg);
		   double t1 = ros::Time::now().toSec();
		   current_angle = angular_speed * (t1-t0);
		   ros::spinOnce();
		   loop_rate.sleep();
		   //cout<<(t1-t0)<<", "<<current_angle <<", "<<relative_angle<<endl;
	   }while(current_angle<relative_angle);
	   vel_msg.angular.z =0;
	   velocity_publisher.publish(vel_msg);
}

/**
 *  converts angles from degree to radians  
 */

double degrees2radians(double angle_in_degrees){
	return angle_in_degrees *PI /180.0;
}

/**
 *  callback function to update the pose of the robot  
 */

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message){
	turtlesim_pose.x=pose_message->x;
	turtlesim_pose.y=pose_message->y;
	turtlesim_pose.theta=pose_message->theta;
}

/**
 *  turns the robot to a desried absolute angle  
 */

double setAbsoluteOrientation (double desired_angle_radians){

	double relative_angle_radians = desired_angle_radians - turtlesim_pose.theta;
	bool clockwise = ((relative_angle_radians<0)?true:false);
	cout<<desired_angle_radians <<","<<turtlesim_pose.theta<<","<<relative_angle_radians<<","<<clockwise<<endl;
	rotate (abs(relative_angle_radians), abs(relative_angle_radians), clockwise);
}

/*
 * get the euclidian distance between two points 
 */
double getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

/*
 * TODO
 * a proportional controller to make the robot moves towards a goal pose
 * IMPLEMENT THIS FUNCTION
 */

void moveGoal(turtlesim::Pose  goal_pose, double distance_tolerance){

}

/*
 * TODO
 * the cleaning appication function. returns true when completed.
 * IMPLEMENT THIS FUNCTION
 */

double clean(){

	
	
	//return the total distance traveled by the robot
	//calculate the distance as the of segment lengths or sum of distance between visited points. 
}

