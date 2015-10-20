/*
 * main.cpp
 *
 *  Created on: 09.12.2011
 *      Author: indorewala@servicerobotics.eu
 */
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

double current_time;
double last_time;
double delta_time;
double init_time;
double x=0;
double y=0;
double th=0;
float velocity_x;
float velocity_y;
float angular_z;

void Callback(const geometry_msgs::TwistConstPtr& msg)
{  
	current_time = ros::Time::now().toSec();
	delta_time=current_time-last_time;
	velocity_x=msg->linear.x;
	velocity_y=msg->linear.y;
	angular_z=msg->angular.z;
 	double delta_x = ((msg->linear.x) * cos(th) - (msg->linear.y) * sin(th)) * delta_time;
	double delta_y = ((msg->linear.x) * sin(th) + (msg->linear.y) * cos(th)) * delta_time;
	double delta_th = msg->angular.z * delta_time;
     	x += delta_x;
    	y += delta_y;
    	th += delta_th;


	last_time = current_time;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "spacenav_publisher");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("cmd_vel_feedback", 1000, &Callback);
  ros::Publisher odom_pub2 = n.advertise<nav_msgs::Odometry>("odom", 1000);
  ros::Rate rate(20);
  tf::TransformBroadcaster odom_broadcaster;
  init_time = ros::Time::now().toSec();
  ros::Time current_time2;
   while(ros::ok())
   {
    current_time2 = ros::Time::now();
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time2;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation=tf::createQuaternionMsgFromRollPitchYaw(0,0,th);

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time2;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,th);

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = velocity_x;
    odom.twist.twist.linear.y = velocity_y;
    odom.twist.twist.angular.z = angular_z;

    //publish the message  (misschien staan x en y nog omgewisseld)
    odom_pub2.publish(odom);


   ros::spinOnce();
   rate.sleep();
   }
  return 0;

}

