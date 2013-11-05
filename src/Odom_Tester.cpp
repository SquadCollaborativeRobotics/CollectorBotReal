#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv){
  
  geometry_msgs::Twist msg;

  // Initialize Node
  ros::init(argc, argv, "odom_test");

  // Get nodehandle
  ros::NodeHandle n;

  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  ros::Time start = ros::Time::now();

  msg.linear.x = .25;

  cmd_vel_pub.publish(msg);
  ROS_INFO("Publishing!");

  ros::Rate loop_rate(1);

  while((ros::Time::now() - start) < ros::Duration(10, 0)){
  	loop_rate.sleep();
	cmd_vel_pub.publish(msg);
	ROS_INFO("IM PUBLSIHING!!!");
  }

  msg.linear.x = 0;

  cmd_vel_pub.publish(msg);

  while((ros::Time::now() - start) < ros::Duration(15, 0)){
  	loop_rate.sleep();
	cmd_vel_pub.publish(msg);
	ROS_INFO("IM PUBLSIHING!!!");
  }

}
