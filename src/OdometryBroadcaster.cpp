#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float64.h"

double x = 0.0;
double y = 0.0;
double th = 0.0;

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

double lws=0;
double rws=0;

void lw_speed_callback(const std_msgs::Float64::ConstPtr& msg)
{
  ROS_INFO("Left wheel speed: [%lf]", msg->data);
  lws = msg->data;
  // Theta velocity depends on the distance between the wheels
  vth = (lws-rws)/0.2159;
}

void rw_speed_callback(const std_msgs::Float64::ConstPtr& msg)
{
  ROS_INFO("Right wheel speed: [%lf]", msg->data);
  rws = msg->data;
  // Theta velocity depends on the distance between the wheels
  vth = (lws-rws)/0.2159;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;

  ros::Subscriber lw_sub = n.subscribe("lw_speed", 1000, lw_speed_callback);
  ros::Subscriber rw_sub = n.subscribe("rw_speed", 1000, rw_speed_callback);

  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(100.0);
  while(n.ok()){
    ROS_INFO("Hey, i'm broadcastin, like a Boss");
    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();

    vx = (lws+rws)/2 * cos(th);
    vy = (lws+rws)/2 * sin(th);
    ROS_INFO("vx = %lf, vy = %lf, vth = %lf", vx, vy, vth);
    double delta_x = vx * dt;
    double delta_y = vy * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}