#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float32.h"

double x = 0.0;
double y = 0.0;
double th = 0.0;

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

double lws=0;
double rws=0;

#define PI 3.1415926
#define WHEEL_DIAMETER 0.123825 //meters
#define WHEEL_SEPARATION 0.2159

ros::Time last_lw_time, last_rw_time;

bool hasNewSpeeds = true;

void lw_speed_callback(const std_msgs::Float32::ConstPtr& msg)
{
  last_lw_time = ros::Time::now();
  //ROS_INFO("Left wheel speed: [%lf]", msg->data);
  lws = msg->data * WHEEL_DIAMETER / 2;
  // Theta velocity depends on the distance between the wheels
  vth = (lws-rws)/WHEEL_SEPARATION;
}

void rw_speed_callback(const std_msgs::Float32::ConstPtr& msg)
{
  last_rw_time = ros::Time::now();
  //ROS_INFO("Right wheel speed: [%lf]", msg->data);
  rws = -msg->data * WHEEL_DIAMETER / 2;
  // Theta velocity depends on the distance between the wheels
  vth = (lws-rws)/WHEEL_SEPARATION;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;

  ros::Subscriber lw_sub = n.subscribe("lw_speed", 10, lw_speed_callback);
  ros::Subscriber rw_sub = n.subscribe("rw_speed", 10, rw_speed_callback);



  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);


  tf::TransformBroadcaster odom_broadcaster;

  tf::TransformBroadcaster camera_link_broadcaster;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(50.0);
  while(n.ok()){
    //ROS_INFO("Hey, i'm broadcastin, like a Boss");
    ros::spinOnce();               // check for incoming messages

    current_time = ros::Time::now();

    //Broadcast the base link to kinect
    camera_link_broadcaster.sendTransform(
        tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0.0, 1),
                      tf::Vector3(0.0, 0.0, 0.0762)),
        ros::Time::now(),"base_link", "camera_link"));


    if ((current_time - last_lw_time < ros::Duration(1.0)) && (current_time - last_lw_time < ros::Duration(1.0)))
    {
      //compute odometry in a typical way given the velocities of the robot
      double dt = (current_time - last_time).toSec();

      vx = (lws+rws)/2 * cos(th);
      vy = (lws+rws)/2 * sin(th);
      //ROS_INFO("vx = %lf, vy = %lf, vth = %lf", vx, vy, vth);
      double delta_x = vx * dt;
      double delta_y = vy * dt;
      double delta_th = vth * dt;

      x += delta_x;
      y += delta_y;
      th += delta_th;
    }
    else
    {
      vx = vy = vth = 0;
    }

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