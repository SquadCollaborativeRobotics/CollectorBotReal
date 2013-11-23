#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float32.h"

// State variables
double x = 0.0;
double y = 0.0;
double th = 0.0;

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

double lws = 0; // Left wheel speed (angular velocity)
double rws = 0; // Right wheel speed (angular velocity)

#define PI 3.1415926
#define WHEEL_DIAMETER 0.123825 //meters
#define WHEEL_RADIUS WHEEL_DIAMETER/2.0 //meters
#define WHEEL_SEPARATION 0.4318 // meters, Distance between two wheels along axis of rotation.

// Last time received callback for left or right wheel.
ros::Time last_lw_time, last_rw_time;

bool hasNewSpeeds = true;

// Update current left wheel speed
void lw_speed_callback(const std_msgs::Float32::ConstPtr& msg)
{
  last_lw_time = ros::Time::now();
  //ROS_INFO("Left wheel speed: [%lf]", msg->data);
  lws = msg->data;
}

// Update current right wheel speed
void rw_speed_callback(const std_msgs::Float32::ConstPtr& msg)
{
  last_rw_time = ros::Time::now();
  //ROS_INFO("Right wheel speed: [%lf]", msg->data);
  rws = -msg->data;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;

  // Subscribers
  ros::Subscriber lw_sub = n.subscribe("lw_speed", 10, lw_speed_callback);
  ros::Subscriber rw_sub = n.subscribe("rw_speed", 10, rw_speed_callback);

  // Publishers
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);

  // Transform Broadcasters
  tf::TransformBroadcaster odom_broadcaster;
  tf::TransformBroadcaster camera_link_broadcaster;

  // Initialize variables for loop
  ros::Rate r(50.0);
  ros::Time current_time = ros::Time::now();
  ros::Time last_time = ros::Time::now();

  while(n.ok()){
    // Set current time
    current_time = ros::Time::now();

    // Broadcast the base link to kinect
    camera_link_broadcaster.sendTransform(
        tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0.0, 1),
                      tf::Vector3(0.0, 0.0, 0.0762)),
        ros::Time::now(),"base_link", "camera_link"));


    // Only update values if the last updated wheel speeds were within the last second, otherwise zero them out
    if ((current_time - last_lw_time < ros::Duration(1.0)) && (current_time - last_rw_time < ros::Duration(1.0)))
    {
      //compute odometry in a typical way given the velocities of the robot
      double dt = (current_time - last_time).toSec();

      // Configuration Transition Equation - http://planning.cs.uiuc.edu/node659.html
      vx  = (WHEEL_RADIUS / 2.0) * (lws + rws) * cos(th);
      vy  = (WHEEL_RADIUS / 2.0) * (lws + rws) * sin(th);
      vth = (WHEEL_RADIUS / WHEEL_SEPARATION) * (lws - rws);

      //ROS_INFO("vx = %lf, vy = %lf, vth = %lf", vx, vy, vth);
      double delta_x  = vx * dt;
      double delta_y  = vy * dt;
      double delta_th = vth * dt;

      x  += delta_x;
      y  += delta_y;
      th += delta_th;
    }
    else
    {
      ROS_INFO("Last updated wheel speeds not within 1 second, zeroing out velocities.");
      vx = vy = vth = 0;
    }

    // Since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    // First, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    // Send the transform
    odom_broadcaster.sendTransform(odom_trans);

    // Next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    // Set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    // Set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    // Publish the message
    odom_pub.publish(odom);

    // Update last time to now and rate limit loop speed
    last_time = current_time;

    // Update callbacks after the fact, for next loop iteration.
    ros::spinOnce();
    r.sleep();
  }
}
