// http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float32.h"

// Just broadcast the camera_link to base_link
int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;

  // Transform Broadcasters
  tf::TransformBroadcaster camera_link_broadcaster;

  // Initialize variables for loop
  ros::Rate r(30.0);

  while(n.ok()){
    // Broadcast the base link to kinect
    camera_link_broadcaster.sendTransform(
        tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0.0, 1),
                      tf::Vector3(0.0, 0.0, 0.0762)),
        ros::Time::now(),"base_link", "camera_link"));

    // Update callbacks after the fact, for next loop iteration.
    ros::spinOnce();
    r.sleep();
  }
}
