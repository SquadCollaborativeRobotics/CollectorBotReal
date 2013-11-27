#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define PI 3.1415926

#define WHEEL_DIAMETER 0.123825 //meters
#define WHEEL_RADIUS WHEEL_DIAMETER/2.0 //meters
#define WHEEL_SEPARATION (0.445-(0.01*2)) // meters
// Distance between two wheels along axis of rotation.
// Wheel separation is the distance between the centers of the 2 drive wheels
// the numbers here are: 0.445 => distance between the far edge of each wheel
//                   0.01 * 2 => 2 cm wheels, so the distance is 0.01*2


int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;

  tf::TransformListener listener(n);
  
  // Publishers

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
