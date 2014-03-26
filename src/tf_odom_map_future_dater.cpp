/*
Future dates odom->map tf
*/
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_odom_map_future_dater");

  ros::NodeHandle node;
  tf::TransformListener listener;
  tf::TransformBroadcaster map_to_odom_broadcaster;

  tf::StampedTransform map_to_odom_transform;
  ros::Duration half_second(0.5);

  ros::Rate rate(10.0);
  while (node.ok()){
    try{
      listener.lookupTransform("/map", "/scanmatcher_frame",
                               ros::Time(0), map_to_odom_transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    // Publish future-dated transform
    tf::StampedTransform transform;
    transform.frame_id_ = "map";
    transform.child_frame_id_ = "base_link";
    transform.setOrigin( map_to_odom_transform.getOrigin() );
    transform.setRotation( map_to_odom_transform.getRotation() );
    transform.stamp_ = map_to_odom_transform.stamp_;
    map_to_odom_broadcaster.sendTransform(transform);

    rate.sleep();
  }

  return 0;
};