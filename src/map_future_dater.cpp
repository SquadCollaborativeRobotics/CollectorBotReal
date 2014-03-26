/*
Future dates map messages
*/
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>


ros::Time given_time;
ros::Publisher pub;
ros::Subscriber sub;
ros::Duration half_second(0.5);


void mapCallback(nav_msgs::OccupancyGrid msg)
{
  //geometry_msgs::PoseWithCovariance posemsg;

  // Push load time half a second into the future
  msg.info.map_load_time = msg.info.map_load_time + half_second;

  // nav_msgs::OccupancyGrid future_map;
  // msg.info.map_load_time = 0; // Test
  pub.publish(msg);
  // ROS_INFO("y position: [%f]", yval);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "map_future_dater");
  ros::NodeHandle n;
  sub = n.subscribe<nav_msgs::OccupancyGrid>("map2", 10, &mapCallback);
  pub = n.advertise<nav_msgs::OccupancyGrid>("map", 50);

  ros::Rate r(10.0);
  ros::spin();
}
