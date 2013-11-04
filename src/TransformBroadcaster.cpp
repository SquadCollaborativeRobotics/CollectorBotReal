#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  tf::TransformBroadcaster broadcaster;

  ros::Rate r(10);
  while(n.ok()){

    broadcaster.sendTransform(
        tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0.0, 1),
                      tf::Vector3(0.0, 0.0, 0.0762)),
        ros::Time::now(),"base_link", "camera_link"));
    ROS_INFO("Broadcasted base_link to camera_link");

    ros::spinOnce();
    r.sleep();
  }
}
