#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher pub;
ros::Subscriber sub;

// Callback for new april tags
void tagCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  ROS_INFO("April Tag Callback!");

  ROS_INFO("Header : %s", msg->header.frame_id.c_str());
  if (msg->header.frame_id.c_str()[0] == '6') {
    ROS_INFO("Header Match!");
    // Publish pose to move_base_simple/goal
    geometry_msgs::PoseStamped goal_pose;

    // Set goal position in front of april tag
    goal_pose.pose.position.x = msg->pose.position.x - 0.5*cos(msg->pose.orientation.z);
    goal_pose.pose.position.y = msg->pose.position.y - 0.5*sin(msg->pose.orientation.z);

    goal_pose.header.frame_id = "map";
    goal_pose.pose.orientation.z = msg->pose.orientation.z;
    goal_pose.pose.orientation.w = 1.0;

    // Just use the /map x & y
    pub.publish(goal_pose);

    // Unsubscribe so we only use goal once
    sub.shutdown();
  }
}

int main(int argc, char** argv){
  
  // ROS Node Initialization
  ros::init(argc, argv, "navigation_april_tags");
  ros::NodeHandle n;

  // Create Rate Object for sleeping
  ros::Rate r(1);

  ROS_INFO("Starting april tag subscription.");
  
  // Subscribe to goal
  sub = n.subscribe("apriltags", 1000, tagCallback);

  // Publisher to goal
  pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000);

  while(ros::ok()){
    
    // Update callbacks after the fact, for next loop iteration.
    ros::spinOnce();

    r.sleep();

  }
}