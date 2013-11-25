#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>

// Convenience Typedef
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

geometry_msgs::PoseStamped goal_pose;
bool new_goal = false;


// Callback for new goals
// TODO : REMAP THE FKING PoseStamped to Move! - Sam
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  ROS_INFO("Goal Callback!");

  // Extract Values from message
  goal_pose = *msg;

  // Inform stack there's a new goal
  new_goal = true;

}



int main(int argc, char** argv){
  
  // ROS Node Initialization
  ros::init(argc, argv, "navigation_goals");
  ros::NodeHandle n;

  // Create Rate Object for sleeping
  ros::Rate r(10);

  ROS_INFO("Starting subscription.");
  // Subscribe to goal
  ros::Subscriber goal_sub = n.subscribe("move_base_simple/goal", 1000, goalCallback);

  // Tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  while(ros::ok()){

    if(new_goal){
      move_base_msgs::MoveBaseGoal goal;

      // Construct goal message
      goal.target_pose = goal_pose;
      // goal.target_pose.header.frame_id = "base_link";
      // goal.target_pose.header.stamp = ros::Time::now();

      // goal.target_pose.pose.position.x = goal_x;
      // goal.target_pose.pose.position.y = goal_y;
      // goal.target_pose.pose.orientation.z = goal_theta;
      // goal.target_pose.pose.orientation.w = 1.0;

      ROS_INFO("Sending goal");
      ac.sendGoal(goal);

      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved 1 meter forward");
      else
        ROS_INFO("The base failed to move forward 1 meter for some reason");
    }

    // Update callbacks after the fact, for next loop iteration.
    ros::spinOnce();

    r.sleep();

  }
}