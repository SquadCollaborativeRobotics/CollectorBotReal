#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/thread/mutex.hpp>
#include "std_msgs/Int32.h"

// The global planner uses the actionlib to do the following:
// Start in safe mode
//
// In safe state
//  cancel all goals and no april tag callbacks
//  if start command:
//   Transition to search state
//  if goto state command:
//   Transition to given state
// 
// In search state
//  travel to poses A,B,etc... on known map
//  If a trashcan is found at any time:
//   Transition to approach_trash state
//  else if hit final search state:
//   Transition to end state
//  
// In approach_trash state
//  travel to infront of trashcan
//  if reach trash pose:
//   transition to dump trash state
//
// In dump trash state
//  travel to dump trash position
//  if reach dump trash position:
//   transition to end state
//
// In end state
//  travel to final pose
//  if reaach final pose:
//   transition to safe state


// Convenience Typedef
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

boost::shared_ptr<MoveBaseClient> action_client_ptr;

// April tag subscriber
ros::Subscriber sub;

// State Machine for Fall Demo #2
enum State { SAFE, SEARCH_A, SEARCH_B, SEARCH_C, SEARCH_D, SEARCH_E, APPROACH_TRASH, DUMP_TRASH, END};
State currState = SAFE;

// Search poses
struct search_pose
{
  // x, y, rot
  double x, y, rz, rw;
};
search_pose search_poses[] = { {1, -1, 0, 1},
                               {1.6, 1, 1, 0},
                               {-2, .85, -.73, .68},
                               {-2, -1, 0, 1},
                               {0, -1, 0.67, 0.72}
                             };
// End position
search_pose end_pose = {0, 0, 0, 1};

// Sets given goal to given x,y and rotation quat rz,rw
void setGoalPoseRaw(double x, double y, double rz, double rw, 
                 move_base_msgs::MoveBaseGoal &goal) {
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.z = rz;
  goal.target_pose.pose.orientation.w = rw;  
}
void setGoalPose(const search_pose &s, 
                 move_base_msgs::MoveBaseGoal &goal) {
  setGoalPoseRaw(s.x, s.y, s.rz, s.rw, goal);
}

// Given a trashcan as a PoseStamped messge, returns the goal pose in front of it
void getGoalPoseFromTrashcan(const geometry_msgs::PoseStamped::ConstPtr& msg,
                             move_base_msgs::MoveBaseGoal &goal) {
  setGoalPoseRaw(msg->pose.position.x - 0.5*cos(msg->pose.orientation.z),
                 msg->pose.position.y - 0.5*sin(msg->pose.orientation.z),
                 msg->pose.orientation.z,
                 msg->pose.orientation.w,
                 goal);
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
}

bool found_trashcan = false;

// Callback for new april tags
void trashcanTagSearcherCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  // ROS_INFO("April Tag Callback!");

  // ROS_INFO("Header : %s", msg->header.frame_id.c_str());

  // If april tag is id 6 (trashcan)
  if (msg->header.frame_id.c_str()[0] == '6') {
    ROS_INFO("Header Match!");
    geometry_msgs::PoseStamped goal_pose;
    found_trashcan = true;

    move_base_msgs::MoveBaseGoal goal;
    
    if (currState == APPROACH_TRASH) {
      // Set goal position to in front of april tag
      getGoalPoseFromTrashcan(msg, goal);

      // http://mirror.umd.edu/roswiki/doc/diamondback/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html#a6bdebdd9f43a470ecd361d2c8b743188
      // If a previous goal is already active when this is called. 
      // We simply forget about that goal and start tracking the new goal. 
      // No cancel requests are made.
      action_client_ptr->sendGoal(goal);
    }
  }
}

void transition(State state, ros::NodeHandle &n) {
  if (currState != state) {
    currState = state;
    ROS_INFO("-- STATE TRANSITION %d --", currState);

    int chosen_search_pose = -1;
    switch(currState) {
      case SAFE:
      action_client_ptr->cancelAllGoals(); // Cancel any current move goals
      sub.shutdown();
      break;

      case SEARCH_A:
      chosen_search_pose = 0;
      ROS_INFO("SENDING POSE SEARCH_A to /move_base/goal");
      break;

      case SEARCH_B:
      chosen_search_pose = 1;
      ROS_INFO("SENDING POSE SEARCH_B to /move_base/goal");
      break;

      case SEARCH_C:
      chosen_search_pose = 2;
      ROS_INFO("SENDING POSE SEARCH_C to /move_base/goal");
      break;

      case SEARCH_D:
      chosen_search_pose = 3;
      ROS_INFO("SENDING POSE SEARCH_D to /move_base/goal");
      break;

      case SEARCH_E:
      chosen_search_pose = 4;
      ROS_INFO("SENDING POSE SEARCH_E to /move_base/goal");
      break;

      case APPROACH_TRASH:
      break;

      case DUMP_TRASH:
      sub.shutdown();
      {
        // Set goal to search pose
        move_base_msgs::MoveBaseGoal goal;
        setGoalPose(end_pose, goal);
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        
        // Send goal to action client
        action_client_ptr->sendGoal(goal);
        ROS_INFO("END GOAL SENT");
      }

      break;

      case END:
      sub.shutdown();
      break;
    }

    // If searching
    if (chosen_search_pose >= 0) {
      // Subscribe to trashcan searcher
      sub = n.subscribe("apriltags", 1000, trashcanTagSearcherCallback);
      
      // Set goal to search pose
      move_base_msgs::MoveBaseGoal goal;
      setGoalPose(search_poses[chosen_search_pose], goal);
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();
      
      // Send goal to action client
      action_client_ptr->sendGoal(goal);
      ROS_INFO("GOAL SENT");
    }
  }
}


int command_value = 0;
void commandCallback(const std_msgs::Int32::ConstPtr& msg) {
  command_value = msg->data;
  ROS_INFO("Received command %d", command_value);
}

int main(int argc, char** argv){
  
  // ROS Node Initialization
  ros::init(argc, argv, "global_planner");
  ros::NodeHandle n;

  // Subscribe to command node
  ros::Subscriber cmd_sub = n.subscribe("cmd_state", 10, commandCallback);

  action_client_ptr.reset( new MoveBaseClient("move_base", true) );

  // Create Rate Object for sleeping
  ros::Rate r(1);

  // Wait for the action server to come up
  while(ros::ok() && !action_client_ptr->waitForServer(ros::Duration(1.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  
  ROS_INFO("Starting global planner.");

  while(ros::ok()){
    if (command_value == 0) {
      transition(SAFE, n);
    }
    if (found_trashcan) {
      found_trashcan = false;
      transition(APPROACH_TRASH, n);
    }
    // State machine command override from safe to chosen state
    switch(currState) {
      case SAFE:
      switch (command_value) {
        case 1:
        transition(SEARCH_A, n);
        break;
        case 2:
        transition(SEARCH_B, n);
        break;
        case 3:
        transition(SEARCH_C, n);
        break;
        case 4:
        transition(SEARCH_D, n);
        break;
        case 5:
        transition(SEARCH_E, n);
        break;
        case 6:
        transition(APPROACH_TRASH, n);
        break;
        case 7:
        transition(DUMP_TRASH, n);
        break;
        case 8:
        transition(END, n);
        break;
      }
      break;

      case SEARCH_A:
      ROS_INFO("Status : %s", action_client_ptr->getState().toString().c_str());
      if (action_client_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED ||
          command_value == 2) {
        transition(SEARCH_B, n);
      }
      break;

      case SEARCH_B:
      ROS_INFO("Status : %s", action_client_ptr->getState().toString().c_str());
      if (action_client_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED ||
          command_value == 3) {
        transition(SEARCH_C, n);
      }
      break;

      case SEARCH_C:
      ROS_INFO("Status : %s", action_client_ptr->getState().toString().c_str());
      if (action_client_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED ||
          command_value == 4) {
        transition(SEARCH_D, n);
      }
      break;

      case SEARCH_D:
      ROS_INFO("Status : %s", action_client_ptr->getState().toString().c_str());
      if (action_client_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED ||
          command_value == 5) {
        transition(SEARCH_E, n);
      }
      break;

      case SEARCH_E:
      ROS_INFO("Status : %s", action_client_ptr->getState().toString().c_str());
      if (action_client_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED ||
          command_value == 6) {
        transition(DUMP_TRASH, n);
      }
      break;

      case APPROACH_TRASH:
      if (action_client_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED ||
          command_value == 7) {
        transition(DUMP_TRASH, n);
      }
      break;

      // Dump trash means go to the end search_pos 6
      case DUMP_TRASH:
      if (action_client_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED ||
          command_value == 8) {
        transition(END, n);
      }
      break;

      case END:
      break;
    }

    if (action_client_ptr->getState() == actionlib::SimpleClientGoalState::ABORTED) {
      transition(SAFE, n);
    }
    
    // Update callbacks after the fact, for next loop iteration.
    ros::spinOnce();

    r.sleep();

  }
}