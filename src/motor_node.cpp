#include "ros/ros.h"
#include "scr_proto/DiffCommand.h"
#include "serial_interface.h"

// Global Array for holding motor commands
int left_motor_command = 0;
int right_motor_command = 0;

// Callback function for Motor Command Topic
void motorCallback(const scr_proto::DiffCommand::ConstPtr& msg){
  left_motor_command = msg->left_motor;
  right_motor_command = msg->right_motor;
}

int main(int argc, char **argv){

  // Initialize Node
  ros::init(argc, argv, "motor_driver");

  // Create a Node Handle
  ros::NodeHandle n;

  // Subscribe to Motor Command topic
  ros::Subscriber sub = n.subscribe("motor_command", 1000, motorCallback);

  // Spin to allow callback to do all the work
  ros::spin();

  return 0;

}
