#include "ros/ros.h"
#include "scr_proto/DiffCommand.h"
#include "serial_interface.h"
#include <string>
#include <cmath>

// Global Array for holding motor commands
int left_motor_command = 0;
int right_motor_command = 0;

// Serial Port Name (Default for USB -> Serial module on Ubuntu)
char port_name[] = "/dev/ttyUSB0";

// Serial Pololu Object
PololuQik pmd = PololuQik(port_name);

// Callback function for Motor Command Topic
void motorCallback(const scr_proto::DiffCommand::ConstPtr& msg){
  // Pull Commands from message
  left_motor_command = msg->left_motor;
  right_motor_command = msg->right_motor;
  // Check that motor commands are within range
  if(abs(left_motor_command) > 255){
    ROS_INFO("Bad Left motor command, got [%d]", 
             left_motor_command);
    return;
  }
  if(abs(right_motor_command) > 255){
    ROS_INFO("Bad Right motor command, got [%d]",
             right_motor_command);
    return;
  }
  ROS_INFO("Left Motor: [%d]", left_motor_command);
  ROS_INFO("Right Motor: [%d]", right_motor_command);
  // Hardware call to drivers to change motor speed
  pmd.setSpeeds(left_motor_command, right_motor_command);
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
