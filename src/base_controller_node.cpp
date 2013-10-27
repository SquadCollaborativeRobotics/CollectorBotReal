#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "scr_proto/DiffCommand.h"
#include <eigen3/Eigen/Dense>

// Physical Parameters... What do to with these?
double wheel_radius=4.875/2.0*.0254, axle_half_length=8.4*.0254;
// Maximum loaded motor rotation speed in radians per sec
double motor_max_speed=25;

// Global values for velocity storage
double cmd_vx=0, cmd_vy=0, cmd_vz=0, cmd_wx=0, cmd_wy=0, cmd_wz=0;

// Callback to attach to command velocity subscription
void commandCallback(const geometry_msgs::Twist::ConstPtr& msg){

  // Pull values from message
  cmd_vx = msg->linear.x;
  cmd_wz = msg->angular.z;
 
}

// Function to calculate wheel speeds given a diff drive command
// Models problem as Ax = b form
Eigen::Vector2d getWheelSpeeds(double wheel_offset, double cmd_x, double cmd_th){
  /*
  @ Params:  wheel_offset -> Distance between center of wheel axle and center of wheel contact
             cmd_x        -> Commanded forward velocity for the robot
             cmd_th       -> Commanded angular velocity about the reference point
  @ Returns: x            -> Eigen::Vector containing body velocities at wheel;
                             x(0) -> Right Wheel
                             x(1) -> Left Wheel
  */

  // Diff Drive Dynamics Solution, vr = right wheel velocity, vl = left wheel velocity
  double vr = cmd_x + cmd_th * wheel_offset/2.0;
  double vl = cmd_x - cmd_th * wheel_offset/2.0;

  // Vector for result and return
  Eigen::Vector2d x;
  // Pack Vector with values and return
  x(0) = vr;
  x(1) = vl;
  return x;

}

int main(int argc, char **argv){

  // Initialize Node
  ros::init(argc, argv, "base_controller");

  // Create Nodehandle
  ros::NodeHandle n;

  // Subscribe to Command Velocity topic
  ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 1000, commandCallback);

  ros::Publisher motor_pub = n.advertise<scr_proto::DiffCommand>("/motor_command", 1000);

  ros::Rate loop_rate(10);

  while(ros::ok()){
    
    // Vector for holding the linear speeds at the wheels
    Eigen::Vector2d wheel_linear_speeds;
    
    // Pass to linear Alg to solver or something?
    wheel_linear_speeds = getWheelSpeeds(axle_half_length, cmd_vx, cmd_wz);

    // Convert Wheel Velocities to Angular Velocities
    // Be careful for sign flip, as motors are not mounted symmetrically
    double right_wheel_omega = wheel_linear_speeds(0)/wheel_radius;
    double left_wheel_omega = wheel_linear_speeds(1)/wheel_radius;

    // Log info to see output of linear system
    ROS_INFO("Right Motor Desired Speed = [%d]", right_wheel_omega);
    ROS_INFO("Left Motor Desired Speed = [%d]", left_wheel_omega);

    // Declare Message
    scr_proto::DiffCommand motor_com;

    // Populate Message
    // Need to map omega's to value between -127 and 127 for motor driver
    right_wheel_omega *= (127.0/motor_max_speed);
    left_wheel_omega *= (127.0/motor_max_speed);

    motor_com.left_motor = (int)left_wheel_omega;
    motor_com.right_motor = (int)right_wheel_omega;

    // Log Info for debugging
    ROS_INFO("Right Motor Command = [%d]", right_wheel_omega);
    ROS_INFO("Left Motor Command = [%d]", left_wheel_omega);

    // Publish Message
    motor_pub.publish(motor_com);

    // Handle loop rate
    ros::spinOnce();
    loop_rate.sleep();
  }
}
  
