#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "scr_proto/DiffCommand.h"
#include <eigen3/Eigen/Dense>

// Physical Parameters... What do to with these?
float wheel_radius=.25, axle_length=1;
float motor_max_speed=2.5;

// Global values for velocity storage
float cmd_vx=0, cmd_vy=0, cmd_vz=0, cmd_wx=0, cmd_wy=0, cmd_wz=0;

// Callback to attach to command velocity subscription
void commandCallback(const geometry_msgs::Twist::ConstPtr& msg){

  // Pull values from message
  cmd_vx = msg->linear.x;
  cmd_wz = msg->angular.z;
 
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
    
    // Calculate Motor Commands given command velocity
    // A matrix based on body parameters
    Eigen::Matrix2f A;
    A << 1.0, 1.0,
         1.0/(axle_length/2.0), -1.0/(axle_length/2.0);
    
    // b vector for command velocities
    Eigen::Vector2f b;
    b << cmd_vx,
         cmd_wz;
    
    // x vector for storing wheel velocities after solve 
    Eigen::Vector2f x;
    
    // Pass to linear Alg to solver or something?
    x = A.ldlt().solve(b);

    // Convert Wheel Velocities to Angular Velocities
    // Be careful for sign flip, as motors are not mounted symmetrically
    float right_wheel_omega = x(0)/wheel_radius;
    float left_wheel_omega = x(1)/wheel_radius;

    // Log info to see output of linear system
    ROS_INFO("Right Motor Desired Speed = [%f]", right_wheel_omega);
    ROS_INFO("Left Motor Desired Speed = [%f]", left_wheel_omega);

    // Declare Message
    scr_proto::DiffCommand motor_com;

    // Populate Message
    // Need to map omega's to value between -255 and 255.
    right_wheel_omega *= (255.0/motor_max_speed);
    left_wheel_omega *= (255.0/motor_max_speed);

    motor_com.left_motor = (int)left_wheel_omega;
    motor_com.right_motor = (int)right_wheel_omega;

    // Log Info for debugging
    ROS_INFO("Right Motor Command = [%f]", right_wheel_omega);
    ROS_INFO("Left Motor Command = [%f]", left_wheel_omega);

    // Publish Message
    motor_pub.publish(motor_com);

    // Handle loop rate
    ros::spinOnce();
    loop_rate.sleep();
  }
}
  
