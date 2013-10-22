#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "scr_proto/DiffCommand.h"

// Physical Parameters... What do to with these?
float wheel_radius, axle_length

// Global values for velocity storage
float cmd_vx=0, cmd_vy=0, cmd_vz=0, cmd_wx=0, cmd_wy=0, cmd_wz=0;

// Callback to attach to command velocity subscription
void commandCallback(const geometry_msgs::Twist::ConstPtr& msg){

  // Pull values from message
  cmd_vx = msg->linear->x;
  cmd_vy = msg->linear->y;
  cmd_vz = msg->linear->z;
  cmd_wx = msg->angular->x;
  cmd_wy = msg->angular->y;
  cmd_wz = msg->angular->z;
 
}

int main(int argc, char **argv){

  // Initialize Node
  ros::init(argc, argv, "base_controller");

  // Create Nodehandle
  ros::Nodehandle n;

  // Subscribe to Command Velocity topic
  ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 1000, commandCallback);

  ros::Publisher motor_pub = n.advertise<scr_proto::DiffCommand>("/motor_command", 1000);

  ros::Rate loop_rate(10);

  while(ros::ok()){
    
    // Calculate Motor Commands given command velocity
    // Need Linear Alg or something to solve aX=b
    float a[2][2] = {{1.0 1.0}, 
                   {1.0/(axle_length/2.0), -1.0/(axle_length/20)}}

    float b[2][1] = {{cmd_vx}, {cmd_wz}}
    
    // Pass to linear Alg to solver or something?

    // Declare Message
    scr_proto::DiffCommand motor_com;

    // Populate Message

    // Publish Message
    motor_pub.publish(motor_com);

    // Handle loop rate
    ros::spinOnce();
    loop_rate.sleep();
  }
}
  
