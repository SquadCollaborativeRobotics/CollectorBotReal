#include "ros/ros.h"
#include "scr_proto/DiffCommand.h"
#include "scr_proto/SpeedCommand.h"
#include "std_msgs/Int32.h"

double PI = 3.141592654;
double pulses_per_rev = 3200;

// CONTROL VARIABLES
double Kp = 1.5;
double Kd = 0.0;
double Ki = 0.0;

double last_control_time;
double lw_prev_error=0, rw_prev_error=0;
double lw_int_error=0, rw_int_error=0;
double int_error_max = 500.0;

// ROS Pub-Sub for node
ros::Publisher motor_cmd_pub;
ros::Subscriber left_enc_sub;
ros::Subscriber right_enc_sub;
ros::Subscriber motor_cmd_sub;

scr_proto::DiffCommand diff_msg;

// Store Last command time so we can shut down if not receiving direction
ros::Time last_command_time;

// Global Variables for left and right encoders
int left_enc_count, left_enc_prev_count, right_enc_count, right_enc_prev_count;
double left_prev_enc_time, right_prev_enc_time;

// Global Variables for Observed Wheel Velocities
double lw_omega, rw_omega;

// Global Variables for Commanded Wheel Speeds
double rw_spd_cmd, lw_spd_cmd;

// Callbacks to store data coming from Arduino
void leftEncoderCallback(const std_msgs::Int32::ConstPtr& msg){
  left_enc_count = msg->data;
} 
void rightEncoderCallback(const std_msgs::Int32::ConstPtr& msg){
  right_enc_count = msg->data;
}

// Gets current wheel velocity in rad/s
// ToDo : implement low pass filter on data?
void updateWheelVels(){
  
  // Get the current time
  double enc_time = ros::Time::now().toSec();
  
  // Calculate unfiltered encoder velocity in pulses per sec
  lw_omega = (left_enc_count - left_enc_prev_count) /
                 (enc_time - left_prev_enc_time);

  rw_omega = (right_enc_count - right_enc_prev_count) /
                 (enc_time - right_prev_enc_time);

  // Convert from pulses per sec to rads per sec
  lw_omega = lw_omega/(pulses_per_rev * 2.0 * 3.14159);
  rw_omega = rw_omega/(pulses_per_rev * 2.0 * 3.14159);

  // Update Previous Values
  left_enc_prev_count = left_enc_count;
  left_prev_enc_time = enc_time;
  right_enc_prev_count = right_enc_count;
  right_prev_enc_time = enc_time;
}

// Write values to Pololu motor Driver by publishing to motor_node
void assign(int rw_pmd_cmd, int lw_pmd_cmd){
  diff_msg.left_motor = lw_pmd_cmd;
  diff_msg.right_motor = rw_pmd_cmd;
  motor_cmd_pub.publish(diff_msg);
}

// Takes a command velocity in rad/s and uses PID to control
// motor to that speed
void control(double lw_cmd, double rw_cmd){

  // Get most current wheel velocities
  updateWheelVels();

  // Calculate Current Error
  double lw_cur_error = lw_omega - lw_cmd;
  double rw_cur_error = rw_omega - rw_cmd;

  // Get time difference between now and last control time
  double dt = ros::Time::now().toSec() - last_control_time;
  
  // Derivative of Error
  double lw_dedt = (lw_cur_error - lw_prev_error) / dt;
  double rw_dedt = (rw_cur_error - rw_prev_error) / dt;

  // Calculate PID Output
  int rw_output = Kp * rw_cur_error + Kd * rw_dedt + Ki * rw_int_error;
  int lw_output = Kp * lw_cur_error + Kd * lw_dedt + Ki * lw_int_error;

  // Write value to Motor Driver
  assign(rw_output, lw_output);

  // Update Error Values
  lw_int_error += (lw_cur_error + lw_prev_error) / 2.0 * dt;
  lw_int_error = std::min(lw_int_error, int_error_max);
  rw_int_error += (rw_cur_error + rw_prev_error) / 2.0 * dt;
  rw_int_error = std::min(rw_int_error, int_error_max);
  
  last_control_time = ros::Time::now().toSec();
  
  lw_prev_error = lw_cur_error;
  rw_prev_error = rw_cur_error;

}

// Callback to respond to new command velocity
void commandCallback(const scr_proto::SpeedCommand::ConstPtr& msg){
  lw_spd_cmd = msg->left_motor_w;
  rw_spd_cmd = msg->right_motor_w;
  last_command_time = ros::Time::now();
}

int main(int argc, char **argv){
  
  // Initialize Node
  ros::init(argc, argv, "speed_controller");

  // Get nodehandle
  ros::NodeHandle n;

  // Initializes Publisher/Subscriber
  motor_cmd_pub = n.advertise<scr_proto::DiffCommand>("/motor_command", 1000);
  motor_cmd_sub = n.subscribe("speed_command", 1000, commandCallback);
  left_enc_sub = n.subscribe("left_encoder", 1000, leftEncoderCallback);
  right_enc_sub = n.subscribe("right_encoder", 1000, rightEncoderCallback);

  last_command_time = ros::Time::now();

  // Loop Rate
  ros::Rate loop_rate(15);

  while(ros::ok()){

    if((ros::Time::now()-last_command_time) > ros::Duration(10, 0)){
      lw_spd_cmd = 0.0;
      rw_spd_cmd = 0.0;
      ROS_INFO("Stale Speed Command, shutting off motors");
      last_command_time = ros::Time::now();
    }

    else{
      control(lw_spd_cmd, rw_spd_cmd);
    }

    ros::spinOnce();


  }
}
