#include "ros/ros.h"
#include "std_msgs/Float64"
#include <algorithm>

// Constructor that stores desired gains in class variable
SpeedController::SpeedController(double kp, double ki, double kd){

  _Kp = kp;
  _Ki = ki;
  _Kd = kd;

}

// Gets current wheel velocity in rad/s
// ToDo : implement low pass filter on data?
private double SpeedController::get_wheel_vels(){
  
  // Get the current time
  enc_time = ros::Time::now().toSec();
  
  // Calculate unfiltered encoder velocity in pulses per sec
  omega = (enc_count - prev_enc_count)/(enc_time - prev_enc_time);

  // Convert from pulses per sec to rads per sec
  omega /= pulses_per_rev * 2 * PI;

  return omega;

}

// Write value to Pololu motor Driver
// Need to wait until ROS Serial architecture is figured out
private void SpeedController::assign(int val){

}

// Takes a command velocity in rad/s and uses PID to control
// motor to that speed
public double SpeedController::control(double w_cmd){


  // Get current wheel angular velocity
  double w_act = get_wheel_vels();
  // Calculate Current Error
  double cur_error = w_act - w_cmd;
  
  // Get time difference between now and last control time
  double dt = ros::Time.now().toSec() - last_control_time;
  
  // Derivative of Error
  double dedt = (cur_error - prev_error) / dt;

  // Calculate PID Output
  output += _Kp * cur_error + _Kd * dedt + _Ki * int_error;
  
  assign(output);

  // Update Error Values
  int_error += (cur_error + prev_error) / 2.0 * dt;
  int_error = min(int_error, int_error_max);
  last_control_time = ros::Time.now().toSec();

}


