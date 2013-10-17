/*
Pololu Motor Driver Code for Use with Arduino Microcontroller.
Adapted from https://github.com/pololu/qik-arduino, a pololu supplied library
Author: Alex Sher, 17 Oct 2013
*/

#include "pololu_qik_arduino.h"

// serial command buffer storage
byte cmd[5];

// Pololu Object, works for models 2s9v1 and 2s12v10
PololuQik::PololuQik(unsigned char receivePin, unsigned char transmitPin, unsigned char resetPin) : SoftwareSerial(receivePin, transmitPin)
{
  _resetPin = resetPin;
}

// Initialize System
void PololuQik::init(long speed /* = 9600 */)
{
  // reset the qik
  digitalWrite(_resetPin, LOW);
  // drive low
  pinMode(_resetPin, OUTPUT);
  delay(1); 
  // return to high-impedance input (reset is internally pulled up on qik)
  pinMode(_resetPin, INPUT); 
  delay(10);

  // Initalize Serial Connection (since this inherits serial, it has function begin)
  begin(speed);
  // Tells Qik to autodetect baud rate
  write(0xAA);
}

// Return Firmware Version
char PololuQik::getFirmwareVersion()
{
  listen();
  write(QIK_GET_FIRMWARE_VERSION);
  while (available() < 1);
  return read();
}

// Return Error Byte (definitions here: http://www.pololu.com/docs/0J29/5.c)
byte PololuQik::getErrors()
{
  listen();
  write(QIK_GET_ERROR_BYTE);
  while (available() < 1);
  return read();
}


// Return Configuration Parameters (see PololuQik.h for list of parameters)
byte PololuQik::getConfigurationParameter(byte parameter)
{
  listen();
  cmd[0] = QIK_GET_CONFIGURATION_PARAMETER;
  cmd[1] = parameter;
  write(cmd, 2);
  while (available() < 1);
  return read();
}

// Set Configuration parameters (see PololuQik.h for info on parameters)
byte PololuQik::setConfigurationParameter(byte parameter, byte value)
{
  listen();
  cmd[0] = QIK_SET_CONFIGURATION_PARAMETER;
  cmd[1] = parameter;
  cmd[2] = value;
  cmd[3] = 0x55;
  cmd[4] = 0x2A;
  write(cmd, 5);
  while (available() < 1);
  return read();
}

// Set Speed command
void PololuQik::setM0Speed(int speed)
{
  // Direction value
  boolean reverse = 0;

  // Handle negative direction speeds
  if (speed < 0)
  {
    // make speed a positive quantity
    speed = -speed;    
    // preserve the direction
    reverse = 1;  
  }

  // Clamp speed at Max readable speed
  if (speed > 255)
    speed = 255;

  if (speed > 127)
  {
    // 8-bit mode: actual speed is (speed + 128)
    cmd[0] = reverse ? QIK_MOTOR_M0_REVERSE_8_BIT : QIK_MOTOR_M0_FORWARD_8_BIT;
    cmd[1] = speed - 128;
  }
  else
  {
    // lower bit mode, can physically write speed in normal mode 
    cmd[0] = reverse ? QIK_MOTOR_M0_REVERSE : QIK_MOTOR_M0_FORWARD;
    cmd[1] = speed;
  }

  write(cmd, 2);
}

// Set Speed command for second channel, see above for explanations
void PololuQik::setM1Speed(int speed)
{
  boolean reverse = 0;

  if (speed < 0)
  {
    speed = -speed;
    reverse = 1;
  }

  if (speed > 255)
    speed = 255;

  if (speed > 127)
  {
    cmd[0] = reverse ? QIK_MOTOR_M1_REVERSE_8_BIT : QIK_MOTOR_M1_FORWARD_8_BIT;
    cmd[1] = speed - 128;
  }
  else
  {
    cmd[0] = reverse ? QIK_MOTOR_M1_REVERSE : QIK_MOTOR_M1_FORWARD;
    cmd[1] = speed;
  }

  write(cmd, 2);
}

// Set speeds on both channels
void PololuQik::setSpeeds(int m0Speed, int m1Speed)
{
  // Simply use commands written above
  setM0Speed(m0Speed);
  setM1Speed(m1Speed);
}

// 2s12v10 specific stuff

// Brake Command
void PololuQik2s12v10::setM0Brake(unsigned char brake)
{
  // Limit to 127, the brake limit
  if (brake > 127)
    brake = 127;
  
  // pack the command array
  cmd[0] = QIK_2S12V10_MOTOR_M0_BRAKE;
  cmd[1] = brake;

  // Serial write
  write(cmd, 2);
}

// Brake command for second channel, see above for explanations
void PololuQik2s12v10::setM1Brake(unsigned char brake)
{
  if (brake > 127)
    brake = 127;
  
  cmd[0] = QIK_2S12V10_MOTOR_M1_BRAKE;
  cmd[1] = brake;
  write(cmd, 2);
}

// Dual Channel Brake Command
void PololuQik2s12v10::setBrakes(unsigned char m0Brake, unsigned char m1Brake)
{
  // Utlizie pre=written Brake Functions
  setM0Brake(m0Brake);
  setM1Brake(m1Brake);
}

// Get Current
unsigned char PololuQik2s12v10::getM0Current()
{
  // Make sure device is still there
  listen();
  // Write command byte
  write(QIK_2S12V10_GET_MOTOR_M0_CURRENT);
  // Wait for response
  while (available() < 1);
  // Return response
  return read();
}

// Current command for second channel, see above for explanation
unsigned char PololuQik2s12v10::getM1Current()
{
  listen();
  write(QIK_2S12V10_GET_MOTOR_M1_CURRENT);
  while (available() < 1);
  return read();
}

// Current command for mA
unsigned int PololuQik2s12v10::getM0CurrentMilliamps()
{
  // Use regular current call and scale
  return getM0Current() * 150;
}

// Current command for second channel, see above for explanation
unsigned int PololuQik2s12v10::getM1CurrentMilliamps()
{
  return getM1Current() * 150;
}

// Get speed command
unsigned char PololuQik2s12v10::getM0Speed()
{
  // Make sure device is still there
  listen();
  // Write command byte
  write(QIK_2S12V10_GET_MOTOR_M0_SPEED);
  // Wait for a response
  while (available() < 1);
  // Return response
  return read();
}

// Speed command for second channel, see above for explanation
unsigned char PololuQik2s12v10::getM1Speed()
{
  listen();
  write(QIK_2S12V10_GET_MOTOR_M1_SPEED);
  while (available() < 1);
  return read();
}
