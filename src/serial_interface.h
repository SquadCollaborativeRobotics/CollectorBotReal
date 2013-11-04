#ifndef serial_interface_h 
#define serial_interface_h

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <SerialStream.h>

// Commands
#define QIK_GET_FIRMWARE_VERSION         0x81
#define QIK_GET_ERROR_BYTE               0x82
#define QIK_GET_CONFIGURATION_PARAMETER  0x83
#define QIK_SET_CONFIGURATION_PARAMETER  0x84

#define QIK_MOTOR_M0_FORWARD             0x88
#define QIK_MOTOR_M0_FORWARD_8_BIT       0x89
#define QIK_MOTOR_M0_REVERSE             0x8A
#define QIK_MOTOR_M0_REVERSE_8_BIT       0x8B
#define QIK_MOTOR_M1_FORWARD             0x8C
#define QIK_MOTOR_M1_FORWARD_8_BIT       0x8D
#define QIK_MOTOR_M1_REVERSE             0x8E
#define QIK_MOTOR_M1_REVERSE_8_BIT       0x8F

// 2s12v10 only
#define QIK_2S12V10_MOTOR_M0_BRAKE       0x86
#define QIK_2S12V10_MOTOR_M1_BRAKE       0x87
#define QIK_2S12V10_GET_MOTOR_M0_CURRENT 0x90
#define QIK_2S12V10_GET_MOTOR_M1_CURRENT 0x91
#define QIK_2S12V10_GET_MOTOR_M0_SPEED   0x92
#define QIK_2S12V10_GET_MOTOR_M1_SPEED   0x93

// Configuration parameters
#define QIK_CONFIG_DEVICE_ID                        0
#define QIK_CONFIG_PWM_PARAMETER                    1
#define QIK_CONFIG_SHUT_DOWN_MOTORS_ON_ERROR        2
#define QIK_CONFIG_SERIAL_TIMEOUT                   3
#define QIK_CONFIG_MOTOR_M0_ACCELERATION            4
#define QIK_CONFIG_MOTOR_M1_ACCELERATION            5
#define QIK_CONFIG_MOTOR_M0_BRAKE_DURATION          6
#define QIK_CONFIG_MOTOR_M1_BRAKE_DURATION          7
#define QIK_CONFIG_MOTOR_M0_CURRENT_LIMIT_DIV_2     8
#define QIK_CONFIG_MOTOR_M1_CURRENT_LIMIT_DIV_2     9
#define QIK_CONFIG_MOTOR_M0_CURRENT_LIMIT_RESPONSE 10
#define QIK_CONFIG_MOTOR_M1_CURRENT_LIMIT_RESPONSE 11

// PololuQik overlay object, good for 2s9v1 and 2s12v10
class PololuQik
{
  public:
    PololuQik(char serial_port_filename[]);

    //char getFirmwareVersion();
    //byte getErrors();

    // Configuration getter/setter
    //byte getConfigurationParameter(byte parameter);
    //byte setConfigurationParameter(byte parameter, byte value);

    // Set Speed Functionality
    void setM0Speed(int speed);
    void setM1Speed(int speed);
    void setSpeeds(int m0Speed, int m1Speed);

};

// 2s12v10 Specific Definitions only
class PololuQik2s12v10 : public PololuQik
{
  public:
    PololuQik2s12v10(char serial_port_filename[]) : PololuQik(serial_port_filename) {}

    // Brake Functionality
    void setM0Brake(unsigned char brake);
    void setM1Brake(unsigned char brake);
    void setBrakes(unsigned char m0Brake, unsigned char m1Brake);

    // Current Functionality
    //unsigned char getM0Current();
    //unsigned char getM1Current();
    //unsigned int getM0CurrentMilliamps();
    //unsigned int getM1CurrentMilliamps();

    // Get Speed Functionality
    //unsigned char getM0Speed();
    //unsigned char getM1Speed();
};

#endif
