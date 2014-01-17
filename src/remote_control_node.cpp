/**
 * Remote control node
 */

#include <unistd.h>
#include <termios.h>
#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define UP_ARROW    65
#define DOWN_ARROW  66
#define RIGHT_ARROW 67
#define LEFT_ARROW  68
#define ESCAPE_KEY 27
#define LEFT_BRACKET_KEY 91

 struct termios old = {0};

 void setupTerminal()
 {
    if (tcgetattr(0, &old) < 0)
        perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");
}

void resetTerminal()
{
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
        perror ("tcsetattr ~ICANON");
}

char getch() {
    char buf = 0;
    if (read(0, &buf, 1) < 0)
        perror ("read()");
    return (buf);
}

int main (int argc, char ** argv)
{
    ros::init(argc, argv, "remote_control_node");

    ros::NodeHandle nh;


    ROS_INFO("Starting Remote Control Node");

    ROS_INFO_STREAM(std::string("Remote Control the robot...\n\n\tUse the arrow keys to move the bot\n\t")<<
        std::string("Type h for help\n\tType 't' to change default cmd_vel topic (defaults to \\cmd_vel")<<
            std::string("\n\ttype 's' or 'e' or to stop (e-stop)"));

    // Publishers
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    geometry_msgs::Twist stop_msg;

    stop_msg.linear.x = 0;
    stop_msg.linear.y = 0;
    stop_msg.linear.z = 0;
    stop_msg.angular.x= 0;
    stop_msg.angular.y= 0;
    stop_msg.angular.z= 0;

    geometry_msgs::Twist cmd_vel_msg;
    cmd_vel_msg = stop_msg;

    bool stop = false;
    char c=0;


    while(nh.ok() && stop == false)
    {
        ros::spinOnce();

        setupTerminal();
        c=getch();

        double angMult = 1.0;
        double linMult = 1.0;

        if (c == ESCAPE_KEY && getch() == LEFT_BRACKET_KEY){
            c = getch();
            //TODO: ALL OF THIS STUFF
            switch (c)
            {
                case UP_ARROW:
                    cmd_vel_msg = stop_msg;
                    cmd_vel_msg.linear.x = 1 * linMult;
                    break;
                case DOWN_ARROW:
                    cmd_vel_msg = stop_msg;
                    cmd_vel_msg.linear.x = -1 * linMult;
                    break;
                case LEFT_ARROW:
                    cmd_vel_msg = stop_msg;
                    cmd_vel_msg.angular.z = 1 * angMult;
                    break;
                case RIGHT_ARROW:
                    cmd_vel_msg = stop_msg;
                    cmd_vel_msg.angular.z = -1 * angMult;
                    break;
                default:
                    //ROS_INFO("idk...\n");
                    break;
            }
        }
        else{
            switch (c)
            {
                case 's':
                    cmd_vel_msg = stop_msg;
                    break;
                case 'e':
                    cmd_vel_msg = stop_msg;
                    stop=true;
                    break;
                default:
                    //ROS_INFO("idk...\n");
                    break;
            }
        }
        resetTerminal();

        cmd_vel_pub.publish(cmd_vel_msg);

    } while (stop == false);

    ROS_INFO("Done");
}
