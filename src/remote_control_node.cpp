/**
 * Remote control node
 */

#include <unistd.h>
#include <termios.h>
#include <stdio.h>
#include <ros/ros.h>

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

    bool stop = false;
    char c=0;


    while(nh.ok() && stop == false)
    {
        setupTerminal();
        c=getch();

        if (c == ESCAPE_KEY && getch() == LEFT_BRACKET_KEY){
            c = getch();
            //TODO: ALL OF THIS STUFF
            switch (c)
            {
                case UP_ARROW:
                //ROS_INFO("UP ARROW\n");
                break;
                case DOWN_ARROW:
                //ROS_INFO("DOWN ARROW\n");
                break;
                case LEFT_ARROW:
                //ROS_INFO("LEFT ARROW\n");
                break;
                case RIGHT_ARROW:
                //ROS_INFO("RIGHT ARROW\n");
                break;
                default:
                //ROS_INFO("idk...\n");
                break;
            }
        }
        else{
            switch (c)
            {
                //TODO: E-STOP CODE
                case 's':
                case 'e':
                    stop=true;
                break;
                default:
                //ROS_INFO("idk...\n");
                break;
            }
        }
        resetTerminal();

    } while (stop == false);

    ROS_INFO("Done");
}
