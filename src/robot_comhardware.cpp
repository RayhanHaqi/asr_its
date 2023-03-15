#include <ros/ros.h>
#include "robot_comhardware.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_comhardware");

    // ROS_INFO("Starting node...");

    Comhardware asr_com;

    return 0;
}