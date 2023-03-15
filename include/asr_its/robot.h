//ros
#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Pose2D.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "asr_its/ControllerData.h"


//std-libraries
#include <iostream>
#include <string>
#include "stdio.h"
#include "stdlib.h"
// #include "rs232.h"

class Robot
{
public:
    Robot();

    ~Robot();

private:
    int     RobotSpeed[3] = {0, 0, 0};
    int     StatusControl;
    float   PosisiOdom[3];
    float   OffsetPos[3];
    
    struct Stik_T {uint8_t button[17]; int axis[4] = {0, 0, 0, 0}; uint8_t prev_button[17];};
    enum eStikButton {segitiga = 0, bulat = 1, eks = 2, r1 = 5, l2 = 6};
    unsigned char Buf[4096];

    Stik_T MyController;

    ros::NodeHandle     Nh;
    ros::Subscriber     SubButton;
    ros::Subscriber     SubAxis;
    ros::Subscriber     OdomSub;
    ros::Publisher      PubSpeed;
    ros::Rate           RosRate;
    std_msgs::Int32     StikButton;
    nav_msgs::Odometry  Odom;

    std_msgs::Int16MultiArray   StikAxis;
    asr_its::ControllerData     MsgSpeed;

    void CallbackButton (const std_msgs::Int32 &MsgBtn);
    void CallbackAxis   (const std_msgs::Int16MultiArray &MsgAxis);
    void OdomCallback   (const nav_msgs::OdometryConstPtr &msg);

};