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
    int     Cport_nr;
    int     Bdrate;
    int     IdxBuff;
    int     RobotSpeed[3];
    int     StatusControl;
    float   BuffVelocity_X[100];
    float   BuffVelocity_Y[100];
    float   BuffVelocity_Z[100];
    float   VelocityFilter[3];
    float   PosisiOdom[3];
    float   OffsetPos[3];
    float   PositionPrev[3]; 
    float   VelocityRaw[3];
    float   PositionFiltered[3];
    char    BitLamp;
    
    struct Stik_T {uint8_t button[17]; int axis[4]; uint8_t prev_button[17];};
    enum eStikButton {segitiga, bulat, eks, r1, l2};
    unsigned char Buf[4096];

    Stik_T MyController;
    
    ros::Rate   RosRate;
    ros::Timer  ThreadSerialTransmit;
    ros::Timer  ThreadSerialReceived;

    ros::NodeHandle     Nh;
    ros::Subscriber     SubButton;
    ros::Subscriber     SubAxis;
    ros::Publisher      PubSpeed;
    std_msgs::Int32     StikButton;

    std_msgs::Int16MultiArray   StikAxis;
    // std_msgs::Int16MultiArray   MsgSpeed;
    asr_its::ControllerData     MsgSpeed;

    void CallbackButton (const std_msgs::Int32 &MsgBtn);
    void CallbackAxis   (const std_msgs::Int16MultiArray &MsgAxis);


    
    

};