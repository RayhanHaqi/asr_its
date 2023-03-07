//ros
#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Pose2D.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>


//std-libraries
#include <iostream>
#include <string>
#include "stdio.h"
#include "stdlib.h"
// #include "rs232.h"


typedef enum
{

  segitiga = 0,
  bulat = 1,
  eks = 2,
  r1 = 5,
  l2 = 6

} stik_button;

float buffVelocity_X[100] = {0};
float buffVelocity_Y[100] = {0};
float buffVelocity_Z[100] = {0};

float VelocityFilter[3];

int idxBuff = 0;

typedef struct
{

  uint8_t button[17];
  int axis[4];
  uint8_t prev_button[17];

} stik_t;

stik_t myControler;

short int robotSpeed[3] = {0, 0, 0};
std_msgs::Int16MultiArray msgSpeed;

float posisiOdom[3];
float offsetPos[3];
char bitLamp;

ros::Timer Thread_SerialTransmit;
ros::Timer Thread_SerialReceived;

ros::Subscriber subButton;
ros::Subscriber subAxis;

ros::Publisher pubSpeed;

std_msgs::Int32 stik_Button;
std_msgs::Int16MultiArray stik_Axis;

float positionPrev[3] = {0}, velocityRaw[3] = {0};
float positionFiltered[3] = {0};

// cport_nr=16, 17, ... => USB0, USB1, ...

int i, n,
    cport_nr = 16,

    bdrate = 115200;

unsigned char buf[4096];

int status_control;

void CallbackButton(const std_msgs::Int32 &msg_btn);
void CallbackAxis(const std_msgs::Int16MultiArray &msg_Axis);


int main(int argc, char **argv)
{
  ros::init(argc, argv, "main");

  ros::NodeHandle nh;
  // ros::MultiThreadedSpinner mts;
  ros::Rate RosRate(10);

  for (int i = 0; i <=2 ; i++){
    msgSpeed.data.push_back(0);
  }

  // msgSpeed.data.clear();
  stik_Button.data = 4095;

  for (int i = 0; i < 12; i++){
    myControler.button[i] = (stik_Button.data >> i) & 1;
  }

  subButton = nh.subscribe("controller/button", 16, CallbackButton);
  subAxis = nh.subscribe("controller/axis", 6, CallbackAxis);

  pubSpeed = nh.advertise<std_msgs::Int16MultiArray>("robot/cmd_vel", 10);

  

  printf("Main node started\n");

  while(ros::ok()){

      if (myControler.button[l2])
      {
        robotSpeed[0] = myControler.axis[0] / 800;
        robotSpeed[1] = -1 * myControler.axis[1] / 800;
        robotSpeed[2] = myControler.axis[3] / 1700;
      }
      else
      {
        robotSpeed[0] = myControler.axis[0] / 500;
        robotSpeed[1] = -1 * myControler.axis[1] / 500;
        robotSpeed[2] = myControler.axis[3] / 1300;
      }

      if (myControler.button[segitiga] == 0 && myControler.prev_button[segitiga] == 1)
      {
        status_control ^= 1;
      }

      myControler.prev_button[segitiga] = myControler.button[segitiga];

      if (myControler.button[bulat] == 0 && myControler.prev_button[bulat] != 0)
      {
        offsetPos[0] = posisiOdom[0] + offsetPos[0];
        offsetPos[1] = posisiOdom[1] + offsetPos[1];
        offsetPos[2] = posisiOdom[2] + offsetPos[2];
      }

      myControler.prev_button[bulat] = myControler.button[bulat];

      if (myControler.button[eks] == 0)
      {
        offsetPos[0] = 0;
        offsetPos[1] = 0;
        offsetPos[2] = 0;
      }

      std::cout << "robotSpeed[0] : " << robotSpeed[0] << " robotSpeed[1] : " << robotSpeed[1] << " robotSpeed[2] : " << robotSpeed[2] << std::endl;


      for(int i = 0 ; i<=2 ; i++){
        msgSpeed.data.at(i) = robotSpeed[i];
      }
      pubSpeed.publish(msgSpeed);


      ros::spinOnce(); 
      RosRate.sleep();
  }

//   Thread_SerialTransmit = nh.createTimer(ros::Duration(0.1), SerialTransmitEvent);
//   Thread_SerialReceived = nh.createTimer(ros::Duration(0.01), SerialReceiveEvent);

}

void CallbackButton(const std_msgs::Int32 &msg_btn)
{

  stik_Button = msg_btn;

  for (int i = 0; i < 12; i++){
    myControler.button[i] = (stik_Button.data >> i) & 1;
  }
}
void CallbackAxis(const std_msgs::Int16MultiArray &msg_Axis)
{
  stik_Axis = msg_Axis;

  myControler.axis[0] = msg_Axis.data.at(0);
  myControler.axis[1] = msg_Axis.data.at(1);
  myControler.axis[2] = msg_Axis.data.at(2);
  myControler.axis[3] = msg_Axis.data.at(3);
}