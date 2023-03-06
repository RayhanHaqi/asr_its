#include "ros/ros.h"
#include "rs232.h"
#include "stdlib.h"
#include "stdio.h"
#include <string.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Pose2D.h"
#include <iostream>

#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

// create publisher odometry
ros::Publisher odom_pub;

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

float posisiOdom[3];
float offsetPos[3];
char bitLamp;

ros::Timer Thread_SerialTransmit;
ros::Timer Thread_SerialReceived;

ros::Subscriber subButton;
ros::Subscriber subAxis;

std_msgs::Int32 stik_Button;
std_msgs::Int16MultiArray stik_Axis;

float positionPrev[3] = {0}, velocityRaw[3] = {0};
float positionFiltered[3] = {0};

// cport_nr=17,  USB1
//

int i, n,
    cport_nr = 16,

    bdrate = 115200;

unsigned char buf[4096];

int status_control;


void SerialTransmitEvent(const ros::TimerEvent &event);
void SerialReceiveEvent(const ros::TimerEvent &event);
void CallbackButton(const std_msgs::Int32 &msg_btn);
void CallbackAxis(const std_msgs::Int16MultiArray &msg_Axis);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ComHardware");

  ros::NodeHandle nh;
  ros::MultiThreadedSpinner mts;
  ros::Rate RosRate(10);

  stik_Button.data = 4095;

  for (int i = 0; i < 12; i++)
  {
    myControler.button[i] = (stik_Button.data >> i) & 1;
  }

  char mode[] = {'8', 'N', '1', 0};

  if (RS232_OpenComport(cport_nr, 115200, mode))
  {
    printf("Cannot Open COM Port\n");

    return (0);
  }
  else
  {
    printf("Port Open\n");
  }

  subButton = nh.subscribe("controller/button", 16, CallbackButton);
  subAxis = nh.subscribe("controller/axis", 6, CallbackAxis);

  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
  


  Thread_SerialTransmit = nh.createTimer(ros::Duration(0.1), SerialTransmitEvent);
  Thread_SerialReceived = nh.createTimer(ros::Duration(0.01), SerialReceiveEvent);
  mts.spin();
}

void SerialTransmitEvent(const ros::TimerEvent &event)
{
  char data_kirim[23] = {'m', 'r', 'i'};
 
  // offsetPos[2]=42.92;
  memcpy(data_kirim + 3, &robotSpeed[0], 2);
  memcpy(data_kirim + 5, &robotSpeed[1], 2);
  memcpy(data_kirim + 7, &robotSpeed[2], 2);
  // status_control = 1;
  // bitLamp = 0b111111101;
  data_kirim[9] = bitLamp;
  data_kirim[10] = status_control;
  memcpy(data_kirim + 11, &offsetPos[0], 4);
  memcpy(data_kirim + 15, &offsetPos[1], 4);
  memcpy(data_kirim + 19, &offsetPos[2], 4);

  RS232_SendBuf(cport_nr, (unsigned char *)data_kirim, 23);

  // std::cout<< posisiOdom[0] << ',' << posisiOdom[1] << ',' << posisiOdom[2] <<'#'<< status_control << std::endl;
  // std::cout<< velocityRaw[0] << ',' << velocityRaw[1] << ',' << velocityRaw[2] <<'#'<< status_control << std::endl;
}
void SerialReceiveEvent(const ros::TimerEvent &event)
{

  n = RS232_PollComport(cport_nr, buf, 4095);

  if (n > 10)
  {
    buf[n] = 0; /* always put a "null" at the end of a string! */

    for (i = 0; i < n; i++)
    {
      if (buf[i] < 32) /* replace unreadable control-codes by dots */
      {
        buf[i] = 0;
      }
    }

    // printf("received %i bytes: %s\n", n, (char *)buf);

    std::string sLocal[5];
    char *token = strtok((char *)buf, ",");
    int data_len = 0;
    while (token != NULL)
    {

      sLocal[data_len].assign(token);

      token = strtok(NULL, ",");
      data_len++;
    }
    if (data_len == 4 && sLocal[0].length() > 3)
    {
      posisiOdom[0] = std::stof(sLocal[0]);
      posisiOdom[1] = std::stof(sLocal[1]);
      posisiOdom[2] = std::stof(sLocal[2]);

      // meidan filter
      static float buffPosXFilter[5] = {0};
      static float buffPosYFilter[5] = {0};
      static float buffPosZFilter[5] = {0};

      static int idxBuffFilter = 0;

      buffPosXFilter[idxBuffFilter] = posisiOdom[0];
      buffPosYFilter[idxBuffFilter] = posisiOdom[1];
      buffPosZFilter[idxBuffFilter] = posisiOdom[2];

      if (++idxBuffFilter > 4)
      {
        idxBuffFilter = 0;
      }

      // sort buffer
      std::sort(buffPosXFilter, buffPosXFilter + 5);
      std::sort(buffPosYFilter, buffPosYFilter + 5);
      std::sort(buffPosZFilter, buffPosZFilter + 5);

      // positionFiltered[0] = buffPosXFilter[2];
      // positionFiltered[1] = buffPosYFilter[2];
      // positionFiltered[2] = buffPosZFilter[2];
      positionFiltered[0] = 0.7137*buffPosXFilter[2] - 0.7887;
      positionFiltered[1] = 0.6983*buffPosYFilter[2] + 0.5634;
      positionFiltered[2] = 1.004*buffPosZFilter[2] + 0.06361;
      // printf("%0.3f,%0.3f,%0.3f\n",posisiOdom[0],posisiOdom[1],posisiOdom[2]);
      // printf("%0.3f,%0.3f,%0.3f#%d\n",positionFiltered[0],positionFiltered[1],positionFiltered[2],status_control);

      for (int i = 0; i < 3; i++)
      {
        velocityRaw[i] = positionFiltered[i] - positionPrev[i];

        positionPrev[i] = positionFiltered[i];
      }

      VelocityFilter[0] = velocityRaw[0] * 0.2 + VelocityFilter[0] * 0.8;
      VelocityFilter[1] = velocityRaw[1] * 0.2 + VelocityFilter[1] * 0.8;
      VelocityFilter[2] = velocityRaw[2] * 0.2 + VelocityFilter[2] * 0.8;
  
 
      // print positon and position filtered
      // printf("%0.3f,%0.3f,%0.3f || %0.3f,%0.3f,%0.3f\n",posisiOdom[0],posisiOdom[1],posisiOdom[2],positionFiltered[0],positionFiltered[1],positionFiltered[2]);

      // print position filtered and velocity
      printf("%0.3f,%0.3f,%0.3f || %0.3f,%0.3f,%0.3f\n", positionFiltered[0], positionFiltered[1], positionFiltered[2], VelocityFilter[0], VelocityFilter[1], VelocityFilter[2]);
     
      //print status control
      printf ("status control = %d \n",status_control);
    





      ros::Time current_time = ros::Time::now();
      // tf::TransformBroadcaster odom_broadcaster;

      // geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(positionFiltered[2]);
        

      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(positionFiltered[2] * 3.14 / 180);

      //first . we 'll publish the transform over tf
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";

      odom_trans.transform.translation.x = positionFiltered[0];
      odom_trans.transform.translation.y = positionFiltered[1];
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;

      tf::TransformBroadcaster odom_broadcaster;
      //send the transform
      odom_broadcaster.sendTransform(odom_trans);

      nav_msgs::Odometry odom;

      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";
    

      //set the position
      odom.pose.pose.position.x = positionFiltered[0];
      odom.pose.pose.position.y = positionFiltered[1];
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;

      //set the velocity
      odom.child_frame_id = "base_link";
      odom.twist.twist.linear.x = VelocityFilter[0];
      odom.twist.twist.linear.y = VelocityFilter[1];
      odom.twist.twist.angular.z = VelocityFilter[2];

      //publish the message
      odom_pub.publish(odom);

      // print velocity and position filtered
      // printf("%0.3f,%0.3f,%0.3f || %0.3f,%0.3f,%0.3f\n",VelocityFilter[0],VelocityFilter[1],VelocityFilter[2],positionFiltered[0],positionFiltered[1],positionFiltered[2]);





      


    }

    memset(buf, 0, 4095);
  }
}

void CallbackButton(const std_msgs::Int32 &msg_btn)
{

  stik_Button = msg_btn;

  for (int i = 0; i < 12; i++)
  {
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
