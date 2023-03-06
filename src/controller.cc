#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int16MultiArray.h"

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Copyright Drew Noakes 2013-2016

#include "joystick.hh"
#include <unistd.h>

int a = 0b00000000000;
unsigned char b = 0;
unsigned char c = 0;
//int b = 0b00000000;


ros::Publisher pub_button;
ros::Publisher pub_axis;
std_msgs::Int32 msg_button;
std_msgs::Int16MultiArray msg_axis;


int main(int argc, char** argv)
{

  ros::init(argc, argv, "controller");

  ros::NodeHandle nh;
  ros::Rate rosRate(200);

  pub_button = nh.advertise<std_msgs::Int32>("controller/button",17);
  pub_axis = nh.advertise<std_msgs::Int16MultiArray>("controller/axis", 6);
  msg_button.data=0;
  msg_axis.data.clear();
  // msg_axis.data[0]=123;
  // msg_axis.data[1]=230;
  // msg_axis.data[2]=905;
  // msg_axis.data[3]=322;
  // msg_axis.data[4]=888;
  // msg_axis.data[5]=111;


		//for loop, pushing data in the size of the array
		for (int i = 0; i < 4; i++)
		{
			//assign array a random number between 0 and 255.
			msg_axis.data.push_back(0);
    }

//  printf ("%d\n",a);
//  printf ("%d\n",1<<3);
  // Create an instance of Joystick
  Joystick joystick("/dev/input/js0");

  // Ensure that it was found and that we can use it
  if (!joystick.isFound())
  {
    printf("open failed.\n");
    exit(1);
  }

  while (ros::ok())
  {
    // Restrict rate
    usleep(1000);

    // Attempt to sample an event from the joystick
    JoystickEvent event;
    if (joystick.sample(&event))
    {
      if (event.isButton())
      {
        printf("Button %u is %s\n",
          event.number,
          event.value == 0 ? "press" : "release");

          a=(1 << event.number)^(a);
          // printf("%d              ",a);
          // printf("%d , %d \n", event.number, event.value);

          msg_button.data=a;
          pub_button.publish(msg_button);
      }
      else if (event.isAxis())
      {
         if (event.number <= 3) //axis 4-5 di skip supaya hemat data
         {
          msg_axis.data.at(event.number)=event.value;
          pub_axis.publish(msg_axis);
          if (msg_axis.data.at(event.number)==0) //kalau nilainya 0 langsung di spam 0 3 kali biar aman
          {
            for (int i=0; i<2; i++)
            {
              pub_axis.publish(msg_axis);
              printf("000 Axis %u is at position %d\n", event.number, event.value);
            }
          }
          printf("Axis %u is at position %d\n", event.number, event.value);
         }
      }
    }


    // msg_button.data=a;
    // pub_button.publish(msg_button);

    ros::spinOnce();
    ros::Duration(0.001).sleep();
  }
  return 0; 
}
