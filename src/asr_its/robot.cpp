#include <ros/ros.h>
#include "robot.h"

Robot::Robot(): RosRate(10)
{
    ROS_INFO("Debug");
    SubButton   = Nh.subscribe("controller/button", 16, &Robot::CallbackButton, this);
    SubAxis     = Nh.subscribe("controller/axis", 6, &Robot::CallbackAxis, this);
    PubSpeed    = Nh.advertise<asr_its::ControllerData>("robot/cmd_vel", 10);

    for (int i = 0; i <=2 ; i++){
        MsgSpeed.data.push_back(0);
    }

    StikButton.data = 4095;

    for (int i = 0; i < 12; i++){
        MyController.button[i] = (StikButton.data >> i) & 1;
    }

    while(ros::ok()){

        if (MyController.button[l2])
        {
            RobotSpeed[0] = MyController.axis[0] / 1600;
            RobotSpeed[1] = -1 * MyController.axis[1] / 1600;
            RobotSpeed[2] = MyController.axis[3] / 1700;
        }
        else
        {
            RobotSpeed[0] = MyController.axis[0] / 1000;
            RobotSpeed[1] = -1 * MyController.axis[1] / 1000;
            RobotSpeed[2] = MyController.axis[3] / 1300;
        }

        if (MyController.button[segitiga] == 0 && MyController.prev_button[segitiga] == 1)
        {
            StatusControl ^= 1;
        }

        MyController.prev_button[segitiga] = MyController.button[segitiga];

        // if (MyController.button[bulat] == 0 && MyController.prev_button[bulat] != 0)
        // {
        //     OffsetPos[0] = PosisiOdom[0] + OffsetPos[0];
        //     OffsetPos[1] = PosisiOdom[1] + OffsetPos[1];
        //     OffsetPos[2] = PosisiOdom[2] + OffsetPos[2];
        // }

        // MyController.prev_button[bulat] = MyController.button[bulat];

        // if (MyController.button[eks] == 0)
        // {
        //     OffsetPos[0] = 0;
        //     OffsetPos[1] = 0;
        //     OffsetPos[2] = 0;
        // }

        std::cout << "RobotSpeed[0] : " << RobotSpeed[0] << " RobotSpeed[1] : " << RobotSpeed[1] << " RobotSpeed[2] : " << RobotSpeed[2] << std::endl;

        for(int i = 0 ; i<=2 ; i++){
            MsgSpeed.data.at(i) = RobotSpeed[i];
        }

        MsgSpeed.StatusControl = StatusControl;

        PubSpeed.publish(MsgSpeed);
        ros::spinOnce(); 
        RosRate.sleep();
    }
};

Robot::~Robot(){}

void Robot::CallbackButton(const std_msgs::Int32 &MsgBtn)
{
    StikButton = MsgBtn;

    for (int i = 0; i < 12; i++){
        MyController.button[i] = (StikButton.data >> i) & 1;
    }
}

void Robot::CallbackAxis(const std_msgs::Int16MultiArray &MsgAxis)
{
    StikAxis = MsgAxis;

    MyController.axis[0] = MsgAxis.data.at(0);
    MyController.axis[1] = MsgAxis.data.at(1);
    MyController.axis[2] = MsgAxis.data.at(2);
    MyController.axis[3] = MsgAxis.data.at(3);
}

void Robot::OdomCallback(const nav_msgs::OdometryConstPtr &msg){
}