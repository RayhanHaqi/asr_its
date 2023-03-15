#include <ros/ros.h>
#include "robot_comhardware.h"

Comhardware::Comhardware(): RosRate(20)
{
    if (RS232_OpenComport(Cport_nr, 115200, Mode))
    {
        printf("Cannot Open COM Port\n");
        ros::shutdown();
    }
    else
    {
        printf("Port Open\n");

        OdomPub = Nh.advertise<nav_msgs::Odometry>("odom", 50);
        VelPub  = Nh.advertise<geometry_msgs::Twist>("cmd_vel", 50);
        SpeedSub = Nh.subscribe("robot/cmd_vel", 10, &Comhardware::SpeedSubCallback, this);

        ThreadSerialTransmit = Nh.createTimer(ros::Duration(0.1), &Comhardware::SerialTransmitEvent, this);
        ThreadSerialReceived = Nh.createTimer(ros::Duration(0.01), &Comhardware::SerialReceiveEvent, this);
        Mts.spin();
    }
};

Comhardware::~Comhardware(){};

void Comhardware::SerialTransmitEvent(const ros::TimerEvent &event)
{
    char data_kirim[23] = {'m', 'r', 'i'};
    memcpy(data_kirim + 3, &RobotSpeed[0], 2);
    memcpy(data_kirim + 5, &RobotSpeed[1], 2);
    memcpy(data_kirim + 7, &RobotSpeed[2], 2);
    data_kirim[9] = BitLamp;
    data_kirim[10] = StatusControl;
    memcpy(data_kirim + 11, &OffsetPos[0], 4);
    memcpy(data_kirim + 15, &OffsetPos[1], 4);
    memcpy(data_kirim + 19, &OffsetPos[2], 4);

    RS232_SendBuf(Cport_nr, (unsigned char *)data_kirim, 23);

}

void Comhardware::SerialReceiveEvent(const ros::TimerEvent &event)
{
    n = RS232_PollComport(Cport_nr, Buf, 4095);

    if (n > 10)
    {
        Buf[n] = 0; /* always put a "null" at the end of a string! */

        for (i = 0; i < n; i++)
        {
            if (Buf[i] < 32) /* replace unreadable control-codes by dots */
            {
                Buf[i] = 0;
            }
        }

        char *token = strtok((char *)Buf, ",");
        DataLen = 0;
        while (token != NULL)
        {
            sLocal[DataLen].assign(token);

            token = strtok(NULL, ",");
            DataLen++;
        }

        if (DataLen == 4 && sLocal[0].length() > 3)
        {
            PosisiOdom[0] = std::stof(sLocal[0]);
            PosisiOdom[1] = std::stof(sLocal[1]);
            PosisiOdom[2] = std::stof(sLocal[2]);

            // meidan filter
            static float BuffPosXFilter[5] = {0};
            static float BuffPosYFilter[5] = {0};
            static float BuffPosZFilter[5] = {0};

            static int IdxBuffFilter = 0;

            BuffPosXFilter[IdxBuffFilter] = PosisiOdom[0];
            BuffPosYFilter[IdxBuffFilter] = PosisiOdom[1];
            BuffPosZFilter[IdxBuffFilter] = PosisiOdom[2];

            if (++IdxBuffFilter > 4)
            {
                IdxBuffFilter = 0;
            }

            // sort buffer
            std::sort(BuffPosXFilter, BuffPosXFilter + 5);
            std::sort(BuffPosYFilter, BuffPosYFilter + 5);
            std::sort(BuffPosZFilter, BuffPosZFilter + 5);

        
            // Regresi Orde 2
            PositionFiltered[0] = -0.1287+0.6901*BuffPosXFilter[2]+0.0001*pow(BuffPosXFilter[2], 2) ;
            PositionFiltered[1] = -0.1006+0.7213*BuffPosYFilter[2]-0.0001*pow(BuffPosYFilter[2], 2) ;
            PositionFiltered[2] = -0.1401+1.0043*BuffPosZFilter[2]+0.0*pow(BuffPosZFilter[2], 2) ;

            for (int i = 0; i < 3; i++)
            {
                VelocityRaw[i] = PositionFiltered[i] - PositionPrev[i];

                PositionPrev[i] = PositionFiltered[i];
            }

            VelocityFilter[0] = VelocityRaw[0] * 0.2 + VelocityFilter[0] * 0.8;
            VelocityFilter[1] = VelocityRaw[1] * 0.2 + VelocityFilter[1] * 0.8;
            VelocityFilter[2] = VelocityRaw[2] * 0.2 + VelocityFilter[2] * 0.8;


            // print position filtered and velocity
            printf("%0.3f,%0.3f,%0.3f || %0.3f,%0.3f,%0.3f\n", PositionFiltered[0], PositionFiltered[1], PositionFiltered[2], VelocityFilter[0], VelocityFilter[1], VelocityFilter[2]);
            
            //print status control
            printf ("vx = %d | vy = %d | vz = %d | status control = %d | bitlamp = %d \n", RobotSpeed[0], RobotSpeed[1], RobotSpeed[2], StatusControl, BitLamp);
            

            CurrentTime = ros::Time::now();
        
            OdomQuat = tf::createQuaternionMsgFromYaw(PositionFiltered[2] * 3.14 / 180);

            Odom.header.stamp = CurrentTime;
            Odom.header.frame_id = "odom";

            //set the position
            Odom.pose.pose.position.x = PositionFiltered[0];
            Odom.pose.pose.position.y = PositionFiltered[1];
            Odom.pose.pose.position.z = 0.0;
            Odom.pose.pose.orientation = OdomQuat;

            //set the velocity
            Odom.child_frame_id = "base_link";
            Odom.twist.twist.linear.x = VelocityFilter[0];
            Odom.twist.twist.linear.y = VelocityFilter[1];
            Odom.twist.twist.angular.z = VelocityFilter[2];

            //publish the message
            OdomPub.publish(Odom);

            RobotVel.linear.x = VelocityFilter[0];
            RobotVel.linear.y = VelocityFilter[1];
            RobotVel.angular.z = VelocityFilter[2];

            VelPub.publish(RobotVel);
        }
        memset(Buf, 0, 4095);
    }
};

void Comhardware::SpeedSubCallback(const asr_its::ControllerData &msg)
{
    StatusControl = msg.StatusControl;
    RobotSpeed[0] = msg.data[0];
    RobotSpeed[1] = msg.data[1];
    RobotSpeed[2] = msg.data[2];
}