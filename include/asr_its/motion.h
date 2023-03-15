#include "stdlib.h"
#include "stdio.h"
#include <string.h>

#include "ros/ros.h"

class Motion
{
public:
    Motion();
    ~Motion();

private:
    ros::NodeHandle     Nh;
    ros::Publisher      OdomPub;
    ros::Subscriber     SpeedSub;
    
    ros::Rate   RosRate;

};
