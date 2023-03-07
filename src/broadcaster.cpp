#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

geometry_msgs::TransformStamped odom_trans;
tf::TransformBroadcaster        odom_broadcaster;

void callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom_trans.header.frame_id = "odom";
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = msg->pose.pose.position.x;
    odom_trans.transform.translation.y = msg->pose.pose.position.y;
    odom_trans.transform.translation.z = msg->pose.pose.position.z;
    odom_trans.transform.rotation = msg->pose.pose.orientation;

    odom_broadcaster.sendTransform(odom_trans);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_base_link_broadcaster");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/odom", 10, &callback);

    ros::spin();
    return 0;
};

