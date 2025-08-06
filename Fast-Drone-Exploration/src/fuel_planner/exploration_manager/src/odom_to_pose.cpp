#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher pose_pub;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = msg->header;
    pose_msg.pose = msg->pose.pose;

    pose_pub.publish(pose_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_to_pose");
    ros::NodeHandle nh;

    ros::Subscriber odom_sub = nh.subscribe("/vins_fusion/imu_propagate", 10, odomCallback);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/converted_camera_pose", 10);

    ros::spin();
    return 0;
}
