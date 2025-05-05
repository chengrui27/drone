#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

ros::Publisher vision_pub;

void vins_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    geometry_msgs::PoseStamped vision_pose;
    vision_pose.header.stamp = msg->header.stamp;
    // vision_pose.header.frame_id = "map";  // 根据实际情况调整，可能是 "vins_world"
    vision_pose.pose = msg->pose.pose;

    vision_pub.publish(vision_pose);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_to_mavros");
    ros::NodeHandle nh("~");

    ros::Subscriber slam_sub = nh.subscribe<nav_msgs::Odometry>("/vins_estimator/imu_propagate", 100, vins_callback);
    vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);

    ros::spin();
    return 0;
}


// #include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <nav_msgs/Odometry.h>

// ros::Publisher vision_pub;

// void vins_callback(const nav_msgs::Odometry::ConstPtr &msg)
// {
//     geometry_msgs::PoseStamped vision_pose;
//     vision_pose.header.stamp = msg->header.stamp;
//     // vision_pose.header.frame_id = "map";  // 根据实际情况调整，可能是 "vins_world"
//     vision_pose.pose = msg->pose.pose;

//     vision_pub.publish(vision_pose);
// }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "vins_to_mavros");
//     ros::NodeHandle nh("~");

//     ros::Subscriber slam_sub = nh.subscribe<nav_msgs::Odometry>("/vins_estimator/odometry", 100, vins_callback);
//     vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);

//     ros::spin();
//     return 0;
// }
