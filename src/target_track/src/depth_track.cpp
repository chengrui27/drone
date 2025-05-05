#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <target_track/BoundingBoxes.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/opencv.hpp>

ros::Publisher pos_pub;

int x, y;
bool captured = false;
image_geometry::PinholeCameraModel cam_model;
cv::Mat depth_image;
geometry_msgs::PoseStamped local_pose;

void depth_cb(const sensor_msgs::ImageConstPtr& msg){
    try{
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        depth_image = cv_ptr->image;
    } catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void camera_info_cb(const sensor_msgs::CameraInfoConstPtr& cam_info){
    cam_model.fromCameraInfo(cam_info);
}

void target_cb(const target_track::BoundingBoxes::ConstPtr& msg){
    for(const auto& box : msg->bounding_boxes){
        if(box.Class == "person"){           
            x = (box.xmin + box.xmax) / 2;
            y = (box.ymin + box.ymax) / 2;
            captured = true;
            // ROS_INFO("Target detected at pixel (%d, %d)", x, y);
            return;
        }
    }
}

void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg){
    local_pose.header.stamp = msg->header.stamp;
    local_pose.pose = msg->pose.pose;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "yolov5_track_node");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    ros::Subscriber target_sub = nh.subscribe("/yolov5/targets", 3, target_cb);
    ros::Subscriber depth_sub = nh.subscribe("/camera/depth/image_rect_raw", 1, depth_cb);
    ros::Subscriber cam_info_sub = nh.subscribe("/camera/depth/camera_info", 1, camera_info_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/vins_estimator/odometry", 1, local_pos_cb);
    pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

    while(ros::ok()){
        if(captured && !depth_image.empty()){
            uint16_t depth_val = depth_image.at<uint16_t>(y, x);
            float depth_m = depth_val * 0.001;  // mm → m

            if(depth_m < 0.001 || depth_m > 10.0){
                ROS_WARN("Invalid depth value: %f m", depth_m);
                captured = false;
            } else {
                cv::Point2d pixel_point(x, y);
                cv::Point3d ray = cam_model.projectPixelTo3dRay(pixel_point);
                cv::Point3d point_3d = ray * depth_m;

                geometry_msgs::PoseStamped pose;
                pose.header.stamp = ros::Time::now();
                // pose.header.frame_id = "map"; // 根据你无人机的参考坐标系修改

                // 坐标系调整（realsense → 无人机坐标系）
                pose.pose.position.x = point_3d.z + local_pose.pose.position.x - 0.5;
                pose.pose.position.y = -point_3d.x + local_pose.pose.position.y;
                pose.pose.position.z = -point_3d.y + local_pose.pose.position.z;

                pos_pub.publish(pose);
                ROS_INFO("pos: [%.2f, %.2f, %.2f]", 
                         pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
            }
        }

        ros::spinOnce();
        rate.sleep();
    }
}