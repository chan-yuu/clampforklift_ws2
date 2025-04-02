/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-09-25 07:47:58
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2024-09-25 13:58:40
 * @FilePath: /undefined/home/nvidia/clamp_forklift_ws/src/perception/src/gps_odom.cpp
 * @Description: 
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "car_interfaces/GpsImuInterface.h"

class TransformNode {
public:
    TransformNode() {
        location_sub_ = nh_.subscribe("/gps_imu", 10, &TransformNode::locationCallback, this);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);
    }

    void locationCallback(const car_interfaces::GpsImuInterface::ConstPtr& msg) {
        // 转换坐标 odom只使用x,y,yaw，其他数据都不用于数据的转换？
        double x = msg->x;
        double y = msg->y;
        // double z = msg->z;
        double z = 0.0;
        // double roll = msg->roll * M_PI / 180.0;
        // double pitch = msg->pitch * M_PI / 180.0;
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = msg->yaw * M_PI / 180.0;

        // 发布 Odom 数据
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.header.frame_id = "map";
        odom_msg.child_frame_id = "base_footprint";
        odom_msg.pose.pose.position.x = x - 525792;
        odom_msg.pose.pose.position.y = y - 4316542;
        odom_msg.pose.pose.position.z = z;

        // 使用四元数转换欧拉角
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        odom_pub_.publish(odom_msg);

        // 发布 TF
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = ros::Time::now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "base_footprint";
        transform.transform.translation.x = x - 525792;
        transform.transform.translation.y = y - 4316542;
        transform.transform.translation.z = z;
        transform.transform.rotation = odom_msg.pose.pose.orientation;

        tf_broadcaster_.sendTransform(transform);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber location_sub_;
    ros::Publisher odom_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_; // 使用 tf2 的广播器
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "transform_node");
    TransformNode transform_node;
    ros::spin();
    return 0;
}
