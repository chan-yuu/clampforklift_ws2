/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-09-21 23:21:31
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2025-01-18 01:54:02
 * @FilePath: /undefined/home/cyun/forklift_sim_ws3/src/perception/src/tf_tree_odom_pub.cpp
 * @Description: 发布定位的信息odom 终点计算使用的dis_odom(始终以前轴为定位点)
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Int8.h>
class OdometryPublisher
{
public:
    OdometryPublisher(ros::NodeHandle& nh)
        : tf_listener_(tf_buffer_)
    {
        // 从launch文件或参数服务器获取源帧和目标帧名称
        nh.param<std::string>("source_frame", source_frame_, "base_footprint");
        nh.param<std::string>("target_frame", target_frame_, "map");

        // 发布 odom 话题
        odom_pub_ = nh.advertise<nav_msgs::Odometry>("/odom", 1);
        odom_pub_front_ = nh.advertise<nav_msgs::Odometry>("/dis_odom", 1);
        odom_pub_rear_ = nh.advertise<nav_msgs::Odometry>("/rear_odom", 1);
        odom_pub_front_front_ = nh.advertise<nav_msgs::Odometry>("/front_front_odom", 1);

        odom_ls_lidar = nh.advertise<nav_msgs::Odometry>("/ls_odom", 1);

        // 从参数服务器获取 odom 和 base_link 的 frame id
        nh.param<std::string>("odom_frame", odom_.header.frame_id, "map");
        nh.param<std::string>("child_frame", odom_.child_frame_id, "base_link");

        gear_sub_ = nh.subscribe("/gear_state", 1, &OdometryPublisher::gear_Callback, this);
        current_gear_.data = 0;
    }

    void gear_Callback(const std_msgs::Int8::ConstPtr& msg){
        current_gear_ = *msg;
    }

    void publishOdom()
    {
        ros::Time current_time = ros::Time::now();
        if(current_gear_.data==1)
        {
            try
            {
                target_frame_ = "map";
                source_frame_ = "base_footprint";
                // 从tf树中获取 source_frame 到 target_frame 的变换
                geometry_msgs::TransformStamped transform_stamped = tf_buffer_.lookupTransform(target_frame_, source_frame_, ros::Time(0));

                // 设置时间戳
                odom_.header.stamp = ros::Time::now();

                // 设置位置
                odom_.pose.pose.position.x = transform_stamped.transform.translation.x;
                odom_.pose.pose.position.y = transform_stamped.transform.translation.y;
                odom_.pose.pose.position.z = transform_stamped.transform.translation.z;

                // 设置四元数的位姿
                odom_.pose.pose.orientation = transform_stamped.transform.rotation;

                // 初始化线速度和角速度为零，因为我们只处理位姿数据
                odom_.twist.twist.linear.x = 0.0;
                odom_.twist.twist.linear.y = 0.0;
                odom_.twist.twist.linear.z = 0.0;

                odom_.twist.twist.angular.x = 0.0;
                odom_.twist.twist.angular.y = 0.0;
                odom_.twist.twist.angular.z = 0.0;

                // 发布 odom 数据
                odom_pub_.publish(odom_);
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN_THROTTLE(1.0, "Could not transform %s to %s: %s", target_frame_.c_str(), source_frame_.c_str(), ex.what());
            }
        }

        else // 3或者其他都是以front_axle
        {
            try
            {
                target_frame_ ="map";
                source_frame_ = "front_axle";
                // 从tf树中获取 source_frame 到 target_frame 的变换
                geometry_msgs::TransformStamped transform_stamped = tf_buffer_.lookupTransform(target_frame_, source_frame_, ros::Time(0));

                // 设置时间戳
                odom_.header.stamp = ros::Time::now();

                // 设置位置
                odom_.pose.pose.position.x = transform_stamped.transform.translation.x;
                odom_.pose.pose.position.y = transform_stamped.transform.translation.y;
                odom_.pose.pose.position.z = transform_stamped.transform.translation.z;

                // 设置四元数的位姿
                odom_.pose.pose.orientation = transform_stamped.transform.rotation;

                // 初始化线速度和角速度为零，因为我们只处理位姿数据
                odom_.twist.twist.linear.x = 0.0;
                odom_.twist.twist.linear.y = 0.0;
                odom_.twist.twist.linear.z = 0.0;

                odom_.twist.twist.angular.x = 0.0;
                odom_.twist.twist.angular.y = 0.0;
                odom_.twist.twist.angular.z = 0.0;

                // 发布 odom 数据
                odom_pub_.publish(odom_);
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN_THROTTLE(1.0, "Could not transform %s to %s: %s", target_frame_.c_str(), source_frame_.c_str(), ex.what());
            }
        }

        try
        {
            target_frame_ ="map";
            source_frame_ = "front_axle";
            // 从tf树中获取 source_frame 到 target_frame 的变换
            geometry_msgs::TransformStamped transform_stamped = tf_buffer_.lookupTransform(target_frame_, source_frame_, ros::Time(0));
            // 设置时间戳
            odom_.header.stamp = ros::Time::now();
            // 设置位置
            odom_.pose.pose.position.x = transform_stamped.transform.translation.x;
            odom_.pose.pose.position.y = transform_stamped.transform.translation.y;
            odom_.pose.pose.position.z = transform_stamped.transform.translation.z;
            // 设置四元数的位姿
            odom_.pose.pose.orientation = transform_stamped.transform.rotation;
            // 初始化线速度和角速度为零，因为我们只处理位姿数据
            odom_.twist.twist.linear.x = 0.0;
            odom_.twist.twist.linear.y = 0.0;
            odom_.twist.twist.linear.z = 0.0;
            odom_.twist.twist.angular.x = 0.0;
            odom_.twist.twist.angular.y = 0.0;
            odom_.twist.twist.angular.z = 0.0;
            // 发布 odom 数据
            odom_pub_front_.publish(odom_);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN_THROTTLE(1.0, "Could not transform %s to %s: %s", target_frame_.c_str(), source_frame_.c_str(), ex.what());
        }
        
        
        try
        {
            target_frame_ ="map";
            source_frame_ = "base_footprint";
            // 从tf树中获取 source_frame 到 target_frame 的变换
            geometry_msgs::TransformStamped transform_stamped = tf_buffer_.lookupTransform(target_frame_, source_frame_, ros::Time(0));
            // 设置时间戳
            odom_.header.stamp = ros::Time::now();
            // 设置位置
            odom_.pose.pose.position.x = transform_stamped.transform.translation.x;
            odom_.pose.pose.position.y = transform_stamped.transform.translation.y;
            odom_.pose.pose.position.z = transform_stamped.transform.translation.z;
            // 设置四元数的位姿
            odom_.pose.pose.orientation = transform_stamped.transform.rotation;
            // 初始化线速度和角速度为零，因为我们只处理位姿数据
            odom_.twist.twist.linear.x = 0.0;
            odom_.twist.twist.linear.y = 0.0;
            odom_.twist.twist.linear.z = 0.0;
            odom_.twist.twist.angular.x = 0.0;
            odom_.twist.twist.angular.y = 0.0;
            odom_.twist.twist.angular.z = 0.0;
            // 发布 odom 数据
            odom_pub_rear_.publish(odom_);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN_THROTTLE(1.0, "Could not transform %s to %s: %s", target_frame_.c_str(), source_frame_.c_str(), ex.what());
        }

        try
        {
            target_frame_ ="map";
            source_frame_ = "front_front_axle";
            // 从tf树中获取 source_frame 到 target_frame 的变换
            geometry_msgs::TransformStamped transform_stamped = tf_buffer_.lookupTransform(target_frame_, source_frame_, ros::Time(0));
            // 设置时间戳
            odom_.header.stamp = ros::Time::now();
            // 设置位置
            odom_.pose.pose.position.x = transform_stamped.transform.translation.x;
            odom_.pose.pose.position.y = transform_stamped.transform.translation.y;
            odom_.pose.pose.position.z = transform_stamped.transform.translation.z;
            // 设置四元数的位姿
            odom_.pose.pose.orientation = transform_stamped.transform.rotation;
            // 初始化线速度和角速度为零，因为我们只处理位姿数据
            odom_.twist.twist.linear.x = 0.0;
            odom_.twist.twist.linear.y = 0.0;
            odom_.twist.twist.linear.z = 0.0;
            odom_.twist.twist.angular.x = 0.0;
            odom_.twist.twist.angular.y = 0.0;
            odom_.twist.twist.angular.z = 0.0;
            // 发布 odom 数据
            odom_pub_front_front_.publish(odom_);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN_THROTTLE(1.0, "Could not transform %s to %s: %s", target_frame_.c_str(), source_frame_.c_str(), ex.what());
        }

    }

private:
    ros::Publisher odom_pub_;
    ros::Publisher odom_pub_front_, odom_pub_rear_, odom_pub_front_fix, odom_pub_front_middle_, odom_ls_lidar, odom_pub_front_front_;
    ros::Subscriber gear_sub_;

    nav_msgs::Odometry odom_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string source_frame_;
    std::string target_frame_;
    std_msgs::Int8 current_gear_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_publisher");
    ros::NodeHandle nh;

    OdometryPublisher odom_publisher(nh);

    ros::Rate rate(100.0);
    while (ros::ok())
    {
        odom_publisher.publishOdom();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
