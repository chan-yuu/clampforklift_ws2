/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-09-15 13:39:13
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2024-09-23 13:17:37
 * @FilePath: /control/src/pid_control.cpp
 * @Description: pid控制
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#include "control/pid_control.h"
#include <tf/tf.h>
#include <cmath>

PIDController::PIDController()
    : control_rate_(100),  // 控制频率设置为10Hz
      prev_error_(0.0), integral_(0.0), 
      has_path_(false),has_odom_(false), emergency_stop_(false)
{
    // 从参数服务器读取PID参数
    ros::NodeHandle private_nh("~");
    private_nh.param("kp", kp_, 1.0);  // 默认为1.0
    private_nh.param("ki", ki_, 0.0);  // 默认为0.0
    private_nh.param("kd", kd_, 0.1);  // 默认为0.1

    // 订阅话题
    odom_sub_ = nh_.subscribe("/odom", 10, &PIDController::odomCallback, this);
    path_sub_ = nh_.subscribe("/path", 10, &PIDController::pathCallback, this);
    stop_sub_ = nh_.subscribe("/stop", 10, &PIDController::stopCallback, this);
    

    // 发布控制指令
    throttle_pub_ = nh_.advertise<std_msgs::Float64>("/throttle_cmd", 10);
    brake_pub_ = nh_.advertise<std_msgs::Float64>("/brake_cmd", 10);
    steering_pub_ = nh_.advertise<std_msgs::Float64>("/steering_cmd", 10);
    gear_pub_ = nh_.advertise<std_msgs::UInt8>("/gear_cmd", 10);
    stop_pub_ = nh_.advertise<std_msgs::Bool>("/stop", 10);
}

double PIDController::computePIDControl(double error)
{
    // 计算积分项
    integral_ += error;
    
    // 计算微分项
    double derivative = error - prev_error_;

    // 计算PID控制量
    double control_output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    std::cout<<"kp_: "<<kp_<<std::endl;

    // 更新误差
    prev_error_ = error;

    return control_output;
}

// 里程计的回调函数，更新当前车辆的位置和姿态
void PIDController::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    current_yaw_ = tf::getYaw(msg->pose.pose.orientation);  // 从四元数中提取偏航角 [red]
    // std::cout<<"sub current_yaw: "<<current_yaw_*180/M_PI<<std::endl;
    has_odom_ = true;
}

// 路径的回调函数，更新当前的目标路径
void PIDController::pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    path_ = *msg;
    has_path_ = true;
}

// 急停的回调函数，更新急停状态
void PIDController::stopCallback(const std_msgs::Bool::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    emergency_stop_ = msg->data;
}

void PIDController::trackPath()
{
    if (!has_path_ || path_.poses.empty() || !has_odom_)
    {
        ROS_WARN("No path or odom available to track");
        return;
    }
    else
    {
        ROS_WARN_ONCE("Path And Odom Init Success!!");
    }

    // 锁定数据
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (emergency_stop_)
    {
        std_msgs::Float64 brake_msg;
        brake_msg.data = 1.0;  // 最大刹车力度
        brake_pub_.publish(brake_msg);

        std_msgs::Bool stop_msg;
        stop_msg.data = true;  // 触发急停
        stop_pub_.publish(stop_msg);

        return;
    }

    // 找到最近的路径点
    double min_distance = std::numeric_limits<double>::max();
    int nearest_index = -1;

    for (size_t i = 0; i < path_.poses.size(); ++i)
    {
        double path_x = path_.poses[i].pose.position.x;
        double path_y = path_.poses[i].pose.position.y;

        double distance = std::sqrt(std::pow(path_x - current_x_, 2) + std::pow(path_y - current_y_, 2));
        if (distance < min_distance)
        {
            min_distance = distance;
            nearest_index = i;
        }
    }

    if (nearest_index == -1 || nearest_index + 1 >= path_.poses.size())
    {
        ROS_WARN("Reached the end of the path or couldn't find a valid path point.");
        return;
    }

    double target_x = path_.poses[nearest_index + 1].pose.position.x;
    double target_y = path_.poses[nearest_index + 1].pose.position.y;

    // double target_yaw = std::atan2(target_y - current_y_, target_x - current_x_);
    // 提取路径点的四元数，转为yaw角
    tf2::Quaternion quat;
    tf2::fromMsg(path_.poses[nearest_index + 1].pose.orientation, quat);
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    double target_yaw = yaw;

    double angle_error = (target_yaw - current_yaw_); // 弧度制
    // 规则：打印角度值，使用弧度制
    ROS_INFO_STREAM("\033[1;32m target_yaw: " << target_yaw * 180 / M_PI);
    ROS_INFO_STREAM("\033[1;32m current_yaw_: " << current_yaw_ * 180 / M_PI);

    while (angle_error > M_PI) angle_error -= 2 * M_PI;
    while (angle_error < -M_PI) angle_error += 2 * M_PI;

    double angle_error_deg = angle_error * 180 / M_PI;

    double steering_angle = computePIDControl(angle_error_deg); // 航向正负对应的转向正负
    // std::cout<<"angle_error_deg: "<<angle_error_deg<<std::endl;
    // std::cout<<"steering_angle: "<<steering_angle<< std::endl;
    ROS_INFO_STREAM("\033[1;32m angle_error_deg: " << angle_error_deg);
    ROS_INFO_STREAM("\033[1;32m steering_angle: " << steering_angle);

    double max_steering_angle = 70; // 最大70度
    steering_angle = std::max(-max_steering_angle, std::min(steering_angle, max_steering_angle));

    double linear_velocity = 0.5;

    std_msgs::Float64 throttle_msg;
    throttle_msg.data = linear_velocity;
    throttle_pub_.publish(throttle_msg);

    std_msgs::Float64 steering_msg;
    steering_msg.data = steering_angle;
    steering_pub_.publish(steering_msg);

    std_msgs::UInt8 gear_msg;
    gear_msg.data = 3; // 默认给前进挡
    gear_pub_.publish(gear_msg);

    std_msgs::Bool stop_msg;
    stop_msg.data = false;
    stop_pub_.publish(stop_msg);
}

void PIDController::run()
{
    while (ros::ok())
    {
        trackPath();
        ros::spinOnce();
        control_rate_.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pid_trajectory_controller");

    PIDController controller;
    controller.run();

    return 0;
}
