/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-09-16 18:22:29
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2024-09-16 18:37:51
 * @FilePath: /undefined/home/cyun/clamp_forklift_ws/src/control/include/control/pid_control.h
 * @Description: 
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <mutex>
#include <tf2/LinearMath/Quaternion.h>  // 用于 tf2::Quaternion
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // 用于 tf2::fromMsg
#include <tf2/utils.h>  // 用于 tf2::getYaw


class PIDController
{
public:
    PIDController();
    void run();

private:
    // ROS 节点句柄
    ros::NodeHandle nh_;

    // 订阅者和发布者
    ros::Subscriber odom_sub_;
    ros::Subscriber path_sub_;
    ros::Subscriber stop_sub_;
    ros::Publisher throttle_pub_;
    ros::Publisher brake_pub_;
    ros::Publisher steering_pub_;
    ros::Publisher gear_pub_;
    ros::Publisher stop_pub_;

    // 控制参数
    double kp_, ki_, kd_;
    double prev_error_, integral_;

    // 车辆状态
    double current_x_, current_y_, current_yaw_;
    nav_msgs::Path path_;
    bool has_path_;
    bool has_odom_;
    bool emergency_stop_;

    // 控制频率
    ros::Rate control_rate_;

    // 数据锁
    std::mutex data_mutex_;

    // 回调函数
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void stopCallback(const std_msgs::Bool::ConstPtr& msg);

    // PID控制函数
    double computePIDControl(double error);

    // 轨迹跟踪函数
    void trackPath();
};

#endif // PID_CONTROLLER_H
