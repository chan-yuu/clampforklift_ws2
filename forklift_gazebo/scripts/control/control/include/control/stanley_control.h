/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-09-19 23:27:57
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2024-10-16 15:05:51
 * @FilePath: /undefined/home/cyun/forklift_sim_ws/src/control/control/include/control/stanley_control.h
 * @Description: 
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#ifndef STANLEY_CONTROLLER_H
#define STANLEY_CONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

class StanleyController
{
public:
    // 控制参数
    double kp_;  // Stanley 控制器增益
    double kp_dhead;// 新加一个航向的增益，最小化转弯的偏差，同时让车辆的转向更加明显
    double max_steering_angle_;  // 最大转向角
    double wheelbase_;  // 车辆轴距
    int preview_point;  // 预瞄点
    double end_dis_fix;
    double kp_rear_cte;
    
    StanleyController(ros::NodeHandle &private_nh);  // 构造函数
    // 轨迹跟踪函数
    double computeSteeringAngle(double yaw_error, double cross_track_error,double current_speed_);  // 跟踪路径函数

      ros::Time lastPrintTime;

};

#endif // STANLEY_CONTROLLER_H

