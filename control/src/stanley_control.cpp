/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-09-23 14:31:27
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2025-01-12 16:47:05
 * @FilePath: /control/src/stanley_control.cpp
 * @Description: 
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#include "control/stanley_control.h"
#include <cmath>
#include <tf/tf.h>

StanleyController::StanleyController(ros::NodeHandle &private_nh)
    :kp_(1.0), max_steering_angle_(1.22), wheelbase_(2.0),preview_point(5) // 初始化 tfListener_ 并连接 tfBuffer_
{
    // 从参数服务器读取Stanley控制参数
    // ros::NodeHandle private_nh("~");
    // private_nh.param("/control/stanley_controller/kp", kp_, 0.5);  // 默认值1.0
    private_nh.param("/control/stanley_controller/max_steering_angle", max_steering_angle_, 1.30);  // 最大转向角
    
    private_nh.param("/control/stanley_controller/kp_dhead", kp_dhead, 1.0);  // 最大转向角

    private_nh.param("/control/stanley_controller/wheelbase", wheelbase_, 1.77);  // 车辆轴距
    lastPrintTime = ros::Time::now();
}

double StanleyController::computeSteeringAngle(double yaw_error, double cross_track_error,double current_speed_,double curvature, double current_steering_,double kp_from_control)
{
    static int count = 0; // 定义一个静态计数器
    const int print_interval = 100; // 设置打印间隔

    if (curvature<=0.1) kp_dhead = 0.4;
    else if (current_speed_>0.1&&current_speed_<=0.3) kp_dhead = 0.5;
    else if (current_speed_>0.3&&current_speed_<=0.6) kp_dhead = 0.6;
    else kp_dhead = 0.8;
    // Stanley 控制器计算公式
    double heading_control = - kp_dhead * yaw_error; // dhead的影响
    
    double current_speed_min = 0.10;
    double current_speed_reindex = std::max(current_speed_, current_speed_min);
    current_speed_reindex = 1.0;
    // std::cout<<"current_speed_"<<current_speed_<<std::endl;
    if (current_speed_<=1.0) kp_from_control = 1.0;
    else if (current_speed_>1.0&&current_speed_<=2.0) kp_from_control = 0.3;
    else if (current_speed_>2.0&&current_speed_<=3.0) kp_from_control = 0.2;
    else if (current_speed_>3.0) kp_from_control = 0.15;
    double cross_track_control = -std::atan2(kp_from_control * cross_track_error, current_speed_reindex);

    // 仿真中是负的 cte 和 dhead作用
    // double steering_angle = -(heading_control + cross_track_control);
    double steering_angle = (heading_control + cross_track_control);
    // double steering_angle_deg = steering_angle * 180 / M_PI;
    // 1.反馈的转角
    // 2.曲率
    
    // 限制转向角
    if(steering_angle>max_steering_angle_) steering_angle=max_steering_angle_;
    if(steering_angle<-max_steering_angle_) steering_angle=-max_steering_angle_;

    // steering_angle = std::max(-max_steering_angle_, std::min(steering_angle, max_steering_angle_));
    
    // 控制已经测试完毕，不再需要打印这些内容了
    // 定次数打印，防止刷屏
    
    ros::Time currentTime = ros::Time::now();
    if ((currentTime - lastPrintTime).toSec() >= 1.0) 
    {
        // 打印误差项和控制量
        ROS_INFO("--------- Stanley Control ---------");
        ROS_INFO_STREAM("\033[1;32m current_speed_: " << current_speed_);
        ROS_INFO_STREAM("\033[1;32m yaw_error: " << -yaw_error * 180 / M_PI);
        ROS_INFO_STREAM("\033[1;32m cte angle: " << cross_track_control * 180 / M_PI);
        ROS_INFO_STREAM("\033[1;32m kp_from_control: " << kp_from_control);
        ROS_INFO_STREAM("\033[1;32m kp_dhead: " << kp_dhead);
        ROS_INFO_STREAM("\033[1;32m cross_track_error: " << cross_track_error);
        ROS_INFO_STREAM("\033[1;32m steering_angle_deg: " << steering_angle*180/M_PI);
        ROS_INFO("--------- Stanley Control ---------");
        lastPrintTime = currentTime;
    }
    
    return steering_angle;
}
