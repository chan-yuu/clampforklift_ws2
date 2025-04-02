/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-09-23 14:31:27
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2024-10-16 15:43:57
 * @FilePath: /undefined/home/cyun/forklift_sim_ws/src/control/control/src/stanley_control.cpp
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
    private_nh.param("/control/stanley_controller/kp", kp_, 0.5);  // 默认值1.0
    private_nh.param("/control/stanley_controller/max_steering_angle", max_steering_angle_, 1.12);  // 最大转向角
    
    private_nh.param("/control/stanley_controller/kp_dhead", kp_dhead, 1.0);  // 最大转向角
    private_nh.param("/control/stanley_controller/kp_rear_cte", kp_rear_cte, 0.2);  // 最大转向角
    private_nh.param("/control/stanley_controller/end_dis_fix", end_dis_fix, 1.6);  // 最大转向角)
    private_nh.param("/control/stanley_controller/wheelbase", wheelbase_, 1.77);  // 车辆轴距
    lastPrintTime = ros::Time::now();
}

double StanleyController::computeSteeringAngle(double yaw_error, double cross_track_error,double current_speed_, int controller_location)
{
    static int count = 0; // 定义一个静态计数器
    const int print_interval = 100; // 设置打印间隔

    // Stanley 控制器计算公式
    double heading_control = -yaw_error; 
    double current_speed_min = 0.10;
    double current_speed_reindex = std::max(current_speed_, current_speed_min);

    double cross_track_control = -std::atan2(kp_ * cross_track_error, current_speed_reindex);

    // 仿真中是负的 cte 和 dhead作用
    double steering_angle = (heading_control + cross_track_control);    

    // double cross_track_control_now;
    // // TODO 测试屁股能否摆正
    // if(end_dis<=end_dis_fix){
    //     // std::cout<<"test: "<<end_dis<<"*********"<<std::endl;
    //     // 到达终点后不允许再调整
    //     // if(cross_track_error==0) cross_track_error=0;
    //     // // 会导致原始的cte增大，所以原始：
    //     // if(abs(cross_track_error)>=0.15) rear_cte=0;
    //     cross_track_error = 0.0; // 用后面的cte和前面的dhead来调整位姿
        
    //     // 仿真中是负的 cte 和 dhead作用
    //     cross_track_control_now = -std::atan2(kp_ * (cross_track_error-rear_cte*kp_rear_cte), current_speed_reindex);
    //     steering_angle = (heading_control*kp_dhead + cross_track_control_now);
    // }

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
        ROS_INFO("-------------------- Stanley Control -------------------"); 
        ROS_INFO_STREAM("\033[1;32m current_speed_: " << current_speed_);
        ROS_INFO_STREAM("\033[1;32m yaw_error: " << -yaw_error * 180 / M_PI);
        ROS_INFO_STREAM("\033[1;32m cte angle: " << cross_track_control * 180 / M_PI);
        ROS_INFO_STREAM("\033[1;32m kp: " << kp_);
        ROS_INFO_STREAM("\033[1;32m kp_dhead: " << kp_dhead);
        ROS_INFO_STREAM("\033[1;32m CTE: " << cross_track_error);

        ROS_INFO_STREAM("\033[1;32m steering_angle_deg: " << steering_angle*180/M_PI);
        ROS_INFO("-------------------- Stanley Control -------------------");
        lastPrintTime = currentTime;
    }
    
    return steering_angle;
}
