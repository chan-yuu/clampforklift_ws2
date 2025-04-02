/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-10-17 11:47:28
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2025-01-18 01:23:04
 * @FilePath: /undefined/home/cyun/forklift_sim_ws3/src/control/src/pid_control.cpp
 * @Description: 
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#include "control/pid_control.h"
#include <iostream>
#include <cmath>
#include <chrono>

template<typename T>
constexpr const T& clamp(const T& v, const T& lo, const T& hi) {
    return (v < lo)? lo : (hi < v)? hi : v;
}

PDController::PDController(ros::NodeHandle& private_nh, double L)
    : integral_CTE(0.0), last_time(std::chrono::high_resolution_clock::now()), last_CTE(0.0), last_dHead(0.0), L(L) {
    // 从参数服务器读取参数
    private_nh.param("control/pd_controller/CTE_Kp_H", CTE_Kp_H, 1.0);
    private_nh.param("control/pd_controller/CTE_Kd_H", CTE_Kd_H, 0.1);
    private_nh.param("control/pd_controller/Head_Kp_H", Head_Kp_H, 1.0);
    private_nh.param("control/pd_controller/Head_Kd_H", Head_Kd_H, 0.1);

    private_nh.param("control/pd_controller/CTE_Kp_M", CTE_Kp_M, 1.0);
    private_nh.param("control/pd_controller/CTE_Kd_M", CTE_Kd_M, 0.1);
    private_nh.param("control/pd_controller/Head_Kp_M", Head_Kp_M, 1.0);
    private_nh.param("control/pd_controller/Head_Kd_M", Head_Kd_M, 0.1);


    private_nh.param("control/pd_controller/CTE_Kp_L", CTE_Kp_L, 1.0);
    private_nh.param("control/pd_controller/CTE_Kd_L", CTE_Kd_L, 0.1);
    private_nh.param("control/pd_controller/Head_Kp_L", Head_Kp_L, 1.0);
    private_nh.param("control/pd_controller/Head_Kd_L", Head_Kd_L, 0.1);

    private_nh.param("control/pd_controller/CTE_Kp_VL", CTE_Kp_VL, 1.0);
    private_nh.param("control/pd_controller/CTE_Kd_VL", CTE_Kd_VL, 0.1);
    private_nh.param("control/pd_controller/Head_Kp_VL", Head_Kp_VL, 1.0);
    private_nh.param("control/pd_controller/Head_Kd_VL", Head_Kd_VL, 0.1);

    lastPrintTime = ros::Time::now();
}

double PDController::path_control(double CTE, double dHead, double kappa,double current_curvature, double now_speed_plan, double dt) {
    if (dt <= 0) {
        auto current_time = std::chrono::high_resolution_clock::now();
        if (last_time == std::chrono::time_point<std::chrono::high_resolution_clock>()) {
            dt = 0.02;  // 初始调用间隔
        } else {
            dt = std::chrono::duration<double>(current_time - last_time).count();
        }
        last_time = current_time;
    }

    double diffCTE = 0.0;
    double diffdHead = 0.0;

    if (std::abs(CTE) <= 0.05){
        CTE = 0.0;
    }
    if (std::abs(dHead) <= 2.0){
        dHead = 0.0;
    }

    if (dt > 0) {
        diffCTE = (CTE - last_CTE) / dt;
        diffdHead = (dHead - last_dHead) / dt;
        last_CTE = CTE;
        last_dHead = dHead;
    }

    double kp_cte = 0.0;
    double kd_cte = 0.0;
    double kp_dHead = 0.0;
    double kd_dHead = 0.0;


    // 根据偏差计算控制量
    if (std::abs(CTE) <= 0.15) {
        kp_cte = CTE_Kp_VL;
        kd_cte = CTE_Kd_VL;
    } else if (std::abs(CTE) > 0.15 && std::abs(CTE) <= 0.3) {
        kp_cte = CTE_Kp_L;
        kd_cte = CTE_Kd_L;
    }
    else if (std::abs(CTE) > 0.3 && std::abs(CTE) <= 0.6) {
        kp_cte = CTE_Kp_M;
        kd_cte = CTE_Kd_M;
    }
     else if (std::abs(CTE) > 0.6) {
        kp_cte = CTE_Kp_H;
        kd_cte = CTE_Kd_H;
    }
    // 根据偏差计算控制量
    if (std::abs(dHead) <= 6) {
        kp_dHead = Head_Kp_VL;
        kd_dHead = Head_Kd_VL;
    } else if (std::abs(dHead) > 6 && std::abs(dHead) <= 12) {
        kp_dHead = Head_Kp_L;
        kd_dHead = Head_Kd_L;
    }
    else if (std::abs(dHead) > 12 && std::abs(dHead) <= 25) {
        kp_dHead = Head_Kp_M;
        kd_dHead = Head_Kd_M;
    }
     else if (std::abs(dHead) > 25) {
        kp_dHead = Head_Kp_H;
        kd_dHead = Head_Kd_H;
    }


    // 车轮转角或者方向盘转角都可
    // double wheelAngle = (CTE * kp_cte + diffCTE * kd_cte - dHead * kp_dHead + diffdHead * kd_dHead);
    double wheelAngle = (CTE * kp_cte + diffCTE * 0 - dHead * kp_dHead + diffdHead * 0);

    // double wheelAngle = -(CTE * CTE_Kp + diffCTE * CTE_Kd + dHead * Head_Kp + diffdHead * Head_Kd);

    // if (kappa <= 0.1 && now_speed_plan > 1) {
    //     // 限制转向角在 -20 度到 20 度
    //     wheelAngle = std::clamp(wheelAngle, -20.0, 20.0);
    // }
    // else if (now_speed_plan < 1) {
    //     // 转向角在 -70 度到 70 度
    //     wheelAngle = std::clamp(wheelAngle, -70.0, 70.0);
    // }

    wheelAngle = clamp(wheelAngle, -70.0, 70.0);

    // 打印调试信息
    // ROS_INFO_STREAM("\033[1;31m dt: " << dt << "\033[0m");
    // ROS_INFO_STREAM("\033[1;34m CTE: " << CTE << ", dHead: " << dHead << ", wheelAngle: " << wheelAngle << "\033[0m");
  // 定次数打印，防止刷屏
  ros::Time currentTime = ros::Time::now();
//   if ((currentTime - lastPrintTime).toSec() >= 1.0) 
  {
    ROS_INFO("----------------------- PD Control ----------------------");
    ROS_INFO_STREAM("\033[1;32m  CTE: " << CTE);
    ROS_INFO_STREAM("\033[1;32m  dHead: " << dHead);
    ROS_INFO_STREAM("\033[1;32m  CTE_Kp: " << kp_cte);
    ROS_INFO_STREAM("\033[1;32m  CTE_Kd: " << kd_cte);
    ROS_INFO_STREAM("\033[1;32m  Head_Kp: " << kp_dHead);
    ROS_INFO_STREAM("\033[1;32m  Head_Kd: " << kd_dHead);
    ROS_INFO_STREAM("\033[1;32m  pre_curvature: " << kappa);
    ROS_INFO_STREAM("\033[1;32m  current_curvature: " << current_curvature);
    ROS_INFO_STREAM("\033[1;32m  now_speed_plan: " << now_speed_plan);
    ROS_INFO_STREAM("\033[1;32m  diffCTE: " << diffCTE);
    ROS_INFO_STREAM("\033[1;32m  diffdHead: " << diffdHead);
    ROS_INFO_STREAM("\033[1;32m  wheelAngle: " << wheelAngle);
    ROS_INFO("----------------------- PD Control ----------------------");
    lastPrintTime = currentTime;
  }
    return wheelAngle*M_PI/180;
}