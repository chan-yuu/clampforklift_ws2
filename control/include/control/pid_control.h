/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2025-01-14 15:00:06
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2025-01-16 11:16:52
 * @FilePath: /undefined/home/cyun/forklift_sim_ws3/src/control/include/control/pid_control.h
 * @Description: 
 * 
 * Copyright (c) 2025 by Tianjin University, All Rights Reserved. 
 */
#ifndef PD_CONTROLLER_H
#define PD_CONTROLLER_H

#include <ros/ros.h>
#include <memory>
#include <chrono>

class PDController {
public:
    PDController(ros::NodeHandle& private_nh, double L = 3.8);

    double path_control(double CTE, double dHead, double kappa,double current_curvature, double now_speed, double dt = -1.0);

    ros::Time lastPrintTime;

private:
    double integral_CTE;  // 积分误差的初始值
    std::chrono::time_point<std::chrono::high_resolution_clock> last_time; // 上次调用时间的初始值
    double last_CTE;      // 上次的 CTE
    double last_dHead;    // 上次的 dHead
    double CTE_Kp_VL;        // CTE 的比例增益
    double CTE_Kd_VL;        // CTE 的微分增益
    double Head_Kp_VL;       // dHead 的比例增益
    double Head_Kd_VL;       // dHead 的微分增益
    double CTE_Kp_L;
    double CTE_Kd_L;
    double Head_Kp_L;
    double Head_Kd_L;
    double CTE_Kp_M;
    double CTE_Kd_M;
    double Head_Kp_M;
    double Head_Kd_M;
    double CTE_Kp_H;
    double CTE_Kd_H;
    double Head_Kp_H;
    double Head_Kd_H;
    double L;             // 车辆轴距
};

#endif // PD_CONTROLLER_H