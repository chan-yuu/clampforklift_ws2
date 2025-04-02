/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-09-14 17:58:22
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2024-11-02 12:02:13
 * @FilePath: /10.27/src/control/include/control/can_pub.h
 * @Description: CYUN LZQ
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#ifndef WIRE_CONTROL_PUB_H
#define WIRE_CONTROL_PUB_H

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>  // 修正的类型
#include <std_msgs/Int8.h>  // 修正的类型

#include <std_msgs/Bool.h>
#include <mutex>
#include <iostream>
#include <cmath>
#include <csignal>
#include <vector>
#include <std_msgs/Bool.h>
#include <car_interfaces/EmergencyStopTask.h>

struct CanMessage {
    uint32_t can_id;
    uint8_t can_dlc;
    uint8_t data[8];
};

class CanMessageHandler {
public:
    CanMessageHandler(ros::NodeHandle& nh);
    static void Quit(int signal);
    void Init(CanMessage& message, int can_id);
    void OperateCan();
    void OperateCan_Brake();

    void DecisionMaking();

    int SendCanMessage(int s, struct can_frame& frame, CanMessage& message);

    // void CameraCallback(const car_interfaces::PathSpeedCtrlInterface::ConstPtr& msg);
    // void PathSpeedCtrlInterfaceCallback(const car_interfaces::PathSpeedCtrlInterface::ConstPtr& msg_p);
    // void OriCallback(const car_interfaces::CarOriInterface::ConstPtr& msg);
    // void AebCallback(const car_interfaces::Decision::ConstPtr& msg);
    void ThrottleCallback(const std_msgs::Float64::ConstPtr& msg);
    void BrakeCallback(const std_msgs::Float64::ConstPtr& msg);
    void SteeringCallback(const std_msgs::Float64::ConstPtr& msg);
    void GearCallback(const std_msgs::UInt8::ConstPtr& msg);
    void StopCallback(const std_msgs::Bool::ConstPtr& msg);

    // clamp action sub
    void ClampCallback(const std_msgs::Float64::ConstPtr& msg);
    void UpdownCallback(const std_msgs::Float64::ConstPtr& msg);
    void UpdownmodeCallback(const std_msgs::Int8::ConstPtr& msg);

    void FyCallback(const std_msgs::Float64::ConstPtr& msg);
    void LateralCallback(const std_msgs::Float64::ConstPtr& msg);

    void ClampStateCallback(const std_msgs::Float64::ConstPtr& msg);
    void UpdownStateCallback(const std_msgs::Float64::ConstPtr& msg);
    void FyStateCallback(const std_msgs::Float64::ConstPtr& msg);
    void LateralStateCallback(const std_msgs::Float64::ConstPtr& msg);
    bool handle_emergency_stop_task(car_interfaces::EmergencyStopTask::Request &req,car_interfaces::EmergencyStopTask::Response &res);

    ros::ServiceServer emergency_stop_ser_;
    void init_flag();
    // bool has_forward;
    double brake_enable_from_aeb;
    double value_brake_from_aeb;
    CanMessage message210, message220,message221;
    int IPC_En;
    double Target_velocity;
    int Target_gear;
    double Target_steering_angle;
    double brake_enable;
    double value_brake;
    int FY;
    int press;
    double FY_Position;
    int UpDown;
    double UpDown_Position;
    double all_time;
    std::chrono::steady_clock::time_point last_time;
    bool first_call;
    double dt;
    
    int emergency_stop;

    int TurnLight;
    int work_mode;
    int clamp_width;
    double lateral_movement_position_mm;
    bool has_control_;
    bool clamp_, updown_, fy_, lateral_;
    bool has_throttle_, has_steering_;
    bool emergency_stop_msg_received;
    ros::Time last_steering_cmd_time_;

private:
    ros::NodeHandle nh_;
    // ros::Subscriber sub;
    // ros::Subscriber ori_sub;
    // ros::Subscriber aeb_sub;
    // ros::Subscriber camera_control_sub;

    ros::Subscriber throttle_sub_;
    ros::Subscriber brake_sub_;
    ros::Subscriber steering_sub_;
    ros::Subscriber gear_sub_;
    ros::Subscriber stop_sub_;
    ros::Subscriber updown_sub_, clamp_sub_, fy_sub_, lateral_sub_;
    ros::Subscriber updown_state_sub_, clamp_state_sub_, fy_state_sub_, lateral_state_sub_;
    ros::Subscriber emergency_stop_sub_;
    ros::Subscriber updown_mode_sub_;

    double wheel2steering;
    int num_cout;
    bool debug_mode;
    
    ros::Time lastPrintTime;
    bool is_print;

    // car_interfaces::PathSpeedCtrlInterface ControlContent;
    // car_interfaces::Decision AebContent;
    // car_interfaces::PathSpeedCtrlInterface CameraContent;

    std::mutex data_mutex; // Mutex for thread-safe data access
};

#endif // WIRE_CONTROL_PUB_H