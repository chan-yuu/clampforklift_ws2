/*
 * @Author: CYUN && LZQ
 * @Date: 2024-09-15 13:39:13
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2024-10-13 17:44:59
 * @FilePath: /undefined/media/cyun/B2428C38428BFEFB/10.13/111/can_pub.cpp
 * @Description: 用于发布底层can消息
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#include "control/can_pub.h"

CanMessageHandler::CanMessageHandler(ros::NodeHandle& nh) : nh_(nh), brake_enable_from_aeb(0), value_brake_from_aeb(0),
      IPC_En(0), Target_velocity(0), Target_gear(0), Target_steering_angle(0),
      brake_enable(0), value_brake(0), FY(0), FY_Position(0), UpDown(0), UpDown_Position(0) ,TurnLight(0), work_mode(0),
      clamp_width(0), lateral_movement_position_mm(0.0)
{
    wheel2steering = nh.param("/can_pub/wheel2steering", 1.0);
    debug_mode = nh.param("/can_pub/openloop/debug_mode", true);
    
    //210
    nh_.param("/can_pub/openloop/IPC_En", IPC_En, 0);
    nh_.param("/can_pub/openloop/Target_velocity", Target_velocity, 0.0); //协议对应位置 写的油门开合度 范围0-65,精度0.1％ ？？
    nh_.param("/can_pub/openloop/Target_gear", Target_gear, 0);
    nh_.param("/can_pub/openloop/brake_enable", brake_enable, 0.0);
    nh_.param("/can_pub/openloop/value_brake", value_brake, 0.0);
    nh_.param("/can_pub/openloop/Target_steering_angle", Target_steering_angle, 0.0); // (-70°~+69.26°)，0°为对应中点位置，精度0.01，signal类型
    nh_.param("/can_pub/openloop/TurnLight", TurnLight, 0); //转向灯控制  0x00：归位；  0x01：左转向；  0x02：右转向
    
    //220
    nh_.param("/can_pub/openloop/FY", FY, 0);  // 俯仰位置控制模式 0:不控制，1:空载调整位置，2:抬起货物，3:放下货物
    nh_.param("/can_pub/openloop/FY_Position", FY_Position, 0.0); //俯仰位置控制  0-1670表示0-167mm,精度0.1mm
    nh_.param("/can_pub/openloop/UpDown", UpDown, 0);   //夹抱高度控制模式  0:不控制，1:空载调整高度，2:抬起货物，3:放下货物
    nh_.param("/can_pub/openloop/UpDown_Position", UpDown_Position, 0.0); //夹抱目标高度 0-2000mm,精度1mm
   
    //221
    nh_.param("/can_pub/openloop/work_mode", work_mode, 0);  //作业控制模式  0：不控制  1：夹抱   2：横移
    nh_.param("/can_pub/openloop/clamp_width", clamp_width, 0); //# 夹抱宽度 0-1352mm,精度1mm
    nh_.param("/can_pub/openloop/lateral_movement_position_mm", lateral_movement_position_mm, 0.0);  //横移位置 （最大最小值参考反馈，左正右负）,精度1mm,signed类型

    // sub = nh_.subscribe<car_interfaces::PathSpeedCtrlInterface>("path_speed_tracking_data", 10, &CanMessageHandler::PathSpeedCtrlInterfaceCallback, this);
    // ori_sub = nh_.subscribe<car_interfaces::CarOriInterface>("/car_ori_data", 10, &CanMessageHandler::OriCallback, this);
    // aeb_sub = nh_.subscribe<car_interfaces::Decision>("/emergency_brake_from_decision", 10, &CanMessageHandler::AebCallback, this);
    // // 直接通过相机的双环pid：
    // camera_control_sub = nh_.subscribe<car_interfaces::PathSpeedCtrlInterface>("/camera_control", 10, &CanMessageHandler::CameraCallback, this);

    // 订阅的控制消息：
    throttle_sub_ = nh_.subscribe<std_msgs::Float64>("/throttle_cmd", 10, &CanMessageHandler::ThrottleCallback, this);
    brake_sub_ = nh_.subscribe<std_msgs::Float64>("/brake_cmd", 10, &CanMessageHandler::BrakeCallback, this);
    steering_sub_ = nh_.subscribe<std_msgs::Float64>("/steering_cmd", 10, &CanMessageHandler::SteeringCallback, this);
    gear_sub_ = nh_.subscribe<std_msgs::UInt8>("/gear_cmd", 10, &CanMessageHandler::GearCallback, this);
    stop_sub_ = nh_.subscribe<std_msgs::Bool>("/stop", 10, &CanMessageHandler::StopCallback, this);
    has_control_ = false;
    
    updown_sub_ = nh_.subscribe<std_msgs::Float64>("updown_cmd", 10, &CanMessageHandler::UpdownCallback, this);
    clamp_sub_ = nh_.subscribe<std_msgs::Float64>("clamp_cmd", 10, &CanMessageHandler::ClampCallback, this);
    fy_sub_ = nh_.subscribe<std_msgs::Float64>("fy_cmd", 10, &CanMessageHandler::FyCallback, this);
    lateral_sub_ = nh_.subscribe<std_msgs::Float64>("lateral_cmd", 10, &CanMessageHandler::LateralCallback, this);
    clamp_ = false;
    updown_ = false;
    fy_ = false;
    lateral_ = false;
    num_cout = 0;
    
    // 反馈的状态
    updown_state_sub_ = nh_.subscribe<std_msgs::Float64>("updown_state", 10, &CanMessageHandler::UpdownStateCallback, this);
    clamp_state_sub_ = nh_.subscribe<std_msgs::Float64>("clamp_state", 10, &CanMessageHandler::ClampStateCallback, this);
    fy_state_sub_ = nh_.subscribe<std_msgs::Float64>("fy_state", 10, &CanMessageHandler::FyStateCallback, this);
    lateral_state_sub_ = nh_.subscribe<std_msgs::Float64>("lateral_state", 10, &CanMessageHandler::LateralStateCallback, this);
}

void CanMessageHandler::Init(CanMessage& message, int can_id) {
    message.can_id = can_id;
    message.can_dlc = 8;
    std::memset(message.data, 0, sizeof(message.data));
}

void CanMessageHandler::init_flag() {
    FY = 0;
    UpDown = 0;
    work_mode = 0;
}

void CanMessageHandler::OperateCan() {
    // test MPC:
    // Target_gear = 3;
    
    // 如果动作的话题不发了那线控也不发了 动作的发送频率必须足够高才能让标志位一直为true  推荐是100hz
    // 只要值不是0就认为有动作 现阶段认为每次只能做一个动作
    if (UpDown_Position!=0) 
    {
        UpDown = 1;
        FY = 0;
        work_mode = 0;
        // updown_ = false;
    }
    if (FY_Position!=0) 
    {
        FY = 1;
        UpDown = 0;
        work_mode = 0;
        // fy_ = false;
    }
    if (clamp_width!=0) 
    {
        work_mode = 1;
        UpDown = 0;
        FY = 0;
        // handler.clamp_ = false;
    }
    if (lateral_movement_position_mm!=0) {
        work_mode = 2;
        UpDown = 0;
        FY = 0;
        // lateral_ = false;
    }
    // else
    // {
    //     work_mode = 0;
    //     UpDown = 0;
    //     FY = 0;
    // }

    // ROS_INFO_STREAM("\033[1;31m IPC_En : " << IPC_En);
    // 控制打印频率，防止刷屏：

    ros::Time currentTime = ros::Time::now();
    if ((currentTime - lastPrintTime).toSec() >= 1.0) {
        ROS_INFO("----------------------- CAN PUB ----------------------");
        ROS_INFO_STREAM("\033[1;31m Target_velocity : " << Target_velocity);
        ROS_INFO_STREAM("\033[1;32m Target_gear " << Target_gear);
        ROS_INFO_STREAM("\033[1;32m Target_steering_angle: " << Target_steering_angle);
        ROS_INFO_STREAM("\033[1;32m value_brake: " << value_brake);
        ROS_INFO_STREAM("\033[1;32m brake_enable: " << brake_enable);
        ROS_INFO_STREAM("\033[1;32m ***************** ");
        // ROS_INFO_STREAM("\033[1;32m UpDown: " << UpDown);
        // ROS_INFO_STREAM("\033[1;32m FY: " << FY);
        ROS_INFO_STREAM("\033[1;32m UpDown_Position: " << UpDown_Position);
        ROS_INFO_STREAM("\033[1;32m FY_Position: " << FY_Position);
        ROS_INFO_STREAM("\033[1;32m work_mode : " << work_mode);
        ROS_INFO_STREAM("\033[1;32m lateral: " << lateral_movement_position_mm);
        ROS_INFO_STREAM("\033[1;32m clamp: " << clamp_width);
        ROS_INFO("----------------------- CAN PUB ----------------------");
        lastPrintTime = currentTime;
    }
    
    IPC_En = 1; // 默认发1就行
    message210.data[0] &= ~(1 << 5);
    if (IPC_En != 0) {
        message210.data[0] |= (1 << 5);
    }

    uint8_t bit7 = static_cast<uint8_t>(Target_gear) & 1;
    uint8_t bit6 = static_cast<uint8_t>(Target_gear >> 1) & 1;
    message210.data[0] &= ~((1 << 6) | (1 << 7));   //将message210.data[0]的第6位和第7位清零，保留其他位。
    message210.data[0] |= (bit6 << 7) | (bit7 << 6);  //将message210.data[0]的第6位和第7位赋值为bit6和bit7

    uint16_t target_velocity_int = static_cast<uint16_t>(Target_velocity * 1000);
    message210.data[1] = static_cast<uint8_t>(target_velocity_int & 0xFF);
    message210.data[2] = static_cast<uint8_t>((target_velocity_int >> 8) & 0xFF);

    int16_t target_steering_angle_int = static_cast<int16_t>(Target_steering_angle * 100);
    message210.data[4] = static_cast<int8_t>(target_steering_angle_int & 0xFF);
    message210.data[5] = static_cast<int8_t>((target_steering_angle_int >> 8) & 0xFF);

    // message210.data[3] = static_cast<uint8_t>(UpDown_Speed);
    message210.data[6] &= ~1;
    message210.data[6] |= static_cast<uint8_t>(brake_enable);

    uint8_t target_travel_int = static_cast<uint8_t>(value_brake * 10);
    message210.data[6] &= ~(0x7F << 1);
    message210.data[6] |= (target_travel_int << 1);

    uint8_t bit1 = static_cast<uint8_t>(TurnLight) & 1;
    uint8_t bit2 = static_cast<uint8_t>(TurnLight >> 1 ) & 1;
    message210.data[7] &= ~((1 << 1) | (1 << 2));   
    message210.data[7] |= (bit2 << 2) | (bit1 << 1); 

    //220
    message220.data[3] = static_cast<uint8_t>(FY);
    uint16_t fy_position_int = static_cast<uint16_t>(FY_Position);
    message220.data[4] = static_cast<uint8_t>(fy_position_int & 0xFF);
    message220.data[5] = static_cast<uint8_t>((fy_position_int >> 8) & 0xFF);

    message220.data[0] = static_cast<uint8_t>(UpDown);
    uint16_t updown_position_int = static_cast<uint16_t>(UpDown_Position);
    message220.data[1] = static_cast<uint8_t>(updown_position_int & 0xFF);
    message220.data[2] = static_cast<uint8_t>((updown_position_int >> 8) & 0xFF);


    //221
    message221.data[0] = static_cast<uint8_t>(work_mode);
    // std::cout<<lateral_movement_position_mm<<std::endl;
    uint16_t lateral_target_position = static_cast<int16_t>(lateral_movement_position_mm);
    message221.data[3] = static_cast<uint8_t>(lateral_target_position & 0xFF);
    message221.data[4] = static_cast<uint8_t>((lateral_target_position >> 8) & 0xFF);

    uint16_t clamp_target_open_position = static_cast<uint16_t>(clamp_width);
    message221.data[1] = static_cast<int8_t>(clamp_target_open_position & 0xFF);
    message221.data[2] = static_cast<int8_t>((clamp_target_open_position >> 8) & 0xFF);

}


void CanMessageHandler::OperateCan_Brake() {
    // ROS_INFO_STREAM("\033[1;31m IPC_En : " << IPC_En);
    // ROS_INFO_STREAM("\033[1;31m Target_velocity : " << Target_velocity);
    // ROS_INFO_STREAM("\033[1;32m TurnLight : " << TurnLight);
    // ROS_INFO_STREAM("\033[1;32m FY_Position : " << FY_Position);
    // ROS_INFO_STREAM("\033[1;32m UpDown_Position : " << UpDown_Position);
    // ROS_INFO_STREAM("\033[1;32m Target_gear " << Target_gear);
    // ROS_INFO_STREAM("\033[1;32m Target_steering_angle: " << Target_steering_angle);
    // ROS_INFO_STREAM("\033[1;32m UpDown: " << UpDown);
    // ROS_INFO_STREAM("\033[1;32m FY: " << FY);
    // ROS_INFO_STREAM("\033[1;32m work_mode : " << work_mode);
    // ROS_INFO_STREAM("\033[1;32m lateral_movement_position_mm: " << lateral_movement_position_mm);
    // ROS_INFO_STREAM("\033[1;32m clamp_width: " <<   clamp_width);

    double value_brake_;
    double brake_enable_;
    // 行车3s过后启动刹车。
    if (all_time>=3)
    {
        value_brake_ = value_brake;
        brake_enable_ = brake_enable;
    }
    else
    {
        value_brake_ = 0;
        brake_enable_ = 0;
    }
    ROS_INFO_STREAM("\033[1;32m value_brake: " << value_brake_);
    ROS_INFO_STREAM("\033[1;32m brake_enable: " <<   brake_enable_);

    if (first_call) 
    {
        dt = 0.01; // 初始调用间隔
        first_call = false;
    }
    else
    {
    auto current_time = std::chrono::steady_clock::now();
        dt = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time).count() / 1000.0;
        std::cout<<"dt: "<< dt<<std::endl;
        all_time += 0.01;
    }
    last_time = std::chrono::steady_clock::now();
    std::cout<<"process_time: "<<all_time<<std::endl;

    message210.data[0] &= ~(1 << 5);
    if (IPC_En != 0) {
        message210.data[0] |= (1 << 5);
    }

    uint8_t bit7 = static_cast<uint8_t>(Target_gear) & 1;
    uint8_t bit6 = static_cast<uint8_t>(Target_gear >> 1) & 1;
    message210.data[0] &= ~((1 << 6) | (1 << 7));   //将message210.data[0]的第6位和第7位清零，保留其他位。
    message210.data[0] |= (bit6 << 7) | (bit7 << 6);  //将message210.data[0]的第6位和第7位赋值为bit6和bit7

    uint16_t target_velocity_int = static_cast<uint16_t>(Target_velocity * 1000);
    message210.data[1] = static_cast<uint8_t>(target_velocity_int & 0xFF);
    message210.data[2] = static_cast<uint8_t>((target_velocity_int >> 8) & 0xFF);

    int16_t target_steering_angle_int = static_cast<int16_t>(Target_steering_angle * 100);
    message210.data[4] = static_cast<int8_t>(target_steering_angle_int & 0xFF);
    message210.data[5] = static_cast<int8_t>((target_steering_angle_int >> 8) & 0xFF);

    // message210.data[3] = static_cast<uint8_t>(UpDown_Speed);
    message210.data[6] &= ~1;
    message210.data[6] |= static_cast<uint8_t>(brake_enable_);

    uint8_t target_travel_int = static_cast<uint8_t>(value_brake_ * 10);
    message210.data[6] &= ~(0x7F << 1);
    message210.data[6] |= (target_travel_int << 1);

    uint8_t bit1 = static_cast<uint8_t>(TurnLight) & 1;
    uint8_t bit2 = static_cast<uint8_t>(TurnLight >> 1 ) & 1;
    message210.data[7] &= ~((1 << 1) | (1 << 2));   
    message210.data[7] |= (bit2 << 2) | (bit1 << 1); 

    //220
    message220.data[3] = static_cast<uint8_t>(FY);
    uint16_t fy_position_int = static_cast<uint16_t>(FY_Position);
    message220.data[4] = static_cast<uint8_t>(fy_position_int & 0xFF);
    message220.data[5] = static_cast<uint8_t>((fy_position_int >> 8) & 0xFF);

    message220.data[0] = static_cast<uint8_t>(UpDown);
    uint16_t updown_position_int = static_cast<uint16_t>(UpDown_Position);
    message220.data[1] = static_cast<uint8_t>(updown_position_int & 0xFF);
    message220.data[2] = static_cast<uint8_t>((updown_position_int >> 8) & 0xFF);

    //221
    message221.data[0] = static_cast<uint8_t>(work_mode);
    // std::cout<<lateral_movement_position_mm<<std::endl;
    uint16_t lateral_target_position = static_cast<int16_t>(lateral_movement_position_mm);
    message221.data[3] = static_cast<uint8_t>(lateral_target_position & 0xFF);
    message221.data[4] = static_cast<uint8_t>((lateral_target_position >> 8) & 0xFF);

    uint16_t clamp_target_open_position = static_cast<uint16_t>(clamp_width);
    message221.data[1] = static_cast<int8_t>(clamp_target_open_position & 0xFF);
    message221.data[2] = static_cast<int8_t>((clamp_target_open_position >> 8) & 0xFF);
}

// void CanMessageHandler::DecisionMaking() {
//     IPC_En = 1;
//     Target_velocity = ControlContent.Target_velocity;
//     Target_gear = ControlContent.gear;
//     Target_steering_angle = ControlContent.Target_steering_angle;
//     if (Target_gear == 1) {
//         Target_steering_angle = -ControlContent.Target_steering_angle;
//     }
//     brake_enable = ControlContent.brake_enable;
//     value_brake = ControlContent.value_brake;
//     FY = ControlContent.FY;
//     FY_Position = ControlContent.FY_Position;
//     UpDown = ControlContent.UpDown;
//     UpDown_Position = ControlContent.UpDown_Position;

//     if (AebContent.brake_enable == 1) {
//         brake_enable = 1;
//         value_brake = AebContent.value_brake;
//         Target_velocity = 0;
//     }

//     if (CameraContent.flag_camera == 1) {
//         Target_velocity = CameraContent.Target_velocity;
//         Target_gear = 3;
//         Target_steering_angle = CameraContent.Target_steering_angle;
//     }
// }

void CanMessageHandler::ThrottleCallback(const std_msgs::Float64::ConstPtr& msg)
{
    Target_velocity = msg->data;
    has_control_ = true;
}

void CanMessageHandler::BrakeCallback(const std_msgs::Float64::ConstPtr& msg)
{
    value_brake = msg->data;
    brake_enable = value_brake > 0 ? 1 : 0;
}

void CanMessageHandler::SteeringCallback(const std_msgs::Float64::ConstPtr& msg)
{
    if(Target_gear==3){
        Target_steering_angle = msg->data * wheel2steering * 180 / M_PI; // 车轮转角比率 and 弧度和角度转换
    }
    else{
        Target_steering_angle = msg->data * -wheel2steering * 180 / M_PI; // 车轮转角比率 and 弧度和角度转换
    }
    // Target_steering_angle = msg->data * wheel2steering * 180 / M_PI; // 车轮转角比率 and 弧度和角度转换
    // std::cout<<"Target_steering_angle: "<<Target_steering_angle<<"\n"<<wheel2steering<<std::endl;
}

void CanMessageHandler::GearCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    Target_gear = msg->data;
    // std::cout<<"target gear:" << Target_gear<<std::endl;
}

void CanMessageHandler::ClampCallback(const std_msgs::Float64::ConstPtr& msg)
{
    clamp_width = msg->data;
    clamp_ = true;
}

void CanMessageHandler::UpdownCallback(const std_msgs::Float64::ConstPtr& msg)
{
    UpDown_Position = msg->data;
    updown_ = true;
}

void CanMessageHandler::FyCallback(const std_msgs::Float64::ConstPtr& msg)
{
    FY_Position = msg->data;
    fy_ = true;
}

void CanMessageHandler::LateralCallback(const std_msgs::Float64::ConstPtr& msg)
{
    lateral_movement_position_mm = msg->data;
    lateral_ = true;
}

void CanMessageHandler::ClampStateCallback(const std_msgs::Float64::ConstPtr& msg)
{
    int i = 1;
    // Target_velocity = msg->data;
}

void CanMessageHandler::UpdownStateCallback(const std_msgs::Float64::ConstPtr& msg)
{
    int i = 1;
    // value_brake = msg->data;
    // brake_enable = value_brake > 0 ? 1 : 0;
}

void CanMessageHandler::FyStateCallback(const std_msgs::Float64::ConstPtr& msg)
{
    int i = 1;
    // Target_steering_angle = msg->data * wheel2steering * 180 / M_PI;
}

void CanMessageHandler::LateralStateCallback(const std_msgs::Float64::ConstPtr& msg)
{
    int i = 1;
    // Target_gear = msg->data;
}

void CanMessageHandler::StopCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data) {
        IPC_En = 0;  // 停止控制
        Target_velocity = 0;  // 停止
        value_brake = 1.0;    // 最大刹车
        brake_enable = 1;     // 启用刹车
    }
}


int CanMessageHandler::SendCanMessage(int s, struct can_frame& frame, CanMessage& message) {
    frame.can_id = (int)message.can_id;
    frame.can_dlc = (int)message.can_dlc;
    std::memcpy(frame.data, message.data, sizeof(message.data));

    if (write(s, &frame, sizeof(struct can_frame)) < 0) {
        perror("Error while sending CAN frame");
        return -3;
    }
    return 0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "can_message_handler_node");
    ros::NodeHandle nh;
    CanMessageHandler handler(nh);

    std::string ifname;
    int loopRate;
    int openloop;
    bool debug_mode;
    int s;
    struct sockaddr_can addr;
    struct can_frame frame;
    struct ifreq ifr;

    ifname = nh.param<std::string>("/can_pub/ifname", "can0");

    loopRate = nh.param("/can_pub/Rate", 100);
    openloop = nh.param("/can_pub/openloop", 0);
    debug_mode = nh.param("/can_pub/debug_mode", false);

    ros::Rate loop_rate(loopRate);

    const char* ifname_cstr = ifname.c_str();

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening socket");
        return -1;
    }

    strcpy(ifr.ifr_name, ifname_cstr);
    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error in socket bind");
        return -2;
    }

    handler.Init(handler.message210, 0x210);
    handler.Init(handler.message220, 0x220);
    handler.Init(handler.message221, 0x221);

    while (ros::ok()) {
        // std::cout<<"openloop: "<<openloop<<std::endl;
        if (openloop==1)
        {
            // ros::spinOnce();
            // 开启了openloop后不应该能够订阅信息
            handler.OperateCan_Brake();  // 测试刹车情况
            // std::cout<<"mes"<<int(handler.message210.data[0])<<std::endl;
            handler.SendCanMessage(s, frame, handler.message210);
            handler.SendCanMessage(s, frame, handler.message220);
            handler.SendCanMessage(s, frame, handler.message221);
        }

        else
        {
            ros::spinOnce(); // 各种话题中直接读取can消息
            handler.OperateCan(); // 将控制信息整合到can中

            // 高度宽度信息应该是一直都发送
            handler.SendCanMessage(s, frame, handler.message220);
            handler.SendCanMessage(s, frame, handler.message221);
            // handler.SendCanMessage(s, frame, handler.message210);
            // 发布这些can数据：
            // if(debug_mode) std::cout<<"handler.has_control_"<<handler.has_control_<<std::endl;
            if (handler.has_control_) // 控制如果不发了那也不能进发送  同时就算发，控制也要有归零机制，防止作业过程出现问题
            {
                // ROS_INFO("sub control success!!!");
                handler.SendCanMessage(s, frame, handler.message210);
                handler.has_control_ = false;
            }
            else // 如果没有控制信息接收到，但是又需要动作
            {
                // 也发，但是发送的内容为0 （因为无人使能在210！！！！）
                // 将 message210 的所有字节都设置为 0
                for (size_t i = 0; i < sizeof(handler.message210.data); ++i) 
                {
                    handler.message210.data[i] = 0;
                }
                // 将 IPC_En 的位置赋 1
                handler.message210.data[0] |= (1 << 5);
                // 发送message210
                handler.SendCanMessage(s, frame, handler.message210);
            }
            // 如果没有订阅的内容过后就需要将标志位置0了：
            handler.init_flag();
        }
        loop_rate.sleep();
    }
    close(s);
    return 0;
}
