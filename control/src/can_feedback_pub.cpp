#include "control/can_feedback.h"
#include <cmath>
#include <ros/ros.h>


int change_map_UpDown;
double change_map_UpDown_Position;

CanFeedBack::CanFeedBack(ros::NodeHandle& nodehandle): nh_(nodehandle) {

    if(!Init_CAN(nh_))
    {
        ROS_INFO_STREAM("\033[1;31m Init_CAN False : ");
    }
    else
    {
        ROS_INFO_STREAM("\033[1;32m Init_CAN True : ");
    }
}


bool CanFeedBack::Init_CAN(ros::NodeHandle nh_)
{
    flag_201 = false;
    flag_202 = false;
    flag_203 = false;
    flag_204 = false;
    flag_205 = false;
    flag_206 = false;
    struct sockaddr_can addr;
    struct ifreq ifr;

    twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/twist", 1);
    gear_state_pub_ = nh_.advertise<std_msgs::Int8>("/gear_state", 1);
    ipc_state_pub_ = nh_.advertise<std_msgs::Int8>("/ipc_state", 1);
    eposts_state_pub = nh_.advertise<std_msgs::Int8>("/eposts_state", 1);

    throttle_state_pub_ = nh_.advertise<std_msgs::Float64>("/throttle_state", 1);
    brake_state_pub_ = nh_.advertise<std_msgs::Float64>("/brake_state", 1);
    soc_state_pub_ = nh_.advertise<std_msgs::Float64>("/soc_state", 1);
    steering_state_pub_ = nh_.advertise<std_msgs::Float64>("/steering_state", 1);
    
    // 夹抱动作相关
    // 横移位置
    lateral_state_pub_ = nh_.advertise<std_msgs::Float64>("/lateral_state", 1);
    // 夹抱位置
    clamp_state_pub_ = nh_.advertise<std_msgs::Float64>("clamp_state", 1);
    // 升降
    updown_state_pub_ = nh_.advertise<std_msgs::Float64>("updown_state", 1);
    // 俯仰
    fy_state_pub_ = nh_.advertise<std_msgs::Float64>("fy_state", 1);
    // 使能
    ifname = nh_.param<std::string>("/can_feedback/ifname", "can1");
    debug_mode = nh_.param("/can_feedback/debug_mode", true);
    is_print = nh_.param("/control/control_main_node/is_print", false);

    // 初始化：
    num_cout = 0;
    lateral_msg.data = 0.0;
    clamp_msg.data = 0.0;
    fy_msg.data = 0.0;
    updown_msg.data = 0.0;
    brake_msg.data = 0.0;
    soc_state_msg.data = -1.0;
    gear_msg.data = 0;
    steering_msg.data = 0.0;
    throttle_msg.data = 0.0;

    geometry_msgs::TwistStamped twist_msg;
    twist_msg.twist.linear.x = 0.0;
    twist_msg.twist.linear.y = 0.0;
    twist_msg.twist.linear.z = 0.0;
    twist_msg.twist.angular.x = 0.0;
    twist_msg.twist.angular.y = 0.0;
    twist_msg.twist.angular.z = 0.0;

    gear_cout = 0;

    const char *ifname_cstr = ifname.c_str();

    // 检查CAN口
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        perror("Error while opening socket");
        return -1;
    }
    // 创建连接
    strcpy(ifr.ifr_name, ifname.c_str());
    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("Error in socket bind");
        return -2;
    }
    return true;
}

CanMessage201 CanFeedBack::parseCanMessage201(const struct can_frame &frame)
{
    CanMessage201 msg;

    int16_t Angle_raw = ((uint8_t)frame.data[2] << 8) | (uint8_t)frame.data[1];
    msg.Angle = static_cast<double>(Angle_raw) * 1e-2;

    uint16_t Car_Speed_raw = ((uint8_t)frame.data[5] << 8) | (uint8_t)frame.data[4];
    msg.Car_Speed = static_cast<double>(Car_Speed_raw) * 1e-3;

    uint8_t EPOSts_raw = (uint8_t)(frame.data[0] >> 4) & 0x03;
    msg.EPOSts = static_cast<double>(EPOSts_raw);

    uint8_t Gear_raw = ((uint8_t)(frame.data[0] >> 2) & 0x03);
    msg.Gear = static_cast<double>(Gear_raw);

    uint8_t IPC_raw = (uint8_t)frame.data[0] & 0x03;
    msg.IPC = static_cast<int>(IPC_raw);

    return msg;
}

CanMessage202 CanFeedBack::parseCanMessage202(const struct can_frame &frame)
{
    CanMessage202 msg;
    int32_t Mileage_raw = ((uint8_t)frame.data[7] << 24) |
                          ((uint8_t)frame.data[6] << 16) |
                          ((uint8_t)frame.data[5] << 8) |
                          frame.data[4];
    // int16_t x = ((uint8_t)frame.data[0] << 8) | frame.data[1];
    // int16_t y = ((uint8_t)frame.data[2] << 8) | frame.data[3];
    // int16_t z = ((uint8_t)frame.data[5] << 8) | frame.data[4];
    // int16_t yaw = ((uint8_t)frame.data[7] << 8) | frame.data[6];
    int8_t Fault1=(uint8_t)frame.data[0];
    int8_t Fault2=(uint8_t)frame.data[1];
    int8_t Fault3=(uint8_t)frame.data[2];
    int8_t Fault4=(uint8_t)frame.data[3];
    msg.Fault1= static_cast<double>(Fault1);  //x 
    msg.Fault2= static_cast<double>(Fault2);  //y
    msg.Fault3= static_cast<double>(Fault3); //z
    msg.Fault4= static_cast<double>(Fault4); //yaw

    //msg.Mileage = static_cast<double>(Mileage_raw) * 1e-1;
    return msg;
}

CanMessage203 CanFeedBack::parseCanMessage203(const struct can_frame &frame)
{
    CanMessage203 msg;

    int16_t SOC = (uint8_t)frame.data[6];
    msg.SOC = static_cast<double>(SOC);

    int16_t Brake_Pressure = ((uint8_t)frame.data[1] << 8) | frame.data[0];
    msg.Brake_Pressure = static_cast<double>(Brake_Pressure) * 0.05; //制动压力采样
    return msg; 
}


CanMessage204 CanFeedBack::parseCanMessage204(const struct can_frame &frame)
{
    CanMessage204 msg;

    int32_t LX_Hight_raw = ((uint8_t)frame.data[4] << 24) |
                           ((uint8_t)frame.data[3] << 16) |
                           ((uint8_t)frame.data[2] << 8) |
                           frame.data[1];

    msg.LX_Hight = static_cast<double>(LX_Hight_raw);

    int16_t Pitching_raw = ((uint8_t)frame.data[6] << 8) |
                           frame.data[5];
    msg.Pitching = static_cast<double>(Pitching_raw);  //俯仰传感器

    return msg;
}


CanMessage205 CanFeedBack::parseCanMessage205(const struct can_frame &frame)
{
    CanMessage205 msg;

    int32_t now_car_speed = ((uint8_t)frame.data[1] << 8) |frame.data[0]; //0-1000 表示0-100km/h,精度0.1
    msg.now_car_speed = static_cast<double>(now_car_speed)*0.1;
    return msg;
}

CanMessage206 CanFeedBack::parseCanMessage206(const struct can_frame &frame)  //对比叉车改了msg
{
    CanMessage206 msg;

    int16_t clamp_width = ((uint8_t)frame.data[1] << 8) |frame.data[0]; //夹抱宽度 有效范围：0-1352 ，精度1mm，Unsignal类型   
    msg.current_clamp_width = static_cast<double>(clamp_width);

    int16_t lateral_movement_left = ((uint8_t)frame.data[5] << 8) |frame.data[4];
    msg.lateral_movement_left = static_cast<double>(lateral_movement_left);   //横移左侧可移动的距离 0-1000 ，精度1mm

    int16_t lateral_movement_right = ((uint8_t)frame.data[7] << 8) |frame.data[6];
    msg.lateral_movement_right = static_cast<double>(lateral_movement_right);   //横移右侧可移动的距离  0-1000 ，精度1mm

    int16_t lateral_movement_position = ((uint8_t)frame.data[3] << 8) |frame.data[2]; ////当前横移位置 有效范围：-1000-1000 ，精度1mm    
    msg.current_lateral_movement_position = static_cast<double>(lateral_movement_position);

    return msg;
}

void CanFeedBack::DecodeCanData()
{
    // 逐一解析并存储。
    if (read(s, &frame, sizeof(struct can_frame)) > 0)
    {
        // std::cout<<"frame.can_id: "<<frame.can_id<<std::endl;
        if (frame.can_id == 0x201)
        {
            CanMessage201 parsed_msg = parseCanMessage201(frame);
            steering_msg.data = parsed_msg.Angle / 180 * M_PI; // 反馈的原始是角度值，现将反馈转为弧度制
            steering_state_pub_.publish(steering_msg);
            std::cout<<"angle now: "<<steering_msg.data<<std::endl;
            twist_msg.header.stamp = ros::Time::now();
            twist_msg.header.frame_id = "front_axle";
            twist_msg.twist.linear.x = parsed_msg.Car_Speed;
            twist_msg.twist.angular.z = 0.0; 
            twist_pub_.publish(twist_msg);
            throttle_msg.data = parsed_msg.Car_Speed;
            throttle_state_pub_.publish(throttle_msg);
            gear_msg.data = parsed_msg.Gear;
            gear_cout = parsed_msg.Gear; // BUG 
            gear_state_pub_.publish(gear_msg);

            // 添加了ipc的状态
            ipc_msg.data = parsed_msg.IPC;
            ipc_state_pub_.publish(ipc_msg);

            // 急停
            eposts_msg.data = parsed_msg.EPOSts;
            eposts_state_pub.publish(eposts_msg);
            

            flag_201 = true;
        }

        else if (frame.can_id == 0x202)
        {
            CanMessage202 parsed_msg = parseCanMessage202(frame);
            flag_202 = true;
        }

        else if (frame.can_id == 0x203)
        {
            CanMessage203 parsed_msg = parseCanMessage203(frame);
            brake_msg.data = parsed_msg.Brake_Pressure;
            brake_state_pub_.publish(brake_msg);
            soc_state_msg.data = parsed_msg.SOC;
            soc_state_pub_.publish(soc_state_msg);
            flag_203 = true;
        }

        else if (frame.can_id == 0x204)
        {
            CanMessage204 parsed_msg = parseCanMessage204(frame);
            updown_msg.data = parsed_msg.LX_Hight;
            updown_state_pub_.publish(updown_msg);
            fy_msg.data = parsed_msg.Pitching;
            fy_state_pub_.publish(fy_msg);

            flag_204 = true;
        }
        else if (frame.can_id == 0x205)
        {
            CanMessage205 parsed_msg = parseCanMessage205(frame);
            flag_205 = true;
        }

        else if (frame.can_id == 0x206)
        {
            CanMessage206 parsed_msg = parseCanMessage206(frame);
            clamp_msg.data = parsed_msg.current_clamp_width;
            lateral_msg.data = parsed_msg.current_lateral_movement_position;
            double l=parsed_msg.lateral_movement_left;
            double r=parsed_msg.lateral_movement_right;
            if(debug_mode){
                std::cout<<"mid: "<<(l+r)/2<<" l: "<<l<<" r: "<<r<<std::endl;
            }
            lateral_state_pub_.publish(lateral_msg);
            clamp_state_pub_.publish(clamp_msg);
            flag_206 = true;
        }
        
        
        if (is_print){
            // 统一打印，减小频率防止刷屏：
            ros::Time currentTime = ros::Time::now();
            if ((currentTime - lastPrintTime).toSec() >= 1.0) {
                    ROS_INFO("----------------------- CAN Feedback ----------------------");
                    ROS_INFO_STREAM("\033[1;32m Steering: "<< steering_msg.data);
                    ROS_INFO_STREAM("\033[1;32m Throttle: "<< throttle_msg.data);
                    ROS_INFO_STREAM("\033[1;32m Gear: "<< gear_cout);
                    ROS_INFO_STREAM("\033[31m Brake: "<< brake_msg.data);
                    ROS_INFO_STREAM("\033[1;32m Updown: "<< updown_msg.data);
                    ROS_INFO_STREAM("\033[1;32m Fy: "<< fy_msg.data);
                    ROS_INFO_STREAM("\033[1;32m] Lateral: "<< lateral_msg.data);
                    ROS_INFO_STREAM("\033[1;32m] Clamp: "<< clamp_msg.data);
                    ROS_INFO("----------------------- CAN Feedback ----------------------");
                lastPrintTime = currentTime;
            }
        }
    }
}
