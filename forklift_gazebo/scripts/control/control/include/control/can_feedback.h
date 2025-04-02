// CanFeedBack.h
#ifndef PATH_SPEED_CONTROL_WP_H
#define PATH_SPEED_CONTROL_WP_H

#include "ros/ros.h"
#include "std_msgs/String.h" // 根据实际需要包含适当的消息类型
#include <vector>
#include <cmath>
#include <algorithm>
#include <nav_msgs/Path.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <csignal>
#include <ros/node_handle.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include <geometry_msgs/TwistStamped.h>


struct CanMessage201
{
    double Angle;
    double Car_Speed;
    double EPOSts;
    double Gear;
};

struct CanMessage202
{
    double Fault1;
    double Fault2;
    double Fault3;
    double Fault4;
    double Mileage;
};

struct CanMessage203
{
    double SOC;
    double Brake_Pressure;
};

struct CanMessage204
{
    double LX_Hight;
    double Pitching;
};

struct CanMessage205
{
    double now_car_speed;
};


struct CanMessage206
{
    double current_clamp_width;
    double lateral_movement_left;
    double lateral_movement_right;
    double current_lateral_movement_position;
};

struct CanMessage {
    uint32_t can_id;
    uint8_t can_dlc;
    uint8_t data[8];
};

class CanFeedBack {
public:
    int s;
    CanFeedBack();
    CanFeedBack(ros::NodeHandle& nodehandle);
    void Run();
    void DecodeCanData();
private:
    ros::NodeHandle nh_;

    ros::Publisher twist_pub_, gear_state_pub_, throttle_state_pub_, brake_state_pub_, steering_state_pub_;
    ros::Publisher lateral_state_pub_, clamp_state_pub_, updown_state_pub_, fy_state_pub_;

    
    bool flag_201;
    bool flag_202;
    bool flag_203;
    bool flag_204;
    bool flag_205;
    bool flag_206;
    
    struct can_frame frame;
    int loopRate;
    // ros::Rate loop_rate;
    std::string ifname;
    bool Init_CAN(ros::NodeHandle nh_);
    bool Init_Control(ros::NodeHandle nh_);

    CanMessage201 parseCanMessage201(const struct can_frame &frame);
    CanMessage202 parseCanMessage202(const struct can_frame &frame);
    CanMessage203 parseCanMessage203(const struct can_frame &frame);
    CanMessage204 parseCanMessage204(const struct can_frame &frame);
    CanMessage205 parseCanMessage205(const struct can_frame &frame);
    CanMessage206 parseCanMessage206(const struct can_frame &frame);

    int num_cout;
    std_msgs::Float64 lateral_msg;
    std_msgs::Float64 clamp_msg;
    std_msgs::Float64 fy_msg;
    std_msgs::Float64 updown_msg;
    std_msgs::Float64 brake_msg;
    std_msgs::Int8 gear_msg;
    std_msgs::Float64 steering_msg;
    geometry_msgs::TwistStamped twist_msg;
    std_msgs::Float64 throttle_msg;

    ros::Time lastPrintTime;
    int gear_cout;
    
};



#endif // PATH_SPEED_CONTROL_WP_H
