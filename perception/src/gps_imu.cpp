/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-09-12 16:10:37
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2025-01-09 15:34:02
 * @FilePath: /undefined/home/nvidia/clamp_forklift_ws2/src/perception/src/gps_imu.cpp
 * @Description: 
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
/*
解析can消息并发布
time： 2024-3-4
cyun 
*/
#include <sys/types.h>
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
#include <car_interfaces/GpsImuInterface.h>
#include <thread>
#include <iostream>
#include <cmath>
#include <csignal>
#include <mutex>
#include <color_print.h>
#include <car_interfaces/CarOriInterface.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>


// 全局变量和互斥锁
std::mutex mutex;
car_interfaces::GpsImuInterface shared_msg;
bool has_new_data = false;
bool has_forward = false;
bool ndtState = false;


ros::Publisher pub ;
int loopRate;

// 转换函数
struct UTMCoordinates {
    double easting;
    double northing;
    int zone_number;
    char zone_letter;
};


UTMCoordinates fromLatLon(double latitude, double longitude, int force_zone_number = -1) {
    const double K0 = 0.9996;
    const double E = 0.00669438;
    const double E2 = E * E;
    const double E3 = E2 * E;
    const double E_P2 = E / (1.0 - E);
    const double SQRT_E = std::sqrt(1 - E);
    const double _E = (1 - SQRT_E) / (1 + SQRT_E);
    const double _E2 = _E * _E;
    const double _E3 = _E2 * _E;
    const double _E4 = _E3 * _E;
    const double _E5 = _E4 * _E;
    const double M1 = (1 - E / 4 - 3 * E2 / 64 - 5 * E3 / 256);
    const double M2 = (3 * E / 8 + 3 * E2 / 32 + 45 * E3 / 1024);
    const double M3 = (15 * E2 / 256 + 45 * E3 / 1024);
    const double M4 = (35 * E3 / 3072);
    const double R = 6378137;
    const std::string ZONE_LETTERS = "CDEFGHJKLMNPQRSTUVWXX";

    if (latitude < -80.0 || latitude > 84.0) {
        throw std::out_of_range("Latitude out of range (must be between -80 deg and 84 deg)");
    }

    if (longitude < -180.0 || longitude > 180.0) {
        throw std::out_of_range("Longitude out of range (must be between -180 deg and 180 deg)");
    }

    double lat_rad = latitude * M_PI / 180.0;
    double lat_sin = std::sin(lat_rad);
    double lat_cos = std::cos(lat_rad);
    double lat_tan = lat_sin / lat_cos;
    double lat_tan2 = lat_tan * lat_tan;
    double lat_tan4 = lat_tan2 * lat_tan2;

    int zone_number;
    char zone_letter;

    if (force_zone_number == -1) {
        zone_number = static_cast<int>((longitude + 180.0) / 6) + 1;
        zone_letter = ZONE_LETTERS[(latitude + 80) / 8];
    } else {
        zone_number = force_zone_number;
        zone_letter = ZONE_LETTERS[(latitude + 80) / 8];
    }

    double lon_rad = longitude * M_PI / 180.0;
    double central_lon = (zone_number - 1) * 6 - 180 + 3;
    double central_lon_rad = central_lon * M_PI / 180.0;

    double n = R / std::sqrt(1 - E * lat_sin * lat_sin);
    double c = E_P2 * lat_cos * lat_cos;

    double a = lat_cos * (lon_rad - central_lon_rad);
    double a2 = a * a;
    double a3 = a2 * a;
    double a4 = a3 * a;
    double a5 = a4 * a;
    double a6 = a5 * a;

    double m = R * (M1 * lat_rad -
                    M2 * std::sin(2 * lat_rad) +
                    M3 * std::sin(4 * lat_rad) -
                    M4 * std::sin(6 * lat_rad));

    double easting = K0 * n * (a +
                              a3 / 6 * (1 - lat_tan2 + c) +
                              a5 / 120 * (5 - 18 * lat_tan2 + lat_tan4 + 72 * c - 58 * E_P2)) + 500000;

    double northing = K0 * (m + n * lat_tan * (a2 / 2 +
                                               a4 / 24 * (5 - lat_tan2 + 9 * c + 4 * c * c) +
                                               a6 / 720 * (61 - 58 * lat_tan2 + lat_tan4 + 600 * c - 330 * E_P2)));
    if (latitude < 0) {
        northing += 10000000;
    }

    UTMCoordinates coords;
    coords.easting = easting;
    coords.northing = northing;
    coords.zone_number = zone_number;
    coords.zone_letter = zone_letter;

    return coords;
}

struct CanMessage10B {
    double yaw;
    double pitch;
    double roll;
};

struct CanMessage20B {
    double lat;
    double lon;
    double posX;
    double posY;
};

struct CanMessage30B {
    double PosAlt;
    double posZ;
    // double posX;
    bool state;
};


struct CanMessage40B {
    double VelE;
    double VelN;
    double VelU;
    double Vel;
};

struct CanMessage50B {
    double x_acc;
    double y_acc;
    double acc;
};

struct CanMessage60B {
    double x_gyro;
    double y_gyro;
};

struct CanMessage70B {
    double z_acc;
    double z_gyro;
};

struct CanMessage51B{
    double heading_gnss;
};

CanMessage10B parseCanMessage10B(const struct can_frame& frame) {
    CanMessage10B msg;
    // for (int i = 0; i < frame.can_dlc; i++) {
    //                 printf("%02X ", frame.data[i]);
    //             }
    //             printf("\n");
    uint16_t yaw_raw = ((uint8_t)frame.data[1] << 8) | frame.data[0];
    // std::cout << "移位后的十六进制数：" << std::hex << yaw_raw << std::endl;
    if (static_cast<double>(yaw_raw) * 1e-2 >=0 && static_cast<double>(yaw_raw) * 1e-2<=360){
        msg.yaw = static_cast<double>(yaw_raw) * 1e-2;
    }
    msg.yaw = static_cast<double>(yaw_raw) * 1e-2;

    int16_t pitch_raw = ((uint8_t)frame.data[3] << 8) | frame.data[2];
    if (static_cast<double>(pitch_raw) * 1e-2 >=-90 && static_cast<double>(pitch_raw) * 1e-2<=90){
    msg.pitch = static_cast<double>(pitch_raw) * 1e-2;
    }
    msg.pitch = static_cast<double>(pitch_raw) * 1e-2;

    // msg.pitch = static_cast<double>(pitch_raw) * 1e-2;
    // std::cout << "移位后的十六进制数：" << std::hex << pitch_raw << std::endl;

    // printf("%02X ", pitch_raw);
    int16_t roll_raw = ((uint8_t)frame.data[5] << 8) | frame.data[4];
    if (static_cast<double>(roll_raw) * 1e-2 >=-180 && static_cast<double>(roll_raw) * 1e-2<=180){
    msg.roll = static_cast<double>(roll_raw) * 1e-2;
    }
    msg.roll = static_cast<double>(roll_raw) * 1e-2;
    return msg;
}

CanMessage51B parseCanMessage51B(const struct can_frame& frame) {
    CanMessage51B msg;
    // for (int i = 0; i < frame.can_dlc; i++) {
    //                 printf("%02X ", frame.data[i]);
    //             }
    //             printf("\n");
    uint16_t heading_gnss_raw = ((uint8_t)frame.data[1] << 8) | frame.data[0];
    // if (static_cast<double>(heading_gnss_raw) * 1e-2 >=0 && static_cast<double>(heading_gnss_raw) * 1e-2<=360){
    //     msg.heading_gnss = static_cast<double>(heading_gnss_raw) * 1e-2;
    // }
    msg.heading_gnss = static_cast<double>(heading_gnss_raw) * 1e-2;
    return msg;
}

CanMessage20B parseCanMessage20B(const struct can_frame& frame) {
    CanMessage20B msg;

    int32_t lat_raw = ((uint8_t)frame.data[3] << 24) | 
    ((uint8_t)frame.data[2] << 16) | 
    ((uint8_t)frame.data[1] << 8) | 
    frame.data[0];
    if (static_cast<double>(lat_raw) * 1e-7 >=-90 && static_cast<double>(lat_raw) * 1e-7<=90){
    msg.lat = static_cast<double>(lat_raw) * 1e-7;
    }
    msg.lat = static_cast<double>(lat_raw) * 1e-7;

    //std::cout << "移位后的十六进制数：" << std::hex << lat_raw << std::endl;

    int32_t lon_raw = ((uint8_t)frame.data[7] << 24) | 
    ((uint8_t)frame.data[6] << 16) | 
    ((uint8_t)frame.data[5] << 8) | 
    frame.data[4];
    if (static_cast<double>(lon_raw) * 1e-7 >=-180 && static_cast<double>(lon_raw) * 1e-7<=180){
    msg.lon = static_cast<double>(lon_raw) * 1e-7;
    }
    msg.lon = static_cast<double>(lon_raw) * 1e-7;

    msg.posX = fromLatLon(msg.lat, msg.lon).easting;//- 604000;
    msg.posY = fromLatLon(msg.lat, msg.lon).northing;// - 4084800;
    
    return msg;
} 

CanMessage30B parseCanMessage30B(const struct can_frame& frame) {
    CanMessage30B msg;

    int32_t alt_raw = ((uint8_t)frame.data[3] << 24) | 
    ((uint8_t)frame.data[2] << 16) | 
    ((uint8_t)frame.data[1] << 8) | 
    frame.data[0];
    // msg.PosAlt = static_cast<double>(alt_raw) * 1e-3;
    // msg.posZ = msg.PosAlt;
    if (static_cast<double>(alt_raw) * 1e-3 >=-2147483.648 && static_cast<double>(alt_raw) * 1e-3<=2147483.647){
    msg.PosAlt = static_cast<double>(alt_raw) * 1e-3;
    msg.posZ = msg.PosAlt;
    }
    msg.PosAlt = static_cast<double>(alt_raw) * 1e-3;
    msg.posZ = msg.PosAlt;

    uint8_t flag_raw = frame.data[4];
    if (flag_raw >=0 && flag_raw <=255){
    if (flag_raw == 4){
        msg.state = true;
    }
    else{
        msg.state = false;
    }
    }

    if (flag_raw == 4){
        msg.state = true;
    }
    else{
        msg.state = false;
    }

    return msg;
} 


double disSquareRootOfSumOfSquares(double num1, double num2) {
    double sumOfSquares = std::pow(num1, 2) + std::pow(num2, 2);
    double result = std::sqrt(sumOfSquares);
    return result;
}


CanMessage40B parseCanMessage40B(const struct can_frame& frame) {
    CanMessage40B msg;

    int16_t VelE_raw = ((uint8_t)frame.data[1] << 8) | frame.data[0];
    
    if (static_cast<double>(VelE_raw) * 1e-2 >=-327.68 && static_cast<double>(VelE_raw) * 1e-2<=327.67){
    msg.VelE = static_cast<double>(VelE_raw) * 1e-2;
    }
    msg.VelE = static_cast<double>(VelE_raw) * 1e-2;

    int16_t VelN_raw = ((uint8_t)frame.data[3] << 8) | frame.data[2];
    // msg.VelN = static_cast<double>(VelN_raw) * 1e-2;
    if (static_cast<double>(VelN_raw) * 1e-2 >=-327.68 && static_cast<double>(VelN_raw) * 1e-2<=327.67){
    msg.VelN = static_cast<double>(VelN_raw) * 1e-2;
    }
    msg.VelN = static_cast<double>(VelN_raw) * 1e-2;

    int16_t VelU_raw = ((uint8_t)frame.data[5] << 8) | frame.data[4];
    msg.VelU = static_cast<double>(VelU_raw) * 1e-2;
    
    if (static_cast<double>(VelU_raw) * 1e-2 >=-323.68 && static_cast<double>(VelU_raw) * 1e-2<=327.67){
    msg.VelN = static_cast<double>(VelU_raw) * 1e-2;
    }
    msg.VelN = static_cast<double>(VelU_raw) * 1e-2;

    msg.Vel = disSquareRootOfSumOfSquares(msg.VelE, msg.VelN);//std::sqrt((std::pow(msg.VelE))+(std::pow(msg.VelN)));
    return msg;
}


CanMessage50B parseCanMessage50B(const struct can_frame& frame) {
    CanMessage50B msg;
    int32_t x_acc_raw = ((uint8_t)frame.data[3] << 24) |
    ((uint8_t)frame.data[2] << 16) |
    ((uint8_t)frame.data[1] << 8) |
    frame.data[0];

    if (static_cast<double>(x_acc_raw) * 1e-5 >=-160 && static_cast<double>(x_acc_raw) * 1e-5<=160){
    msg.x_acc = static_cast<double>(x_acc_raw) * 1e-5;
    }
    msg.x_acc = static_cast<double>(x_acc_raw) * 1e-5;

    // msg.x_acc = static_cast<double>(x_acc_raw) * 1e-5;
    
    int32_t y_acc_raw = ((uint8_t)frame.data[7] << 24) | 
    ((uint8_t)frame.data[6] << 16) | 
    ((uint8_t)frame.data[5] << 8) | 
    frame.data[4];
    if (static_cast<double>(y_acc_raw) * 1e-5 >=-160 && static_cast<double>(y_acc_raw) * 1e-5<=160){
    msg.y_acc = static_cast<double>(y_acc_raw) * 1e-5;
    }
    msg.y_acc = static_cast<double>(y_acc_raw) * 1e-5;

    // msg.acc = std::sqrt((std::pow(msg.x_acc))+(std::pow(msg.y_acc)));
    msg.acc = disSquareRootOfSumOfSquares(msg.x_acc, msg.y_acc);//std::sqrt((std::pow(msg.VelE))+(std::pow(msg.VelN)));
    return msg;
}


CanMessage60B parseCanMessage60B(const struct can_frame& frame) {
    CanMessage60B msg;
    // double x_gyro;
    // double y_gyro;
    int32_t x_gyro_raw = ((uint8_t)frame.data[3] << 24) | 
    ((uint8_t)frame.data[2] << 16) | 
    ((uint8_t)frame.data[1] << 8) | 
    frame.data[0];
    if (static_cast<double>(x_gyro_raw) * 1e-5 >=-300 && static_cast<double>(x_gyro_raw) * 1e-5<=300){
    msg.x_gyro = static_cast<double>(x_gyro_raw) * 1e-5;
    }
    msg.x_gyro = static_cast<double>(x_gyro_raw) * 1e-5;

    int32_t y_gyro_raw = ((uint8_t)frame.data[7] << 24) | 
    ((uint8_t)frame.data[6] << 16) | 
    ((uint8_t)frame.data[5] << 8) | 
    frame.data[4];
    if (static_cast<double>(y_gyro_raw) * 1e-5 >=-300 && static_cast<double>(y_gyro_raw) * 1e-5<=300){
    msg.y_gyro = static_cast<double>(y_gyro_raw) * 1e-5;
    }
    msg.y_gyro = static_cast<double>(y_gyro_raw) * 1e-5;

    return msg;
} 



CanMessage70B parseCanMessage70B(const struct can_frame& frame) {
    CanMessage70B msg;
    // for (int i = 0; i < frame.can_dlc; i++) {
    //                 printf("%02X ", frame.data[i]);
    //             }
    //             printf("\n");
    int32_t z_gyro_raw = ((uint8_t)frame.data[3] << 24) | 
    ((uint8_t)frame.data[2] << 16) | 
    ((uint8_t)frame.data[1] << 8) | 
    frame.data[0];

    if (static_cast<double>(z_gyro_raw) * 1e-5 >=-300 && static_cast<double>(z_gyro_raw) * 1e-5<=300){
    msg.z_gyro = static_cast<double>(z_gyro_raw) * 1e-5;
    }
    msg.z_gyro = static_cast<double>(z_gyro_raw) * 1e-5;

    // std::cout << "移位后的十六进制数：" << std::hex << z_gyro_raw << std::endl;

    int32_t z_acc_raw = ((uint8_t)frame.data[7] << 24) | 
    ((uint8_t)frame.data[6] << 16) | 
    ((uint8_t)frame.data[5] << 8) | 
    frame.data[4];
    if (static_cast<double>(z_acc_raw) * 1e-5 >=-160 && static_cast<double>(z_acc_raw) * 1e-5<=160){
    msg.z_acc = static_cast<double>(z_acc_raw) * 1e-5;
    }
        msg.z_acc = static_cast<double>(z_acc_raw) * 1e-5;

    return msg;
}



void oriCallback(const car_interfaces::CarOriInterfaceConstPtr& msg)
{
    int gear;
    gear = msg->Gear;
    if(gear==3)  has_forward=true;
    else has_forward=false;
    // std::cout<<gear<<std::endl;
    // color_print::prBlue("gear:",msg->Gear);
}

 
void mapCallback(const car_interfaces::CarOriInterfaceConstPtr& msg)
{
    ndtState = true;
}

// 0-360 ->  -180 - 180
double angle_2_angle(double angle) {
    // angle -= 90;
    // while (angle < -180) {
    //     angle += 360;
    // }
    // while (angle > 180) {
    //     angle -= 360;
    // }
    // return angle;
    angle -= 90;  //0-360
    while (angle < 0) {
        angle += 360;
    }
    return angle;
}



void quit(int signal) {
    std::cout << "Ctrl+C pressed. Exiting..." << std::endl;
    std::exit(0);
}



bool flag_10B = false;
bool flag_20B = false;
bool flag_30B = false;
bool flag_40B = false;
bool flag_50B = false;
bool flag_60B = false;
bool flag_70B = false;


// 0-360 -> -180-180
double convert(double angle){
    // 将输入的角度标准化到 [-180, 180] 范围内
    // while (angle > 180) angle -= 360;
    // while (angle <= -180) angle += 360;

    // 计算转换后的航向
    // if (angle >= 0) {
    //     // 当角度在 [0, 180] 范围内时
    //     return -(angle + 180);
    // } else {
    //     // 当角度在 [-180, 0) 范围内时
    //     return -(angle + 180);
    // }
    return (180 - angle);
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, quit);
    
    std::setlocale(LC_ALL, "");
    ros::init(argc, argv, "gps_imu_node");

    // car_interfaces::GpsImuInterface msg;
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<car_interfaces::GpsImuInterface>("gps_imu", 10);
    ros::Subscriber ori_sub = n.subscribe<car_interfaces::CarOriInterface>("/car_ori_data", 10, oriCallback);
    // ros::Subscriber map_sub = n.subscribe<car_interfaces::GpsImuInterface>("/map_pose", 10, mapCallback);
    
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    std::string ifname;

    double global_pos_X, global_pos_Y;
    // int loopRate;

    // 使用param方法直接指定默认值
    ifname = n.param<std::string>("/gps_imu_node/ifname", "can1");
    global_pos_X = n.param("/gps_imu_node/global_pos_X", 520000.0);
    global_pos_Y = n.param("/gps_imu_node/global_pos_Y", 4200000.0);
    loopRate = n.param("/gps_imu_node/loopRate", 100);
    // ros::Rate loop_rate(loopRate); // 或其他适当的频率

    const char* ifname_cstr = ifname.c_str();
    // std::cout<<"ifname.c_str()"<<ifname.c_str()<<std::endl;
    // const char *ifname = "can1";

    // 检查CAN口
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening socket");
        return -1;
    }
    // 创建连接
    strcpy(ifr.ifr_name, ifname.c_str());
    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error in socket bind");
        return -2;
    }
    ros::Rate loop_rate(loopRate); // 或其他适当的频率

    struct can_frame frame;
    car_interfaces::GpsImuInterface msg;
    sensor_msgs::NavSatFix gps_msg;
    sensor_msgs::Imu imu_msg;
    ros::Publisher gps_pub = n.advertise<sensor_msgs::NavSatFix>("gps/fix", 1);
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu", 1);

    while (ros::ok()) 
    {
        // ros::spinOnce();
        ssize_t nbytes =read(s, &frame, sizeof(struct can_frame));
        if (nbytes > 0)
        {
            // std::cout<<frame.can_id<<"frame.can_id"<<std::endl;
            if (frame.can_id == 0x10B) 
            {
                CanMessage10B parsed_msg_10B = parseCanMessage10B(frame);
                //ADD 换：
                msg.roll = parsed_msg_10B.roll;
                msg.pitch = parsed_msg_10B.pitch;
                // color_print::prGreen("pitch", msg.AngleHeading, msg.pitch,"\n");
                flag_10B = true;
            }
            else if (frame.can_id == 0x20B) 
            {
                CanMessage20B parsed_msg_20B = parseCanMessage20B(frame);
                    msg.PosLat = parsed_msg_20B.lat;
                    msg.lat = msg.PosLat;
                    msg.PosLon = parsed_msg_20B.lon;
                    msg.lon = msg.PosLon;
                    msg.posX = parsed_msg_20B.posX - global_pos_X;
                    msg.posY = parsed_msg_20B.posY - global_pos_Y;
                    msg.x = msg.posX;
                    msg.y = msg.posY;
                    flag_20B = true;
                    // color_print::prBlue("lat", msg.lat, msg.lon,"\n");
            }
            else if (frame.can_id == 0x30B) 
            {
                CanMessage30B parsed_msg_30B = parseCanMessage30B(frame);
                    msg.PosAlt = parsed_msg_30B.PosAlt;
                    msg.posZ = msg.PosAlt;//parsed_msg_30B.posZ;
                    msg.z = msg.posZ;
                    msg.state = parsed_msg_30B.state;
                    flag_30B = true;
            }
            else if (frame.can_id == 0x40B) 
            {
                CanMessage40B parsed_msg_40B = parseCanMessage40B(frame);
                msg.VelE = parsed_msg_40B.VelE;
                msg.VelN = parsed_msg_40B.VelN;
                msg.VelU = parsed_msg_40B.VelU;
                msg.Vel = parsed_msg_40B.Vel;
                flag_40B = true;
            }
            else if (frame.can_id == 0x50B) 
            {
                CanMessage50B parsed_msg_50B = parseCanMessage50B(frame);
                msg.x_acc = parsed_msg_50B.x_acc;
                msg.y_acc = parsed_msg_50B.y_acc;
                msg.acc = parsed_msg_50B.acc;
                imu_msg.linear_acceleration.x = parsed_msg_50B.x_acc;
                imu_msg.linear_acceleration.y = parsed_msg_50B.y_acc;
                flag_50B = true;
            }
            else if (frame.can_id == 0x60B)
            {
                CanMessage60B parsed_msg_60B = parseCanMessage60B(frame);
                msg.x_gyro = parsed_msg_60B.x_gyro;
                msg.y_gyro = parsed_msg_60B.y_gyro;
                imu_msg.angular_velocity.x = msg.x_gyro * (M_PI / 180.0);
                imu_msg.angular_velocity.y = msg.y_gyro * (M_PI / 180.0);
                flag_60B = true;
            }
            else if (frame.can_id == 0x70B) 
            {
                CanMessage70B parsed_msg_70B = parseCanMessage70B(frame);
                msg.z_acc = parsed_msg_70B.z_acc;
                msg.z_gyro = parsed_msg_70B.z_gyro;
                imu_msg.linear_acceleration.z = parsed_msg_70B.z_acc;
                imu_msg.angular_velocity.z = msg.z_gyro * (M_PI / 180.0);
                flag_70B = true;
            }
            else if (frame.can_id == 0x51B) 
            {
                CanMessage51B parsed_msg_51B = parseCanMessage51B(frame);
                // msg.alt_g = parsed_msg_51B.heading_gnss;
                msg.AngleHeading = convert(parsed_msg_51B.heading_gnss);
                msg.yaw = msg.AngleHeading;
            }
        }
        
        if(flag_10B&&flag_30B)
        {
            msg.header.frame_id = "gps";
            msg.header.stamp = ros::Time::now();
            pub.publish(msg);
            flag_10B = false;
            flag_30B = false;

            sensor_msgs::NavSatFix gps_msg;
            gps_msg.header.stamp = ros::Time::now();
            gps_msg.header.frame_id = "gps_link";
            gps_msg.latitude = msg.lat;
            gps_msg.longitude = msg.lon;
            gps_msg.altitude = 0.0;
            gps_msg.status.status = 2;
            gps_pub.publish(gps_msg);

            // TODO 确认6110的惯导的imu数据的情况是怎么样的
            tf::Quaternion q;
            q.setRPY(msg.roll * (M_PI / 180.0), msg.pitch * (M_PI / 180.0), msg.yaw * (M_PI / 180.0));
            imu_msg.orientation.x = q.x();
            imu_msg.orientation.y = q.y();
            imu_msg.orientation.z = q.z();
            imu_msg.orientation.w = q.w();
            // 使用接收到的GPS消息的时间戳
            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = "imu_link";
            imu_pub.publish(imu_msg);
        }



    }

    close(s);
    return 0;
}