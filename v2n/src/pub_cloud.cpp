/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-12-21 22:38:01
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2025-01-10 16:02:18
 * @FilePath: /undefined/home/cyun/forklift_sim_ws3/src/v2n/src/pub_cloud.cpp
 * @Description: 
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#include <iostream>
#include <mqtt/async_client.h>
#include <thread>
#include <chrono>
#include "nlohmann/json.hpp"
#include <ros/ros.h>
#include <ros/topic.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/NavSatFix.h>
#include <ros/service_server.h>
#include <car_interfaces/TaskSts.h>
#include <car_interfaces/GpsImuInterface.h>
#include <std_msgs/Bool.h>

using json = nlohmann::json;
using namespace std::chrono;

const std::string SERVER_ADDRESS("tcp://60.28.24.166:1883");
const std::string CLIENT_ID("XEIPY30011JA98744");

// 定义数据结构
struct CarState {
    int car_run_mode;
    int EPOSts;
    int Gear;
    float Car_Speed;
    float Mileage;
    float clamp_width;
    float clamp_height;
    bool CarStartState;
    bool Fault;
    bool CarSts1;
    bool CarSts2;
    float soc;
    bool assignment_sts;
    bool vcu_sts;
};

struct DriverlessState {
    time_t timestamp; // 使用time_t类型存储时间戳
    float pos_x;
    float pos_y;
    float head;
    int unattended_state;

    int rviz_status;
    int fusion_pointclouds_status;
    int location_ndt_gps_status;
    int camera_node_status;
    int lidar_camera_det_status;
    int run_hybrid_a_star_status;
    int vehicle_control_status;
    int smach_fork_status;
    int v2n_status;
};

struct TaskStatus {
    std::string order_id;
    std::string task_id;
    std::string business_type;
    std::string origin_shelf_area;
    std::string destination_shelf_area;
    int bale_num_to_handle;
    std::string task_sts;
    struct Pose {
        float x;
        float y;
        float yaw;
    } final_pose;
};

struct HeartbeatData {
    time_t timestamp; // 使用time_t类型存储时间戳
    std::string state;
};

// 全局
DriverlessState g_driverlessState;
CarState g_carState;
TaskStatus g_taskStatus;
HeartbeatData g_heartbeatData;
CarState getCarState() {
    CarState state = {};
    return state;
}

bool is_program_running(const std::string& program_name) {
    FILE* pipe = popen(("ps -ef | grep " + program_name + " | grep -v grep | wc -l").c_str(), "r");
    if (!pipe) {
        std::cerr << "执行命令出错" << std::endl;
        return false;
    }

    char buffer[128];
    std::string result = "";
    while (fgets(buffer, sizeof(buffer), pipe)!= nullptr) {
        result += buffer;
    }

    int count = 0;
    try {
        count = std::stoi(result);
    } catch (const std::invalid_argument& e) {
        std::cerr << "转换输出为整数时出错: " << e.what() << std::endl;
    } catch (const std::out_of_range& e) {
        std::cerr << "转换输出为整数时出错: " << e.what() << std::endl;
    }

    pclose(pipe);
    return count > 0;
}


DriverlessState getDriverlessState() {
    static DriverlessState state = {};
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    state.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    // state.timestamp = system_clock::to_time_t(system_clock::now());
    // std::cout << "timestamp: " << state.timestamp << std::endl;
    state.rviz_status = is_program_running("sim.launch");
    state.smach_fork_status = is_program_running("smach_fork.launch");
    state.fusion_pointclouds_status = is_program_running("fusion_pointclouds.launch");
    state.vehicle_control_status = is_program_running("vehicle_control.launch");
    state.camera_node_status = is_program_running("camera_node.launch");
    state.lidar_camera_det_status = is_program_running("lidar_camera_det.launch");
    state.run_hybrid_a_star_status = is_program_running("run_hybrid_a_star.launch");
    state.location_ndt_gps_status = is_program_running("location_ndt_gps.launch");
    state.v2n_status = is_program_running("v2n.launch");
    return state;
}
TaskStatus getTaskStatus() {
    static TaskStatus task = {};
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    // task.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    task.bale_num_to_handle = 1;
    return task;
}
HeartbeatData getHeartbeatData() {
    static HeartbeatData data;
    data.state = "running";
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    data.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    
    return data;
}

void sendMessage(mqtt::async_client& client, const std::string& topic) {
    json message_json;


    if (topic == "suntae/agv/XEIPY30011JA98744/driveless/up") {
        auto driverless_state = getDriverlessState();
        message_json = {
            {"timestamp", static_cast<long>(driverless_state.timestamp)},
            {"x", g_driverlessState.pos_x},
            {"y", g_driverlessState.pos_y},
            {"head", g_driverlessState.head},
            {"unattended_state", driverless_state.unattended_state},
            {"sim", driverless_state.rviz_status},
            {"fusion_pointclouds", driverless_state.fusion_pointclouds_status},
            {"location_ndt_gps", driverless_state.location_ndt_gps_status},
            {"camera_node", driverless_state.camera_node_status},
            {"run_hybrid_a_star", driverless_state.run_hybrid_a_star_status},
            {"vehicle_control", driverless_state.vehicle_control_status},
            {"smach_fork", driverless_state.smach_fork_status},
            {"v2n", driverless_state.v2n_status},
            {"lidar_camera_det", driverless_state.lidar_camera_det_status}
        };
        // ROS_INFO_STREAM("\033[32m driverless_state: " << message_json.dump());

    }
    else if (topic == "suntae/agv/XEIPY30011JA98744/base/up") {
        auto car_state = getCarState();
        auto driverless_state = getDriverlessState();
        message_json = {
        {"timestamp", static_cast<long>(driverless_state.timestamp)},
        {"state", {
            {"reported", {
                {"car_run_mode", car_state.car_run_mode},
                {"epos_ts", g_carState.EPOSts},
                {"gear", car_state.Gear},
                {"car_speed", car_state.Car_Speed},
                {"mileage", car_state.Mileage},
                {"clamp_width", car_state.clamp_width},
                {"clamp_height", car_state.clamp_height},
                {"car_start_state", car_state.CarStartState},
                {"fault", car_state.Fault},
                {"car_sts1", car_state.CarSts1},
                {"car_sts2", car_state.CarSts2},
                {"vcu_sts", car_state.vcu_sts},
                {"soc", 22},
                {"assignment_sts", car_state.assignment_sts}
            }}
        }},
        
    };
        // ROS_INFO_STREAM("\033[32m car_state: " << message_json.dump());

    }
    else if (topic == "suntae/agv/XEIPY30011JA98744/task_sts/up") {
        auto task_status = getTaskStatus();
        message_json = {
            {"order_id", task_status.order_id},
            {"task_id", task_status.task_id},
            {"business_type", task_status.business_type},
            {"origin_shelf_area", task_status.origin_shelf_area},
            {"destination_shelf_area", task_status.destination_shelf_area},
            {"bale_num_to_handle", task_status.bale_num_to_handle},
            {"task_sts", task_status.task_sts},
            {"final_pose", {
                {"x", task_status.final_pose.x},
                {"y", task_status.final_pose.y},
                {"yaw", task_status.final_pose.yaw}
            }}
        };


        ROS_INFO_STREAM("\033[32m task_sts_data: " << message_json.dump());
    }
    else if (topic == "suntae/agv/XEIPY30011JA98744/heartbeat/up") {
        auto heartbeat_data = getHeartbeatData();
        message_json = {
            {"timestamp", static_cast<long>(heartbeat_data.timestamp)},
            {"state", heartbeat_data.state}
        };
        // ROS_INFO_STREAM("\033[32m heartbeat_data: " << message_json.dump());

    }

    mqtt::message_ptr pubmsg = mqtt::make_message(topic, message_json.dump());
    pubmsg->set_qos(1);
    client.publish(pubmsg)->wait();
}

void GPS_callback(const car_interfaces::GpsImuInterface& msg) {
    g_driverlessState.pos_x = msg.x;
    g_driverlessState.pos_y = msg.y;
    g_driverlessState.head = msg.yaw;
    // ROS_INFO("Received GPS data: lat = %f, lon = %f", msg.latitude, msg.longitude);
}

void EPO_callback(const std_msgs::Int8& msg) {
    g_carState.EPOSts = msg.data;
    // ROS_INFO("Received EPO data: value = %d", msg.data);
}

void SOC_callback(const std_msgs::Float64& msg) {
    g_carState.soc = msg.data;
    // ROS_INFO("Received SOC data: value = %f", msg.data);
}

// 服务回调函数
bool send_task_sts_message(car_interfaces::TaskSts::Request& req, 
car_interfaces::TaskSts::Response& res)
{
    res.success = true;
    ROS_INFO("Task status service called.");

    json task_status_json = {
        {"order_id", req.order_id},
        {"task_id", req.task_id},
        {"business_type", req.business_type},
        {"origin_shelf_area", req.origin_shelf_area},
        {"destination_shelf_area", req.destination_shelf_area},
        {"bale_num_to_handle", req.bale_num_to_handle},
        {"task_sts", req.task_sts},
        {"final_pose", {
            {"x", req.final_pose.x},
            {"y", req.final_pose.y},
            {"yaw", req.final_pose.yaw}
        }}
    };
    mqtt::async_client client(SERVER_ADDRESS, CLIENT_ID);
    mqtt::connect_options connOpts;
    connOpts.set_clean_session(true);
    connOpts.set_user_name("tju_me");
    connOpts.set_password("tjuME!@#");
    try {
        mqtt::token_ptr conntok = client.connect(connOpts);
        conntok->wait();
        mqtt::message_ptr pubmsg = mqtt::make_message("suntae/agv/XEIPY30011JA98744/task_sts/up", task_status_json.dump());
        pubmsg->set_qos(1);
        client.publish(pubmsg)->wait();
    } catch (const mqtt::exception& exc) {
        std::cerr << "MQTT Error: " << exc.what() << std::endl;
    }

    return true;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "pub_cloud");
    ros::NodeHandle nh;

    mqtt::async_client client(SERVER_ADDRESS, CLIENT_ID);

    mqtt::connect_options connOpts;
    connOpts.set_clean_session(true);
    connOpts.set_user_name("tju_me");
    connOpts.set_password("tjuME!@#");

    try {
        // std::cout << "Connecting to MQTT server." << std::endl;
        mqtt::token_ptr conntok = client.connect(connOpts);
        conntok->wait(); 
        // std::cout << "Connected to MQTT server." << std::endl;

        ros::Subscriber gps_sub = nh.subscribe("gps_imu", 1, GPS_callback);
        ros::Subscriber epo_sub = nh.subscribe("eposts_state", 1, EPO_callback);
        ros::Subscriber soc_sub = nh.subscribe("soc_state", 1, SOC_callback);

        ros::ServiceServer service = nh.advertiseService("task_sts_service", send_task_sts_message);
        ros::Publisher connect_pub = nh.advertise<std_msgs::Bool>("/v2n_pub_connect", 1);
        std_msgs::Bool connect_msg;

        while (ros::ok()) {
            // std::cout << "Sending messages..." << std::endl;
            connect_msg.data = true;
            connect_pub.publish(connect_msg);
            sendMessage(client, "suntae/agv/XEIPY30011JA98744/driveless/up");
            sendMessage(client, "suntae/agv/XEIPY30011JA98744/base/up");
            // sendMessage(client, "suntae/agv/XEIPY30011JA98744/task_sts/up");
            sendMessage(client, "suntae/agv/XEIPY30011JA98744/heartbeat/up");
            ros::spinOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

    } catch (const mqtt::exception& exc) {
    std::cerr << "Error: " << exc.what() << std::endl;
    }

    return 0;
}