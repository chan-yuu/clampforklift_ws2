/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-12-21 21:06:37
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2025-01-10 15:31:27
 * @FilePath: /undefined/home/cyun/forklift_sim_ws3/src/v2n/src/sub_cloud.cpp
 * @Description: 
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#include "sub_cloud.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <ctime>
#include <sstream>
#include <cstdlib>
#include <signal.h>
#include <ros/ros.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>

TaskManager::TaskManager() : nodeHandle("~") {
    username = "tju_me";
    password = "tjuME!@#";
    brokerAddress = "60.28.24.166";
    brokerPort = 1883;
    clientId = "vehicle_task_planning_client" + std::to_string(rand() % 1000 + 1);

    // 初始化
    newMesg = false;
    onConnectOk = false;
    SN = "XEIPY30011JA98744";
    emergencyStopEn = 0;
    tarTaskId = "";
    setTaskSts = "";
    cameraEnable = -1;
    taskId = "";
    orderId = "";
    retryInterval = 5;
    maxConnectionAttempts = 10;
    // debugMode = false;
    mosq = nullptr;
}

// void TaskManager::signal_handler(int signum) {
//     ROS_INFO("Received signal %d, shutting down...", signum);
//     ros::shutdown();
//     exit(signum);
// }

// 检查网络连接是否正常的函数
bool TaskManager::checkNetworkConnection() {
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        std::cerr << "Error creating socket" << std::endl;
        return false;
    }

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(brokerPort);
    if (inet_pton(AF_INET, brokerAddress.c_str(), &server_addr.sin_addr) <= 0) {
        std::cerr << "Invalid address or address not supported" << std::endl;
        close(sockfd);
        return false;
    }

    // 设置连接超时时间（这里设置为5秒，可根据实际需求调整）
    struct timeval timeout;
    timeout.tv_sec = 5;
    timeout.tv_usec = 0;
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout)) < 0) {
        std::cerr << "Error setting socket timeout" << std::endl;
        close(sockfd);
        return false;
    }

    if (connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Connection to MQTT broker failed" << std::endl;
        close(sockfd);
        return false;
    }

    close(sockfd);
    return true;
}


// MQTT连接回调函数
void TaskManager::on_connect(struct mosquitto *mosq, void *obj, int rc) {
    TaskManager *manager = static_cast<TaskManager *>(obj);
    if (rc == 0) {
        manager->onConnectOk = true;
        std::cout << "\033[32m 与云端连接成功\033[0m" << std::endl;
        std::string topic = "suntae/agv/" + manager->SN + "/task";
        mosquitto_subscribe(mosq, NULL, topic.c_str(), 2);
    } else {
        std::cerr << "Connection failed. Retrying..." << std::endl;
        mosquitto_reconnect(mosq);
    }
}

// MQTT断开连接回调函数
void TaskManager::on_disconnect(struct mosquitto *mosq, void *obj, int rc) {
    if (rc!= 0) {
        std::cerr << "Connection lost. Reconnecting..." << std::endl;
        mosquitto_reconnect(mosq);
    }
}

// MQTT消息接收回调函数
void TaskManager::on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message) {
    TaskManager *manager = static_cast<TaskManager *>(obj);
    try {
        nlohmann::json root = nlohmann::json::parse(static_cast<const char *>(message->payload));
        manager->cloudJsonReceived = root;
        manager->newMesg = true;
        std::cout << "\033[33m 收到云平台消息\033[0m" << std::endl;
        std::cout << "云端: " << root << std::endl;
        // manager->handleInboundTask();
    } catch (const std::exception &e) {
        std::cerr << "Exception when handling MQTT message: " << e.what() << std::endl;
    }
}

// 处理接收到的入站任务消息
void TaskManager::handleInboundTask() {
    orderId = cloudJsonReceived["order_id"].get<std::string>();
    std::string businessType = cloudJsonReceived["business_type"].get<std::string>();
    nlohmann::json taskInfoJson = cloudJsonReceived["task_info"];
    if (taskInfoJson.is_array() && taskInfoJson.size() > 0) {
        taskId = taskInfoJson[0]["task_id"].get<std::string>();
        ROS_INFO("\033[90m收到任务:\ntask id:%s\n业务类型：%s\n目标货架：%s\033[0m",
                taskId.substr(taskId.length() - 4).c_str(),
                businessType.c_str(),
                taskInfoJson[0]["destination_shelf_area"].get<std::string>().c_str());
        cmdRequest(taskInfoJson[0]);
    } else {
        std::cerr << "Invalid task_info format in received message." << std::endl;
    }
}


// 启动整个任务管理流程
void TaskManager::start(int argc, char** argv) {
    ros::init(argc, argv, "sub_cloud", ros::init_options::AnonymousName);
    // signal(SIGINT, signal_handler);

    // TODO 自动建图和整体的功能包放到同一处地方
    std::string packageName = "v2n";
    std::string packagePath = ros::package::getPath(packageName);
    connect_pub = nodeHandle.advertise<std_msgs::Bool>("/v2n_sub_connect", 10);
    // debug
    nodeHandle.param<bool>("debugMode", debugMode, true);

    if (packagePath.empty()) {
        ROS_ERROR("Could not find package path for %s.", packageName.c_str());
        return;
    }
    ROS_INFO("Package path for %s is: %s", packageName.c_str(), packagePath.c_str());

    std::string workspacePath = packagePath.substr(0, packagePath.find_last_of('/'));
    ROS_INFO("Workspace path is: %s", workspacePath.c_str());

    // std::string workspacePath = "/home/cyun/forklift_sim_ws3";
    std::string workspacePathMap = "/home/nvidia/mapping_ws";
    ROS_INFO("Mapping workspace path is: %s", workspacePathMap.c_str());

    // 检查网络连接，直到连接成功
    while (!checkNetworkConnection()) {
        ROS_INFO("Checking network connection...");
        std::this_thread::sleep_for(std::chrono::seconds(retryInterval));
    }

    // 启动任务规划线程
    // 换成while ros::ok的形式
    // std::thread taskPlanningThread(&TaskManager::taskPlanning, this);

    // 初始化mosquitto
    mosquitto_lib_init();

    // 创建MQTT客户端对象
    mosq = mosquitto_new(clientId.c_str(), true, this);
    if (!mosq) {
        ROS_ERROR("Error creating MQTT client.");
        return;
    }

    // 回调函数
    mosquitto_connect_callback_set(mosq, on_connect);
    mosquitto_disconnect_callback_set(mosq, on_disconnect);
    mosquitto_message_callback_set(mosq, on_message);

    // 设置用户名和密码
    mosquitto_username_pw_set(mosq, username.c_str(), password.c_str());

    // 连接到MQTT服务器
    int result = mosquitto_connect(mosq, brokerAddress.c_str(), brokerPort, 60);
    if (result!= MOSQ_ERR_SUCCESS) {
        ROS_ERROR("Error connecting to MQTT broker.");
        mosquitto_destroy(mosq);
        return;
    }

    // 启动MQTT网络循环，非阻塞模式
    mosquitto_loop_start(mosq);

    // 进入ROS主循环，保持程序运行并处理ROS相关事件
    while (ros::ok()) {
        taskPlanning();
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    // // 进入ROS主循环，保持程序运行并处理ROS相关事件
    // ros::spin();

    // 当程序结束（比如收到中断信号等情况），停止MQTT网络循环并清理资源
    mosquitto_loop_stop(mosq, false);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();

    // 多线程模式
    // taskPlanningThread.join();
}


// 处理任务规划相关逻辑
void TaskManager::taskPlanning() {
    std_msgs::Bool connect_msg;
    if (onConnectOk){
        connect_msg.data = true;
        connect_pub.publish(connect_msg);
    }else{
        connect_msg.data = false;
        connect_pub.publish(connect_msg);
    }

    if (onConnectOk) {
        if (newMesg) {
            std::string cmdId;
            if (cloudJsonReceived.contains("cmd_id")) {
                cmdId = cloudJsonReceived["cmd_id"].get<std::string>();
            } else {
                cmdId = "";
            }
            if (cmdId == "1001") {
                handleInboundTask();
            } else if (cmdId == "1003") {
                emergencyStopEn = cloudJsonReceived["epos"].get<int>();
                emergencyStopRequest();
            } else if (cmdId == "1004") {
                tarTaskId = cloudJsonReceived["tar_task_id"].get<std::string>();
                setTaskSts = cloudJsonReceived["set_task_sts"].get<std::string>();
            } else if (cmdId == "1005") {
                cameraEnable = cloudJsonReceived["camera_enable"].get<int>();
                if (cameraEnable == 1) {
                    cameraEnableRequest(2);
                }
            } else if (cmdId == "1010") {
                std::string function = cloudJsonReceived["function"].get<std::string>();
                std::string status = cloudJsonReceived["status"].get<std::string>();
                if (function == "lidar") {
                    if (status == "1") {
                        runCommand("bash -c 'source " + workspacePath + "/devel/setup.bash; roslaunch fusion_pointclouds fusion_pointclouds.launch'");
                    } else if (status == "0") {
                        stopCommand("fusion_pointclouds.launch");
                    }
                } else if (function == "camera") {
                    if (status == "1") {
                        runCommand("bash -c 'source " + workspacePath + "/src/lidar_camera_det/sh/start_camera_lidar_det.sh'");
                    } else if (status == "0") {
                        stopCommand("lidar_camera_det.launch");
                        stopCommand("camera_node.launch");
                    }
                } else if (function == "auto_start") {
                    if (status == "1") {
                        runCommand("bash -c 'source " + workspacePath + "/src/auto_start/sh/start_work_d.sh'");
                    } else if (status == "0") {
                        stopCommand("run_hybrid_a_star.launch");
                        stopCommand("vehicle_control.launch");
                        stopCommand("smach_fork.launch");
                    }
                } else if (function == "car_model") {
                    if (status == "1") {
                        runCommand("bash -c 'source " + workspacePath + "/devel/setup.bash; roslaunch car_ori_display sim.launch'");
                    } else if (status == "0") {
                        stopCommand("sim.launch");
                    }
                } else if (function == "location") {
                    if (status == "1") {
                        runCommand("bash -c 'source " + workspacePath + "/devel/setup.bash; roslaunch perception location_ndt_gps.launch'");
                    } else if (status == "0") {
                        stopCommand("location_ndt_gps.launch");
                    }
                } else if (function == "planning") {
                    if (status == "1") {
                        runCommand("bash -c 'source " + workspacePath + "/devel/setup.bash; roslaunch hybrid_a_star run_hybrid_a_star.launch'");
                    } else if (status == "0") {
                        stopCommand("run_hybrid_a_star.launch");
                    }
                } else if (function == "control") {
                    if (status == "1") {
                        runCommand("bash -c 'source " + workspacePath + "/devel/setup.bash; roslaunch control vehicle_control.launch'");
                    } else if (status == "0") {
                        stopCommand("vehicle_control.launch");
                    }
                } else if (function == "smach") {
                    if (status == "1") {
                        runCommand("bash -c 'source " + workspacePath + "/devel/setup.bash; roslaunch smach_fork smach_fork.launch'");
                    } else if (status == "0") {
                        stopCommand("smach_fork.launch");
                    }
                } else if (function == "mapping1") {
                    if (status == "1") {
                        runCommand("bash -c 'source " + workspacePathMap + "/src/auto_start/sh/start_map_1.sh");
                    } else if (status == "0") {
                        stopCommand("start_rviz.launch");
                        stopCommand("rviz");
                        stopCommand("lidar_start.launch");
                        stopCommand("rosbag");
                    }
                } else if (function == "mapping2") {
                    if (status == "1") {
                        runCommand("bash -c 'source " + workspacePathMap + "/src/auto_start/sh/start_map_2.sh'");
                    } else if (status == "0") {
                        stopCommand("run_lego_loam.launch");
                        stopCommand("rosbag");
                    }
                } else if (function == "mapping3") {
                    if (status == "1") {
                        runCommand("bash -c 'source " + workspacePathMap + "/src/auto_start/sh/start_map_3.sh'");
                    } else if (status == "0") {
                        stopCommand("run_pgm.launch");
                    }
                } else if (function == "mapping4") {
                    if (status == "1") {
                        runCommand("bash -c 'source " + workspacePathMap + "/src/auto_start/sh/start_map_4.sh'");
                    } else if (status == "0") {
                        stopCommand("ndt_localizer_rviz.launch");
                        stopCommand("start_no_rviz.launch");
                        stopCommand("sim.launch");
                    }
                } else if (function == "mapping5") {
                    if (status == "1") {
                        runCommand("bash -c 'source " + workspacePathMap + "/src/auto_start/sh/start_map_5.sh'");
                    } else if (status == "0") {
                        stopCommand("rosbag");
                    }
                } else if (function == "mapping6") {
                    if (status == "1") {
                        runCommand("bash -c 'source " + workspacePathMap + "/src/auto_start/sh/start_map_6.sh'");
                    } else if (status == "0") {
                        stopCommand("lidar_gps_fusion.py");
                    }
                }
            }
            newMesg = false;
        }
    } else {
        std::cout << "Waiting for connect to cloud." << std::endl;
    }
}

// 运行命令
void TaskManager::runCommand(const std::string &command) {
    std::stringstream ss;
    if (debugMode) {
        // 这里使用xterm模拟终端启动命令示例，可根据实际情况调整
        ss << "xterm -e \"" << command << "\" &";
    } else {
        // 在非调试模式下，后台执行命令并将输出重定向等，示例使用nohup，可根据实际情况调整
        ss << "nohup " << command << " > /dev/null 2>&1 &";
    }
    std::system(ss.str().c_str());
}

// 停止命令
void TaskManager::stopCommand(const std::string &commandName) {
    std::stringstream ss;
    ss << "pkill -f \"" << commandName << "\"";
    std::system(ss.str().c_str());
}

// 相机使能请求相关函数
void TaskManager::cameraEnableRequest(int cameraEnable) {
    ros::ServiceClient client = nodeHandle.serviceClient<car_interfaces::BalesNumInCamera>("bales_num_in_camera_service");
    car_interfaces::BalesNumInCamera srv;
    srv.request.data = cameraEnable;
    if (client.call(srv)) {
        ROS_INFO("Camera enable request sent successfully.");
    } else {
        ROS_ERROR("Failed to call bales_num_in_camera_service.");
    }
}


void TaskManager::cmdRequest(const nlohmann::json &taskInfo) {
    ros::ServiceClient client = nodeHandle.serviceClient<car_interfaces::cloud_order>("cloud_order_service");
    car_interfaces::cloud_order srv;
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "world";
    srv.request.header = header;
    car_interfaces::single_task task;
    task.task_id = taskInfo["task_id"].get<std::string>();
    task.bale_num_to_handle = taskInfo["bale_num_to_handle"].get<int>();
    try {
        task.origin_shelf_job_position.position.x = taskInfo["origin_shelf_job_position"]["x"].get<double>();
        task.origin_shelf_job_position.position.y = taskInfo["origin_shelf_job_position"]["y"].get<double>();
        task.origin_shelf_job_position.position.z = taskInfo["origin_shelf_job_position"]["z"].get<double>();
        task.origin_shelf_job_position.orientation.z = taskInfo["origin_shelf_job_position"]["yaw"].get<double>();
        task.destination_shelf_job_position.position.x = taskInfo["destination_shelf_job_position"]["x"].get<double>();
        task.destination_shelf_job_position.position.y = taskInfo["destination_shelf_job_position"]["y"].get<double>();
        task.destination_shelf_job_position.position.z = taskInfo["destination_shelf_job_position"]["z"].get<double>();
        task.destination_shelf_job_position.orientation.z = taskInfo["destination_shelf_job_position"]["yaw"].get<double>();
        // task.destination_shelf_area = taskInfo["destination_shelf_area"].get<std::string>();
        // std::cout << "task.destinationShelfArea:" << task.destinationShelfArea << std::endl;
    } catch (const std::exception &e) {
        ROS_ERROR("货架坐标信息缺失!");
    }
    srv.request.task_info.push_back(task);
    if (client.call(srv)) {
        // 这里可以根据实际需求处理服务响应相关逻辑，比如返回任务进度等，当前示例暂未详细处理
        srv.request.task_info.clear();
    } else {
        ROS_ERROR("Failed to call cloud_order_service.");
        srv.request.task_info.clear();
    }
}


void TaskManager::emergencyStopRequest() {
    ros::ServiceClient client = nodeHandle.serviceClient<car_interfaces::EmergencyStopTask>("emergency_stop_ser");
    car_interfaces::EmergencyStopTask srv;
    srv.request.data = emergencyStopEn;
    if (client.call(srv)) {
        std::cout << "急停指令下发状态：" << srv.response.success << std::endl;
    } else {
        ROS_ERROR("Failed to call emergency_stop_ser.");
    }
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "sub_cloud_node");
    TaskManager manager;
    manager.start(argc, argv);
    ros::shutdown();
    return 0;
}