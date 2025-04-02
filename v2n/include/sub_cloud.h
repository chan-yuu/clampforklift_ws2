#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include "nlohmann/json.hpp"
#include <mosquitto.h>
#include "car_interfaces/cloud_order.h"
#include "car_interfaces/single_task.h"
#include "car_interfaces/EmergencyStopTask.h"
#include "car_interfaces/BalesNumInCamera.h"
#include "std_msgs/Header.h"

class TaskManager {
public:
    static void signal_handler(int signum);
    TaskManager();
    // 启动整个任务管理流程，初始化相关资源、启动线程、连接MQTT等
    void start(int argc, char** argv);

    // MQTT连接回调函数
    static void on_connect(struct mosquitto *mosq, void *obj, int rc);

    // MQTT断开连接回调函数
    static void on_disconnect(struct mosquitto *mosq, void *obj, int rc);

    // MQTT消息接收回调函数
    static void on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message);

    // 处理接收到的入站任务消息
    void handleInboundTask();

    // 运行命令
    void runCommand(const std::string& command);

    // 停止命令
    void stopCommand(const std::string& commandName);

    // 处理任务规划相关逻辑
    void taskPlanning();

    // 相机使能请求相关函数
    void cameraEnableRequest(int cameraEnable);

    // 命令请求相关函数
    void cmdRequest(const nlohmann::json& taskInfo);

    // 紧急停止请求相关函数
    void emergencyStopRequest();

private:

    std::string workspacePath;
    std::string workspacePathMap;

    std::string username;
    std::string password;
    std::string brokerAddress;
    int brokerPort;
    std::string clientId;
    nlohmann::json cloudJsonReceived;
    bool newMesg;
    bool onConnectOk;
    std::string SN;
    int emergencyStopEn;
    std::string tarTaskId;
    std::string setTaskSts;
    int cameraEnable;
    std::string taskId;
    std::string orderId;
    int retryInterval;
    int maxConnectionAttempts;
    bool debugMode;
    struct mosquitto *mosq;
    ros::NodeHandle nodeHandle;
    ros::Publisher connect_pub;

    // 检查网络连接是否正常的函数
    bool checkNetworkConnection();
};

#endif