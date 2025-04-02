#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>
#include <vector>
#include <iostream>
#include <cmath>
#include <autoware_msgs/DetectedObjectArray.h>
#include <car_interfaces/StopTask.h>

// 定义状态枚举
enum class State {
    IDLE_STATE,
    FORWARD_STATE,
    BACKWORD_STATE
};

// 全局变量声明
State currentState = State::IDLE_STATE;
int control_state = 0;
geometry_msgs::Pose2D current_position;
bool have_odom_ = true;
std::vector<std::pair<double, double>> obstacle_pos;
ros::Time lastPrintTime;
std_msgs::Float64 brake_msg;
bool brake = false;

double obs_x;
double obs_y;


// 新增用于存储障碍物坐标的变量
std::vector<geometry_msgs::Point> detected_obstacle_coordinates;

// 从ros参数服务器获取参数
double forword_y_set;
double forword_x_set;
double backword_y_set;
double backword_x_set;

ros::Publisher brake_pub;

// 状态转换
void transitionState(State newState) {
    std::string state_str[3] = {"IDLE_STATE", "FORWARD_STATE", "BACKWORD_STATE"};
    int pre_s = int(currentState);
    currentState = newState;
    std::cout << "\033[1;33m From " + state_str[pre_s] + " To " + state_str[int(newState)] << std::endl;
}

void coutState() {
    std::string state_str[3] = {"IDLE_STATE", "FORWARD_STATE", "BACKWORD_STATE"};
    int pre_s = int(currentState);
    ROS_INFO_STREAM("\033[1;32m State is: " << state_str[pre_s]);
}

bool handle_control_task(car_interfaces::StopTask::Request  &req, car_interfaces::StopTask::Response &res)
{
    ROS_INFO("request: control_state = %d", req.data);
    control_state = req.data;
    // 设置响应
    res.success = true;
    res.message = "Control state updated successfully.";
    return true;
}

// void process_data() {
//     // 确保这里使用ros::Time的时候，ROS环境已经初始化
//     ros::Time current_time = ros::Time::now();
//     // 处理数据的逻辑
// }

void timerCallback(const ros::TimerEvent&) {
    ros::Time start_time = ros::Time::now();
    // process_data();
    // 全局和局部任务接收
    switch (currentState) {
        //1.等待发布任务===================================
        case State::IDLE_STATE:
        {
            // 初始化：
            brake_msg.data = 0.0;
            brake = false;
            // 0.自检部分，需要一直都有定位信息：
            if (!have_odom_) {
                ROS_INFO_ONCE("\033[1;31m Wait Lidar ");
                return;
            }
            else {
                ROS_INFO_ONCE("\033[1;32m Odom Get");
            }

            if (control_state == 1) // 前进任务
            {
                transitionState(State::FORWARD_STATE);
            }
            else if (control_state == 2) // 倒车
            {
                transitionState(State::BACKWORD_STATE);
            }
            else {
                ROS_INFO_ONCE("\033[1;31m Wait Smach ");
            }
            break;
        }

        case State::FORWARD_STATE:
        {
            brake = false;
            brake_msg.data = 0.0;
            obs_x = 0.0;
            obs_y = 0.0;
            if (control_state!= 1) {
                transitionState(State::IDLE_STATE);
            }
            for (const auto& point : detected_obstacle_coordinates) {
                if (std::abs(point.y) < forword_y_set) {
                    if (point.x < forword_x_set&&point.x>0) {
                        // std::cout<< "point_x: " << point.x <<"point.y: "<<point.y<<std::endl;
                        obs_x = point.x;
                        obs_y = point.y;
                        brake = true;
                        brake_msg.data = 6.0;
                        brake_pub.publish(brake_msg);
                    }
                }
            }
            break;
        }

        case State::BACKWORD_STATE:
        {
            brake = false;
            brake_msg.data = 0.0;
            obs_x = 0.0;
            obs_y = 0.0;
            if (control_state!= 2) {
                transitionState(State::IDLE_STATE);
            }
            for (const auto& point : detected_obstacle_coordinates) {
                if (std::abs(point.y) < backword_y_set) {
                    if (point.x > backword_x_set&&point.x<0) {
                        // std::abs(point.y)
                        obs_x = point.x;
                        obs_y = point.y;
                        brake = true;
                        brake_msg.data = 6.0;
                        brake_pub.publish(brake_msg);
                    }
                }

            }
            break;
        }

    } // switch
    ros::Time end_time = ros::Time::now();
    ros::Duration exec_time = end_time - start_time;  // 计算执行时间

    ros::Time currentTime = ros::Time::now();
    if ((currentTime - lastPrintTime).toSec() >= 1.0) {
        ROS_INFO("--------- Stop FSM ---------");
        if (brake) {
            ROS_INFO_STREAM("\033[1;31m Brake: " << brake);
            ROS_INFO_STREAM("\033[1;31m Brake_Value: " << brake_msg.data);
            ROS_INFO_STREAM("\033[1;31m Obs X: " << obs_x);
            ROS_INFO_STREAM("\033[1;31m Obs Y: " << obs_y);
        }
        else {
            ROS_INFO_STREAM("\033[1;32m Brake: " << brake);
            ROS_INFO_STREAM("\033[1;32m Brake_Value: " << brake_msg.data);
        }
        brake = false;
        // ROS_INFO_STREAM("\033[1;32m Brake: " << brake);
        // ROS_INFO_STREAM("\033[1;32m Brake: " << brake);
        coutState();
        ROS_INFO_STREAM("Execution time: " << exec_time.toSec() << " seconds");
        ROS_INFO("--------- Stop FSM ---------");
        lastPrintTime = currentTime;
    }
}

void detectedObjectCallback(const autoware_msgs::DetectedObjectArray::ConstPtr& msg) {
    detected_obstacle_coordinates.clear();
    // std::cout<< "sub detection"<<std::endl;
    for (const auto& object : msg->objects) {
        geometry_msgs::Point point;
        point.x = object.pose.position.x;
        point.y = object.pose.position.y;
        point.z = object.pose.position.z;
        detected_obstacle_coordinates.push_back(point);
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "stop_obstacle");
    ros::NodeHandle nh;

    // 从参数服务器获取参数，设置默认值
    nh.param<double>("forword_y_set", forword_y_set, 0.5);
    nh.param<double>("forword_x_set", forword_x_set, 1.0);
    nh.param<double>("backword_y_set", backword_y_set, 0.5);
    nh.param<double>("backword_x_set", backword_x_set, -1.0);

    brake_pub = nh.advertise<std_msgs::Float64>("brake_cmd", 1);

    ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback);
    ros::Subscriber sub = nh.subscribe("/detection/lidar_detector/objects", 1, detectedObjectCallback);

    ros::ServiceServer service_plan_task_ = nh.advertiseService("stop_task", handle_control_task);

    ros::spin();
    return 0;
}