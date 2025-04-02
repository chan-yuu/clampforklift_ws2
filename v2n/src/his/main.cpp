#include <iostream>
#include <mqtt/client.h>
#include <string>
#include <chrono>
#include "nlohmann/json.hpp"
#include <thread>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "car_interfaces/CarOriInterface.h"
#include "car_interfaces/GpsImuInterface.h"
#include "car_interfaces/PathSpeedCtrlInterface.h"
// #include "/home/hui/v2n_git_ws/devel/include/car_interfaces/PathSpeedCtrlInterface.h"


#include <csignal>
#include <unistd.h>
#include "CarDataHandler.h" 
#include "GpsDataHandler.h"
#include "TrackDataHandler.h"
void signalHandler( int signum )
{
   exit(signum);  
}

using json = nlohmann::json;


mqtt::message_ptr heartbeat(){
    // get current time
    auto now = std::chrono::system_clock::now();
    auto now_sec = std::chrono::time_point_cast<std::chrono::seconds>(now);
    auto value = now_sec.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(value).count();
    json v2n_json_data;
    v2n_json_data["timestamp"] = seconds;
    std::string v2n_json_data_str = v2n_json_data.dump();
    mqtt::message_ptr pubmsg = mqtt::make_message("suntae/agv/XEIPY30011JA98744/heartbeat/up", v2n_json_data_str);
    // std::cout << "heartbeat" << v2n_json_data_str << std::endl;
    pubmsg->set_qos(1);

    return pubmsg;
}

mqtt::message_ptr make_ori_json_data_msg(CarDataHandler car_data_handler) {
    auto now = std::chrono::system_clock::now();
    auto now_sec = std::chrono::time_point_cast<std::chrono::seconds>(now);
    auto value = now_sec.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(value).count();

    json v2n_json_data;
    v2n_json_data["timestamp"] = seconds;
    v2n_json_data["state"]["reported"] = {
        {"car_speed", car_data_handler.get_car_speed()},
        {"mileage", car_data_handler.get_mileage()},
        {"gear", car_data_handler.get_gear()},
        {"pitching", car_data_handler.get_Pitching()},
        {"lx_height", car_data_handler.get_LX_Hight()},
        {"eposts", car_data_handler.get_eposts()},
        {"car_run_mode", car_data_handler.get_car_run_mode()},
        {"vcu_sts", car_data_handler.get_vcu_sts()},
       {"brake_pressure", car_data_handler.get_brake_pressure()},
       {"car_sts1",car_data_handler.get_car_sts1()},
       {"car_sts2",car_data_handler.get_car_sts2()}
    };
    
    std::string v2n_json_data_str = v2n_json_data.dump();
    std::cout << v2n_json_data_str << std::endl;
    mqtt::message_ptr pubmsg = mqtt::make_message("suntae/agv/XEIPY30011JA98744/base/up", v2n_json_data_str);
    pubmsg->set_qos(1);
    return pubmsg;

}

mqtt::message_ptr make_driveless_data(GpsDataHandler gps_data_handler){
    auto now = std::chrono::system_clock::now();
    auto now_sec = std::chrono::time_point_cast<std::chrono::seconds>(now);
    auto value = now_sec.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(value).count();

    json v2n_json_data = {
        {"x",gps_data_handler.get_pos_x()},
        {"y",gps_data_handler.get_pos_y()},
        {"head",gps_data_handler.get_head()},
        {"timestamp",seconds}
    };

    std::string v2n_json_data_str = v2n_json_data.dump();
    std::cout << v2n_json_data_str << std::endl;
    mqtt::message_ptr pubmsg = mqtt::make_message("suntae/agv/XEIPY30011JA98744/driveless/up", v2n_json_data_str);
    pubmsg->set_qos(1);
    return pubmsg;

   }

void publishHeartbeat(mqtt::client& client) {
    while (1) {
        std::signal(SIGINT, signalHandler);  

        // 发布消息
        try {
            client.publish(heartbeat());
        } catch (const mqtt::exception& exc) {
            std::cerr << exc.what() << std::endl;
            return; // 或者使用适当的退出机制终止线程
        }
        sleep(5);
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "my_node");
    ros::NodeHandle nh;

    std::string username, password, broker_address;
    nh.getParam("/username", username);
    nh.getParam("/password", password);
    nh.getParam("/broker_address", broker_address);
    std::cout << "username: " << username << std::endl;
    std::cout << "password: " << password << std::endl;
    std::cout << "broker_address: " << broker_address << std::endl;

    // 检查是否成功获取了参数
    if (username.empty() || password.empty() || broker_address.empty()) {
        ROS_ERROR("Failed to read required parameters from the parameter server.");
        return 1;
    }

    // 创建 MQTT 客户端
    mqtt::client client(broker_address,"name");
    mqtt::connect_options connOpts;
    connOpts.set_user_name(username);
    connOpts.set_password(password);
    // // 连接到 MQTT 服务器
    try {
        client.connect(connOpts);
    } catch (const mqtt::exception& exc) {
        std::cerr << exc.what() << std::endl;
        return 1;
    }
    std::cout << "connect to mqtt server success" << std::endl;
    // 创建一个新线程来发布心跳消息
    std::thread heartbeatThread(publishHeartbeat, std::ref(client));

    CarDataHandler car_data_handler;
    GpsDataHandler gps_data_handler;
    TrackDataHandler track_data_handler;
    ros::Subscriber sub = nh.subscribe("/car_ori_data", 10,&CarDataHandler::CarOri_callback, &car_data_handler);
    ros::Subscriber sub1 = nh.subscribe("/map_pose", 10, &GpsDataHandler::GpsData_callback, &gps_data_handler);
    ros::Subscriber sub2 = nh.subscribe("path_speed_tracking_data", 10, &TrackDataHandler::TrackData_callback, &track_data_handler);

    ros::Rate loop_rate(10);  // 假设循环频率为10Hz
    while (ros::ok()) {
        ros::spinOnce();  // 处理一次回调函数
        if (car_data_handler.is_data_updated()) {  // 检查数据是否已更新
            // std::cout << "LX_Hight: " << car_data_handler.get_LX_Hight() << std::endl;
            // std::cout << "Pitching: " << car_data_handler.get_Pitching() << std::endl;
            // std::cout << "soc: " << car_data_handler.get_soc() << std::endl;
            // std::cout << "brake_pressure: " << car_data_handler.get_brake_pressure() << std::endl;
            // std::cout << "mileage: " << car_data_handler.get_mileage() << std::endl;
            // std::cout << "angle: " << car_data_handler.get_angle() << std::endl;
            // std::cout << "eposts: " << car_data_handler.get_eposts() << std::endl;
            // std::cout << "gear: " << car_data_handler.get_gear() << std::endl;
            // std::cout << "car_speed: " << car_data_handler.get_car_speed() << std::endl;
            // std::cout << "vcu_sts: " << car_data_handler.get_vcu_sts() << std::endl;
            // std::cout << "car_run_mode: " << car_data_handler.get_car_run_mode() << std::endl;
            client.publish(make_ori_json_data_msg(car_data_handler));
            std::this_thread::sleep_for(std::chrono::seconds(1));
            car_data_handler.reset_data_updated();  // 重置数据更新标记
        }

        if (gps_data_handler.is_data_updated()){
            std::cout << "pos_x_:" <<gps_data_handler.get_pos_x() << std::endl;
            std::cout << "pos_y_:" <<gps_data_handler.get_pos_y() << std::endl;
            std::cout << "head_:" <<gps_data_handler.get_head() << std::endl;
            client.publish(make_driveless_data(gps_data_handler));

            gps_data_handler.reset_data_updated();  // 重置数据更新标记

        }
        if (track_data_handler.is_data_updated()){
            // std::cout << "cte_:" <<track_data_handler.get_cte() << std::endl;
            // std::cout << "d_head_:" <<track_data_handler.get_d_head() << std::endl;
            track_data_handler.reset_data_updated();
        }
        
        loop_rate.sleep();  // 控制循环频率

    }

    // 断开连接
    client.disconnect();
    heartbeatThread.join();
    return 0;

}