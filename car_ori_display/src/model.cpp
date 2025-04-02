/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-09-13 11:20:19
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2024-09-15 14:17:20
 * @FilePath: /src/car_ori_display/src/model.cpp
 * @Description: 测试集中常见的模型对应的轨迹情况
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <angles/angles.h>
#include <cmath>
#include <std_msgs/Float64.h>
#include <car_interfaces/GpsImuInterface.h>
#include <car_interfaces/PathSpeedCtrlInterface.h>

car_interfaces::GpsImuInterface locationMsg;
car_interfaces::PathSpeedCtrlInterface controlMsg;


// void LocationCallback(const car_interfaces::GpsImuInterface::ConstPtr& msg){
// 	locationMsg= *msg;
// }

// void ControlCallback(const car_interfaces::PathSpeedCtrlInterface::ConstPtr& msg){
// 	controlMsg= *msg;
// }


/**
 * @brief 差速轮和阿克曼运动模型预测位姿
 * @param  pose             当前位姿
 * @param  v                当前线速度
 * @param  ws               当前角速度或阿克曼的转向轮角度
 * @param  dt               运动所长时间
 * @param  wheel_base       阿克曼的轴距
 * @param  use_steering     是否将 ws 参数作为阿克曼转向轮角度
 * @return geometry_msgs::PoseStamped 返回运动 dt 时间后的位姿
 */
geometry_msgs::PoseStamped predict(const geometry_msgs::PoseStamped& pose,
                                   const double v, const double ws,
                                   const double dt, const double wheel_base,
                                   const bool use_steering = false) {
  double dx = 0, dy = 0, alpha = 0;
  double epsilon = 0.001;

  if (fabs(ws) < epsilon) { // 认为是直线运动
    dx = v * dt;
  } else {
    double r, w = ws;
    if (use_steering) {
      r = fabs(wheel_base / tan(ws));
      r *= (ws > 0) ^ (v >= 0) ? -1 : 1;
      w = v / r;
    }
    double angle = fabs(w * dt);
    while (angle > 2 * M_PI) angle -= 2 * M_PI;
    int dx_sign = (angle > M_PI ? -1 : 1) * (v > 0 ? 1 : -1);
    alpha = angles::normalize_angle(w * dt);
    dy = v / w * (1 - cos(w * dt));
    dx = sqrt(pow(v / w, 2) - pow(v / w - dy, 2)) * dx_sign;
  }

  geometry_msgs::PoseStamped next_pose = pose;
  double yaw = tf::getYaw(next_pose.pose.orientation);
  next_pose.pose.position.x += cos(yaw) * dx - sin(yaw) * dy;
  next_pose.pose.position.y += sin(yaw) * dx + cos(yaw) * dy;
  double next_yaw = angles::normalize_angle(yaw + alpha);
  next_pose.pose.orientation = tf::createQuaternionMsgFromYaw(next_yaw);
  return next_pose;
}

/**
 * @brief 自行车运动学模型预测位姿
 * @param  pose             当前位姿
 * @param  v                当前线速度
 * @param  delta            前轮转角
 * @param  dt               运动时间
 * @param  L                轴距
 * @return geometry_msgs::PoseStamped 返回运动 dt 时间后的位姿
 */
geometry_msgs::PoseStamped predict_kinematic_bicycle(const geometry_msgs::PoseStamped& pose,
                                                     const double v, const double delta,
                                                     const double dt, const double L) {
  geometry_msgs::PoseStamped next_pose = pose;
  double yaw = tf::getYaw(next_pose.pose.orientation);

  double beta = atan2(L / 2.0 * tan(delta), L);
  next_pose.pose.position.x += v * cos(yaw + beta) * dt;
  next_pose.pose.position.y += v * sin(yaw + beta) * dt;
  double next_yaw = yaw + v / L * tan(delta) * dt;
  next_pose.pose.orientation = tf::createQuaternionMsgFromYaw(angles::normalize_angle(next_yaw));
  return next_pose;
}

/**
 * @brief 自行车动力学模型预测位姿
 * @param  pose             当前位姿
 * @param  v                当前线速度
 * @param  delta            前轮转角
 * @param  dt               运动时间
 * @param  L                轴距
 * @param  L_f              重心到前轴的距离
 * @param  L_r              重心到后轴的距离
 * @param  C_alpha          轮胎侧偏刚度
 * @param  m                车辆质量
 * @param  I_z              车辆绕z轴的转动惯量
 * @return geometry_msgs::PoseStamped 返回运动 dt 时间后的位姿
 */
geometry_msgs::PoseStamped predict_dynamic_bicycle(const geometry_msgs::PoseStamped& pose,
                                                   const double v, const double delta,
                                                   const double dt, const double L,
                                                   const double L_f, const double L_r,
                                                   const double C_alpha, const double m,
                                                   const double I_z) {
  geometry_msgs::PoseStamped next_pose = pose;
  double yaw = tf::getYaw(next_pose.pose.orientation);
  double beta = atan2(L_r * tan(delta), L);
  double a_f = delta - beta;
  double a_r = -beta;
  double F_yf = 2 * C_alpha * a_f;
  double F_yr = 2 * C_alpha * a_r;

  double a_x = v * cos(beta);
  double a_y = v * sin(beta);
  double a_psi = (L_f * F_yf - L_r * F_yr) / I_z;

  next_pose.pose.position.x += (v * cos(yaw) - a_y * sin(yaw)) * dt;
  next_pose.pose.position.y += (v * sin(yaw) + a_y * cos(yaw)) * dt;
  double next_yaw = yaw + a_psi * dt;
  next_pose.pose.orientation = tf::createQuaternionMsgFromYaw(angles::normalize_angle(next_yaw));
  return next_pose;
}

// void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
// {
//     // 创建新的路径
//     if(goal_received == true){
//         goal_received = false;
//         // nav_msgs::Path new_path;
//         // nav_msgs::Path path;
//         new_path.poses.clear();
//         path.poses.clear();
//     }

//     new_path.header = msg->header;

//     // 获取最后一个点的坐标和方向
//     geometry_msgs::PoseStamped pose_stamped;
//     pose_stamped.header = msg->header;
//     pose_stamped.pose.position = msg->pose.pose.position;
//     pose_stamped.pose.orientation = msg->pose.pose.orientation;

//     // 添加点到路径
//     new_path.poses.push_back(pose_stamped);

//     // 打印最后一个点的坐标和方向
//     geometry_msgs::Point point = pose_stamped.pose.position;
//     geometry_msgs::Quaternion orientation = pose_stamped.pose.orientation;
//     ROS_INFO("Received initial pose: (%f, %f, %f)", point.x, point.y, point.z);
//     ROS_INFO("Orientation: (%f, %f, %f, %f)", orientation.x, orientation.y, orientation.z, orientation.w);

//     // 清空旧路径并更新为新路径
//     path = new_path;
// }

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_prediction_node");
  ros::NodeHandle nh;

  ros::Publisher current_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("current_pose", 10);
  ros::Publisher diff_drive_pub = nh.advertise<geometry_msgs::PoseArray>("diff_drive_trajectory", 10);
  ros::Publisher ackermann_pub = nh.advertise<geometry_msgs::PoseArray>("ackermann_trajectory", 10);
  ros::Publisher kinematic_bicycle_pub = nh.advertise<geometry_msgs::PoseArray>("kinematic_bicycle_trajectory", 10);
  ros::Publisher dynamic_bicycle_pub = nh.advertise<geometry_msgs::PoseArray>("dynamic_bicycle_trajectory", 10);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("vehicle_marker", 10);
  // ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("vehicle_h", 10);
  ros::Publisher angle_pub = nh.advertise<std_msgs::Float64>("/vehicle_angle", 10);

  // 订阅定位和control节点 ：
  // ros::Subscriber location_sub_ = nh.subscribe("/map_pose", 10, LocationCallback);
  // ros::Subscriber location_sub_ = nh.subscribe("/map_pose", 10, LocationCallback);
  // ros::Subscriber control_sub_ = nh.subscribe("/path_speed_tracking_data", 10, ControlCallback);
  //   ros::Subscriber initial_pose_subscriber = nh.subscribe("/initialpose", 1, initialPoseCallback);

  // 创建tf广播器
  tf::TransformBroadcaster br;

  // 初始位置和速度
  geometry_msgs::PoseStamped current_pose;
  current_pose.header.frame_id = "map";
  current_pose.pose.position.x = 0.0;
  current_pose.pose.position.y = 0.0;
  current_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  double v = 1.0;  // 线速度
  double ws_diff = 0.5;  // 差速轮模型的角速度
  double ws_ackermann = 0.5;  // 阿克曼模型的转向角
  double dt = 0.1;  // 时间步长
  double wheel_base = 1.0;  // 阿克曼模型的轴距

  // 自行车模型参数
  double L = 1.0;  // 轴距
  double L_f = 0.5;  // 重心到前轴的距离
  double L_r = 0.5;  // 重心到后轴的距离
  double C_alpha = 1.0;  // 轮胎侧偏刚度
  double m = 1500.0;  // 车辆质量
  double I_z = 3000.0;  // 车辆绕z轴的转动惯量

  ros::Rate rate(10);
  double time = 0.0;

  while (ros::ok()) {
    // 发布当前位姿
    current_pose.header.stamp = ros::Time::now();
    current_pose_pub.publish(current_pose);

    // 更新角速度和转向角
    ws_diff = 0.5 * sin(time);
    ws_ackermann = 0.5 * sin(time);

    // 预测差速轮模型的轨迹
    geometry_msgs::PoseArray diff_drive_trajectory;
    diff_drive_trajectory.header.stamp = ros::Time::now();
    diff_drive_trajectory.header.frame_id = "map";
    geometry_msgs::PoseStamped diff_drive_pose = current_pose;
    for (int i = 0; i < 50; ++i) {
      diff_drive_pose = predict(diff_drive_pose, v, ws_diff, dt, wheel_base, false);
      diff_drive_trajectory.poses.push_back(diff_drive_pose.pose);
    }
    diff_drive_pub.publish(diff_drive_trajectory);

    // 预测阿克曼模型的轨迹
    geometry_msgs::PoseArray ackermann_trajectory;
    ackermann_trajectory.header.stamp = ros::Time::now();
    ackermann_trajectory.header.frame_id = "map";
    geometry_msgs::PoseStamped ackermann_pose = current_pose;
    for (int i = 0; i < 50; ++i) {
      ackermann_pose = predict(ackermann_pose, v, ws_ackermann, dt, wheel_base, true);
      ackermann_trajectory.poses.push_back(ackermann_pose.pose);
    }
    ackermann_pub.publish(ackermann_trajectory);
    // std::cout<<"ws_ackermann"<<ws_ackermann<<std::endl;

    std_msgs::Float64 angle_msg;
    // 发布车轮转角
    angle_msg.data = ws_ackermann*180/M_PI;
    angle_pub.publish(angle_msg);


    // 预测自行车运动学模型的轨迹
    geometry_msgs::PoseArray kinematic_bicycle_trajectory;
    kinematic_bicycle_trajectory.header.stamp = ros::Time::now();
    kinematic_bicycle_trajectory.header.frame_id = "map";
    geometry_msgs::PoseStamped kinematic_bicycle_pose = current_pose;
    for (int i = 0; i < 50; ++i) {
      kinematic_bicycle_pose = predict_kinematic_bicycle(kinematic_bicycle_pose, v, ws_ackermann, dt, L);
      kinematic_bicycle_trajectory.poses.push_back(kinematic_bicycle_pose.pose);
    }
    kinematic_bicycle_pub.publish(kinematic_bicycle_trajectory);

    // 预测自行车动力学模型的轨迹
    geometry_msgs::PoseArray dynamic_bicycle_trajectory;
    dynamic_bicycle_trajectory.header.stamp = ros::Time::now();
    dynamic_bicycle_trajectory.header.frame_id = "map";
    geometry_msgs::PoseStamped dynamic_bicycle_pose = current_pose;
    for (int i = 0; i < 50; ++i) {
      dynamic_bicycle_pose = predict_dynamic_bicycle(dynamic_bicycle_pose, v, ws_ackermann, dt, L, L_f, L_r, C_alpha, m, I_z);
      dynamic_bicycle_trajectory.poses.push_back(dynamic_bicycle_pose.pose);
    }
    dynamic_bicycle_pub.publish(dynamic_bicycle_trajectory);

    // 更新当前位置
    current_pose = predict(ackermann_pose, v, ws_ackermann, dt, wheel_base, true);

    // 根据控制指令更新车辆位置ws_ackermann dt 
    // current_pose = predict(ackermann_pose, v, controlMsg.target_steering_angle, dt, wheel_base, true);


    // 发布车辆的marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "base_link";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = current_pose.pose;
    marker.scale.x = 1.0;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    marker_pub.publish(marker);

    // 广播车辆的tf变换
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(current_pose.pose.position.x, current_pose.pose.position.y, 0.0));
    tf::Quaternion q;
    tf::quaternionMsgToTF(current_pose.pose.orientation, q);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

    time += dt;
    rate.sleep();
  }

  return 0;
}
