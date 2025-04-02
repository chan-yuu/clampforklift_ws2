/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-09-25 07:47:58
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2024-10-17 17:30:08
 * @FilePath: /undefined/home/nvidia/10.11_ws/src/perception/src/tf_pub_by_odom.cpp
 * @Description: 仿真使用
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  // std::cout<<"sub odom"<<std::endl;
  // ROS_INFO_ONCE("\033[1;32m sub odom ");
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transform_stamped;

  // 设置变换的时间戳和源目标框架
  transform_stamped.header.stamp = ros::Time::now();
  transform_stamped.header.frame_id = "map";
  transform_stamped.child_frame_id = "base_footprint";

  // 从Odometry消息中提取位置和姿态信息
  transform_stamped.transform.translation.x = msg->pose.pose.position.x;
  transform_stamped.transform.translation.y = msg->pose.pose.position.y;
  transform_stamped.transform.translation.z = msg->pose.pose.position.z;
  transform_stamped.transform.rotation = msg->pose.pose.orientation;

  // 广播从map到base_footprint的变换
  br.sendTransform(transform_stamped);
}

int main(int argc, char** argv)
{
  // 初始化ROS节点
  ros::init(argc, argv, "odom_to_tf");
  ros::NodeHandle n;

  // 创建一个订阅者，监听名为"odom"的话题
  ros::Subscriber sub = n.subscribe("odom_p3d", 10, odomCallback);

  // 进入循环等待回调函数被调用
  ros::spin();

  return 0;
}

