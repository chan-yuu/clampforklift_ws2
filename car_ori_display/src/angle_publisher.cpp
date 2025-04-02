/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-09-13 11:20:19
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2024-09-15 14:12:09
 * @FilePath: /src/car_ori_display/src/angle_publisher.cpp
 * @Description: 用于发布指定角度的rviz panel
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "angle_publisher");
  ros::NodeHandle nh;

  ros::Publisher angle_pub = nh.advertise<std_msgs::Float64>("/vehicle_heading", 10);
  tf::TransformListener listener;
  ros::Rate rate(10.0);

  while (nh.ok())
  {
    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
      double yaw = tf::getYaw(transform.getRotation());

      std_msgs::Float64 angle_msg;
      // 将接受到的tf转化为角度重新发送出去可视化
      angle_msg.data = yaw*180/M_PI;
      angle_pub.publish(angle_msg);
    }
    catch (tf::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
    rate.sleep();
  }
  return 0;
}
