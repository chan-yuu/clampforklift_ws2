/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-09-01 23:46:56
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2024-10-07 10:43:02
 * @FilePath: /undefined/home/cyun/forklift_sim_ws/src/car_ori_display/include/car_ori_display/path_speed_ctrl_display.h
 * @Description: 
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#ifndef PATH_SPEED_CTRL_DISPLAY_H
#define PATH_SPEED_CTRL_DISPLAY_H

#include <rviz/panel.h>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>

#include <std_msgs/Float64.h>

namespace car_ori_display
{

class PathSpeedCtrlDisplay : public rviz::Panel
{
Q_OBJECT
public:
  PathSpeedCtrlDisplay(QWidget* parent = 0);

protected:
  virtual void onInitialize();

private Q_SLOTS:
  void gearCmdCallback(const std_msgs::UInt8::ConstPtr& msg);
  void brakeCmdCallback(const std_msgs::Float64::ConstPtr& msg);
  void throttleCmdCallback(const std_msgs::Float64::ConstPtr& msg);
  void steeringCmdCallback(const std_msgs::Float64::ConstPtr& msg);
  void cteStateCallback(const std_msgs::Float64::ConstPtr& msg);
  void dheadStateCallback(const std_msgs::Float64::ConstPtr& msg);
  void clampCmdCallback(const std_msgs::Float64::ConstPtr& msg);
  void widthCmdCallback(const std_msgs::Float64::ConstPtr& msg);
  void fyCmdCallback(const std_msgs::Float64::ConstPtr& msg);
  void lateralCmdCallback(const std_msgs::Float64::ConstPtr& msg);
  void cteCallback(const std_msgs::Float64::ConstPtr& msg);
  void dheadCallback(const std_msgs::Float64::ConstPtr& msg);

private:
  QPushButton* createButton(const QString& text);
  void updateButton(QPushButton* button, const QString& text);

  // UI Elements
  QPushButton* gear_button_;
  QPushButton* brake_button_;
  QPushButton* speed_button_;
  QPushButton* wheelangle_button_;
  QPushButton* cte_button_;
  QPushButton* dhead_button_;
  QPushButton* clamp_button_;
  QPushButton* width_button_;
  QPushButton* fy_button_;
  QPushButton* lateral_button_;

  // ROS-related
  ros::NodeHandle nh_;
  ros::Subscriber gear_sub_;
  ros::Subscriber brake_sub_;
  ros::Subscriber throttle_sub_;
  ros::Subscriber steering_sub_;
  ros::Subscriber cte_state_sub_;
  ros::Subscriber dhead_state_sub_;
  ros::Subscriber clamp_sub_;
  ros::Subscriber width_sub_;
  ros::Subscriber fy_sub_;
  ros::Subscriber lateral_sub_;
  
  ros::Subscriber cte_sub_;
  ros::Subscriber dhead_sub_;
};

} // end namespace car_ori_display

#endif // PATH_SPEED_CTRL_DISPLAY_H