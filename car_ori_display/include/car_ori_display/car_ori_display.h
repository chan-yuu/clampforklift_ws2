/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-09-11 11:35:24
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2024-09-13 12:15:17
 * @FilePath: /undefined/home/nvidia/clamp_forklift_ws/src/car_ori_display/include/car_ori_display/car_ori_display.h
 * @Description: 
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#ifndef CAR_ORI_DISPLAY_H
#define CAR_ORI_DISPLAY_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <QPushButton>
#include <QGridLayout>

namespace car_ori_display
{

class CarOriDisplay : public rviz::Panel
{
Q_OBJECT
public:
  CarOriDisplay(QWidget* parent = 0);

protected:
  virtual void onInitialize();

public Q_SLOTS:
  void gearStateCallback(const std_msgs::Int8::ConstPtr& msg);
  void brakeStateCallback(const std_msgs::Float64::ConstPtr& msg);
  void throttleStateCallback(const std_msgs::Float64::ConstPtr& msg);
  void steeringStateCallback(const std_msgs::Float64::ConstPtr& msg);
  void clampStateCallback(const std_msgs::Float64::ConstPtr& msg);
  void widthStateCallback(const std_msgs::Float64::ConstPtr& msg);
  void fyStateCallback(const std_msgs::Float64::ConstPtr& msg);
  void lateralStateCallback(const std_msgs::Float64::ConstPtr& msg);

private:
  QPushButton* createButton(const QString& text);
  void updateButton(QPushButton* button, const QString& text);

  // Buttons for state data
  QPushButton* gear_button_;
  QPushButton* brake_button_;
  QPushButton* speed_button_;
  QPushButton* wheelangle_button_;

  QPushButton* clamp_button_;
  QPushButton* width_button_;
  QPushButton* fy_button_;
  QPushButton* lateral_button_;

  // ROS-related
  ros::NodeHandle nh_;
  ros::Subscriber gear_state_sub_;
  ros::Subscriber brake_state_sub_;
  ros::Subscriber throttle_state_sub_;
  ros::Subscriber steering_state_sub_;
  ros::Subscriber clamp_state_sub_;
  ros::Subscriber width_state_sub_;
  ros::Subscriber fy_state_sub_;
  ros::Subscriber lateral_state_sub_;
};

} // end namespace car_ori_display

#endif // CAR_ORI_DISPLAY_H