/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-09-04 21:41:39
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2024-09-12 21:39:52
 * @FilePath: /undefined/home/kemove/forklift_sim_ws2/src/car_ori_display/include/car_ori_display/gps_imu_display.h
 * @Description: 
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#ifndef GPS_IMU_DISPLAY_H
#define GPS_IMU_DISPLAY_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <nav_msgs/Odometry.h>
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>
#include <QGridLayout>
#include <tf/tf.h>

namespace car_ori_display
{

class LocationDisplay : public rviz::Panel
{
Q_OBJECT
public:
  LocationDisplay(QWidget* parent = 0);

protected:
  virtual void onInitialize();
  void gpsImuCallback(const nav_msgs::Odometry::ConstPtr& msg);

private Q_SLOTS:
  void updateTopic();

private:
  QPushButton* createButton(const QString& text);
  void updateButton(QPushButton* button, const QString& text);

  // UI Elements
  QLineEdit* topic_input_;
  QLabel* current_topic_label_;

  // Buttons for position data
  QPushButton* position_x_button_;
  QPushButton* position_y_button_;
  QPushButton* yaw_button_;

  // Buttons for frame_id
  QPushButton* frame_id_button_;

  // ROS-related
  ros::NodeHandle nh_;
  ros::Subscriber gps_imu_sub_;
};

} // end namespace car_ori_display

#endif // GPS_IMU_DISPLAY_H