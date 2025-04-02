/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-09-13 11:20:19
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2024-09-15 14:15:41
 * @FilePath: /src/car_ori_display/src/calibration_lidar_panel.cpp
 * @Description: 在rviz上进行标定雷达
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#include <pluginlib/class_list_macros.h>
#include <rviz/panel.h>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <ros/ros.h>
#include "car_ori_display/calibration_lidar_panel.h"

namespace calibration_lidar_panel
{

CalibrationLidarPanel::CalibrationLidarPanel(QWidget* parent)
  : rviz::Panel(parent)
{
  QVBoxLayout* layout = new QVBoxLayout;

  // 创建输入框和标签
  x_offset_edit_ = new QLineEdit("0.0");
  y_offset_edit_ = new QLineEdit("0.0");
  z_offset_edit_ = new QLineEdit("0.4");
  roll_offset_edit_ = new QLineEdit("-0.074");
  pitch_offset_edit_ = new QLineEdit("0.0");
  yaw_offset_edit_ = new QLineEdit("-1.57");
  input_topic_edit_ = new QLineEdit("/lslidar_point_cloud");
  output_topic_edit_ = new QLineEdit("/calibrated_point_cloud");

  layout->addWidget(new QLabel("x_offset:"));
  layout->addWidget(x_offset_edit_);
  layout->addWidget(new QLabel("y_offset:"));
  layout->addWidget(y_offset_edit_);
  layout->addWidget(new QLabel("z_offset:"));
  layout->addWidget(z_offset_edit_);
  layout->addWidget(new QLabel("roll_offset:"));
  layout->addWidget(roll_offset_edit_);
  layout->addWidget(new QLabel("pitch_offset:"));
  layout->addWidget(pitch_offset_edit_);
  layout->addWidget(new QLabel("yaw_offset:"));
  layout->addWidget(yaw_offset_edit_);
  layout->addWidget(new QLabel("input_topic:"));
  layout->addWidget(input_topic_edit_);
  layout->addWidget(new QLabel("output_topic:"));
  layout->addWidget(output_topic_edit_);

  // 创建保存按钮
  save_button_ = new QPushButton("Save Calibration Params");
  layout->addWidget(save_button_);

  setLayout(layout);

  connect(save_button_, SIGNAL(clicked()), this, SLOT(saveCalibrationParams()));
}

void CalibrationLidarPanel::onInitialize()
{
  // 如果需要初始化其他内容，可以在这里添加
}

void CalibrationLidarPanel::saveCalibrationParams()
{
  nh_.setParam("calibration/x_offset", x_offset_edit_->text().toDouble());
  nh_.setParam("calibration/y_offset", y_offset_edit_->text().toDouble());
  nh_.setParam("calibration/z_offset", z_offset_edit_->text().toDouble());
  nh_.setParam("calibration/roll_offset", roll_offset_edit_->text().toDouble());
  nh_.setParam("calibration/pitch_offset", pitch_offset_edit_->text().toDouble());
  nh_.setParam("calibration/yaw_offset", yaw_offset_edit_->text().toDouble());
  nh_.setParam("input_topic", input_topic_edit_->text().toStdString());
  nh_.setParam("output_topic", output_topic_edit_->text().toStdString());

  ROS_INFO("Calibration parameters saved.");
}

} // end namespace calibration_lidar_panel

PLUGINLIB_EXPORT_CLASS(calibration_lidar_panel::CalibrationLidarPanel, rviz::Panel)
