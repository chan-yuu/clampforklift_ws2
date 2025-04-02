/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-09-13 11:20:19
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2024-09-15 14:11:41
 * @FilePath: /src/car_ori_display/src/angle_display.cpp
 * @Description: 用于显示车辆信息的rviz panel
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#include <pluginlib/class_list_macros.h>
#include <rviz/panel.h>
#include <std_msgs/Float64.h>
#include <QVBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include "car_ori_display/angle_display.h"

namespace angle_display
{

AngleDisplay::AngleDisplay(QWidget* parent)
  : rviz::Panel(parent),
    angle_label_(new QLabel("Angle: 0.0")),
    angle_h_label_(new QLabel("angleH: 0.0"))
{
  QVBoxLayout* layout = new QVBoxLayout;
  topic_input_ = new QLineEdit;
  topic_input_->setPlaceholderText("Enter topic name...");
  update_button_ = new QPushButton("Update Topic");
  layout->addWidget(topic_input_);
  layout->addWidget(update_button_);
  layout->addWidget(angle_label_);
  layout->addWidget(angle_h_label_);
  setLayout(layout);

  connect(update_button_, SIGNAL(clicked()), this, SLOT(updateTopic()));
}

void AngleDisplay::onInitialize()
{
  // 初始订阅一个默认的话题
  angle_sub_ = nh_.subscribe("/vehicle_angle", 1, &AngleDisplay::angleCallback, this);
  angle_h_sub_ = nh_.subscribe("/vehicle_heading", 1, &AngleDisplay::hCallback, this);
}

void AngleDisplay::angleCallback(const std_msgs::Float64::ConstPtr& msg)
{
  QString angle_text = QString::number(msg->data, 'f', 2);
  angle_label_->setText("Angle: " + angle_text);
}

void AngleDisplay::hCallback(const std_msgs::Float64::ConstPtr& msg)
{
  QString angle_text = QString::number(msg->data, 'f', 2);
  angle_h_label_->setText("AngleHeading: " + angle_text);
}

void AngleDisplay::updateTopic()
{
  QString topic = topic_input_->text();
  if (!topic.isEmpty())
  {
    angle_sub_ = nh_.subscribe(topic.toStdString(), 1, &AngleDisplay::angleCallback, this);
    angle_h_sub_ = nh_.subscribe("angle_h", 1, &AngleDisplay::hCallback, this);
  }
}

} // end namespace angle_display

PLUGINLIB_EXPORT_CLASS(angle_display::AngleDisplay, rviz::Panel)
