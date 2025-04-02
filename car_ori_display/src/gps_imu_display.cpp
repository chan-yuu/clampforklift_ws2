/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-09-04 21:42:03
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2024-10-13 17:54:21
 * @FilePath: /undefined/home/cyun/10.11_ws/src/car_ori_display/src/gps_imu_display.cpp
 * @Description: 用于rviz显示定位信息
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-09-04 21:42:03
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2024-09-13 12:02:41
 * @FilePath: /include/home/nvidia/clamp_forklift_ws/src/car_ori_display/src/gps_imu_display.cpp
 * @Description: 用于rviz显示定位信息
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#include "car_ori_display/gps_imu_display.h"
#include <pluginlib/class_list_macros.h>
#include <QGridLayout>
#include <QLabel>
#include <QDebug>

namespace car_ori_display
{

LocationDisplay::LocationDisplay(QWidget* parent)
  : rviz::Panel(parent)
{
  QGridLayout* layout = new QGridLayout;

  // Setup topic input
  topic_input_ = new QLineEdit;
  topic_input_->setPlaceholderText("Enter topic name...");
  topic_input_->setStyleSheet("QLineEdit { padding: 2px; border-radius: 3px; border: 1px solid #5e35b1; }");

  // Display current subscribed topic
  current_topic_label_ = new QLabel("Current Topic: ");
  current_topic_label_->setStyleSheet("QLabel { color : #5e35b1; padding: 2px; }");

  // Setup buttons for position
  position_x_button_ = createButton("X: 0.0");
  position_y_button_ = createButton("Y: 0.0");
  yaw_button_ = createButton("Yaw: 0.0");

  // Setup buttons for frame_id
  frame_id_button_ = createButton("Frame: ");

  // Adjust buttons to stretch and align within the grid layout
  layout->setColumnStretch(0, 1);
  layout->setColumnStretch(1, 1);

  // Add widgets to layout
  layout->addWidget(topic_input_, 0, 0, 1, 2);
  layout->addWidget(current_topic_label_, 1, 0, 1, 2);
  layout->addWidget(position_x_button_, 2, 0);
  layout->addWidget(position_y_button_, 2, 1);
  layout->addWidget(yaw_button_, 3, 0);
  layout->addWidget(frame_id_button_, 3, 1);

  connect(topic_input_, SIGNAL(returnPressed()), this, SLOT(updateTopic()));

  setLayout(layout);
}

void LocationDisplay::onInitialize()
{
  gps_imu_sub_ = nh_.subscribe("/odom", 1, &LocationDisplay::gpsImuCallback, this);
  current_topic_label_->setText("Current Topic: /odom");
}

QPushButton* LocationDisplay::createButton(const QString& text)
{
  QPushButton* button = new QPushButton(text);
  button->setCheckable(true);
  button->setStyleSheet("QPushButton { background-color : #f0f0f0; color : #1e88e5; border: 1px solid #1e88e5; padding: 2px; border-radius: 2px; }"
                        "QPushButton:checked { background-color : #5e35b1; color : white; }");
  return button;
}

void LocationDisplay::updateButton(QPushButton* button, const QString& text)
{
  button->setText(text);
}

void LocationDisplay::gpsImuCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  updateButton(position_x_button_, "X: " + QString::number(msg->pose.pose.position.x, 'f', 2));
  updateButton(position_y_button_, "Y: " + QString::number(msg->pose.pose.position.y, 'f', 2));
  updateButton(yaw_button_, "Yaw: " + QString::number(tf::getYaw(msg->pose.pose.orientation), 'f', 2));
  updateButton(frame_id_button_, "Frame: " + QString(msg->child_frame_id.c_str()));
}

void LocationDisplay::updateTopic()
{
  QString topic = topic_input_->text();
  if (!topic.isEmpty())
  {
    gps_imu_sub_ = nh_.subscribe(topic.toStdString(), 1, &LocationDisplay::gpsImuCallback, this);
    current_topic_label_->setText("Current Topic: " + topic);
  }
}

} // end namespace car_ori_display

PLUGINLIB_EXPORT_CLASS(car_ori_display::LocationDisplay, rviz::Panel)