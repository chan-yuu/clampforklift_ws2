/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-09-13 11:20:19
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2024-10-17 17:48:38
 * @FilePath: /undefined/home/nvidia/10.11_ws/src/car_ori_display/src/car_ori_display.cpp
 * @Description: 用于可视化底盘数据
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#include "car_ori_display/car_ori_display.h"
#include <pluginlib/class_list_macros.h>
#include <QGridLayout>
#include <QPushButton>
#include <QDebug>

namespace car_ori_display
{

CarOriDisplay::CarOriDisplay(QWidget* parent)
  : rviz::Panel(parent)
{
  QGridLayout* layout = new QGridLayout;

  int row = 2;

  // 使用 createButton() 创建所有的按钮
  gear_button_ = createButton("档位: 0");
  layout->addWidget(gear_button_, row, 0);
  brake_button_ = createButton("刹车: 0.0");
  layout->addWidget(brake_button_, row++, 1);

  speed_button_ = createButton("速度: 0.0");
  layout->addWidget(speed_button_, row, 0);
  wheelangle_button_ = createButton("转角: 0.0");
  layout->addWidget(wheelangle_button_, row++, 1);

  clamp_button_ = createButton("宽度: 0.0");
  layout->addWidget(clamp_button_, row, 0);
  width_button_ = createButton("高度: 0.0");
  layout->addWidget(width_button_, row++, 1);

  fy_button_ = createButton("俯仰: 0.0");
  layout->addWidget(fy_button_, row, 0);
  lateral_button_ = createButton("横移: 0.0");
  layout->addWidget(lateral_button_, row++, 1);

  setLayout(layout);
}

QPushButton* CarOriDisplay::createButton(const QString& text)
{
  QPushButton* button = new QPushButton(text);
  button->setCheckable(true);
  button->setStyleSheet("QPushButton { background-color : #f0f0f0; color : #1e88e5; border: 1px solid #1e88e5; padding: 2px; border-radius: 2px; }"
                        "QPushButton:checked { background-color : #5e35b1; color : white; }");
  connect(button, &QPushButton::clicked, this, [button]() {
    if (button->isChecked()) {
      button->setStyleSheet("background-color: #5e35b1; color: white; border-radius: 2px; padding: 2px;");
    } else {
      button->setStyleSheet("background-color: #f0f0f0; color: #1e88e5; border: 1px solid #1e88e5; border-radius: 2px; padding: 2px;");
    }
  });
  return button;
}

void CarOriDisplay::onInitialize()
{
  gear_state_sub_ = nh_.subscribe("/gear_state", 1, &CarOriDisplay::gearStateCallback, this);
  brake_state_sub_ = nh_.subscribe("/brake_state", 1, &CarOriDisplay::brakeStateCallback, this);
  throttle_state_sub_ = nh_.subscribe("/throttle_state", 1, &CarOriDisplay::throttleStateCallback, this);
  steering_state_sub_ = nh_.subscribe("/steering_state", 1, &CarOriDisplay::steeringStateCallback, this);
  clamp_state_sub_ = nh_.subscribe("/clamp_state", 1, &CarOriDisplay::clampStateCallback, this);
  width_state_sub_ = nh_.subscribe("/updown_state", 1, &CarOriDisplay::widthStateCallback, this);
  fy_state_sub_ = nh_.subscribe("/fy_state", 1, &CarOriDisplay::fyStateCallback, this);
  lateral_state_sub_ = nh_.subscribe("/lateral_state", 1, &CarOriDisplay::lateralStateCallback, this);
}

void CarOriDisplay::gearStateCallback(const std_msgs::Int8::ConstPtr& msg)
{
  updateButton(gear_button_, "Gear: " + QString::number(msg->data));
}

void CarOriDisplay::brakeStateCallback(const std_msgs::Float64::ConstPtr& msg)
{
  updateButton(brake_button_, "Brake: " + QString::number(msg->data, 'f', 2));
}

void CarOriDisplay::throttleStateCallback(const std_msgs::Float64::ConstPtr& msg)
{
  updateButton(speed_button_, "Speed: " + QString::number(msg->data, 'f', 2));
}

void CarOriDisplay::steeringStateCallback(const std_msgs::Float64::ConstPtr& msg)
{
  updateButton(wheelangle_button_, "Wheelangle: " + QString::number(msg->data, 'f', 2));
}

void CarOriDisplay::clampStateCallback(const std_msgs::Float64::ConstPtr& msg)
{
  updateButton(clamp_button_, "Clamp: " + QString::number(msg->data, 'f', 2));
}

void CarOriDisplay::widthStateCallback(const std_msgs::Float64::ConstPtr& msg)
{
  updateButton(width_button_, "Width: " + QString::number(msg->data, 'f', 2));
}

void CarOriDisplay::fyStateCallback(const std_msgs::Float64::ConstPtr& msg)
{
  updateButton(fy_button_, "FY: " + QString::number(msg->data, 'f', 2));
}

void CarOriDisplay::lateralStateCallback(const std_msgs::Float64::ConstPtr& msg)
{
  updateButton(lateral_button_, "Lateral: " + QString::number(msg->data, 'f', 2));
}

void CarOriDisplay::updateButton(QPushButton* button, const QString& text)
{
  button->setText(text);
}

} // end namespace car_ori_display

PLUGINLIB_EXPORT_CLASS(car_ori_display::CarOriDisplay, rviz::Panel)