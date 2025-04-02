/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-09-13 11:20:19
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2024-10-13 17:50:40
 * @FilePath: /undefined/home/cyun/10.11_ws/src/car_ori_display/src/path_speed_ctrl_display.cpp
 * @Description: 用于显示车辆控制的数据
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#include "car_ori_display/path_speed_ctrl_display.h"
#include <pluginlib/class_list_macros.h>
#include <rviz/panel.h>
#include <QGridLayout>
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>
#include <QDebug>

namespace car_ori_display
{

PathSpeedCtrlDisplay::PathSpeedCtrlDisplay(QWidget* parent)
  : rviz::Panel(parent)
{
  QGridLayout* layout = new QGridLayout;

  // 使用 createButton() 创建所有的按钮
  gear_button_ = createButton("档位: 0");
  layout->addWidget(gear_button_, 0, 0);
  brake_button_ = createButton("刹车: 0.0");
  layout->addWidget(brake_button_, 0, 1);

  speed_button_ = createButton("速度: 0.0");
  layout->addWidget(speed_button_, 1, 0);
  wheelangle_button_ = createButton("转角: 0.0");
  layout->addWidget(wheelangle_button_, 1, 1);

  clamp_button_ = createButton("宽度: 0.0");
  layout->addWidget(clamp_button_, 2, 0);
  width_button_ = createButton("高度: 0.0");
  layout->addWidget(width_button_, 2, 1);

  fy_button_ = createButton("俯仰: 0.0");
  layout->addWidget(fy_button_, 3, 0);
  lateral_button_ = createButton("横移: 0.0");
  layout->addWidget(lateral_button_, 3, 1);

  fy_button_ = createButton("CTE: 0.0");
  layout->addWidget(fy_button_, 4, 0);
  lateral_button_ = createButton("dHead: 0.0");
  layout->addWidget(lateral_button_, 4, 1);

  setLayout(layout);

  gear_sub_ = nh_.subscribe("/gear_cmd", 1, &PathSpeedCtrlDisplay::gearCmdCallback, this);
  brake_sub_ = nh_.subscribe("/brake_cmd", 1, &PathSpeedCtrlDisplay::brakeCmdCallback, this);
  throttle_sub_ = nh_.subscribe("/throttle_cmd", 1, &PathSpeedCtrlDisplay::throttleCmdCallback, this);
  steering_sub_ = nh_.subscribe("/steering_cmd", 1, &PathSpeedCtrlDisplay::steeringCmdCallback, this);
  clamp_sub_ = nh_.subscribe("/clamp_cmd", 1, &PathSpeedCtrlDisplay::clampCmdCallback, this);
  width_sub_ = nh_.subscribe("/updown_cmd", 1, &PathSpeedCtrlDisplay::widthCmdCallback, this);
  fy_sub_ = nh_.subscribe("/fy_cmd", 1, &PathSpeedCtrlDisplay::fyCmdCallback, this);
  lateral_sub_ = nh_.subscribe("/lateral_cmd", 1, &PathSpeedCtrlDisplay::lateralCmdCallback, this);

  cte_sub_ = nh_.subscribe("/cte_state", 1, &PathSpeedCtrlDisplay::cteCallback, this);
  dhead_sub_ = nh_.subscribe("/dhead_state", 1, &PathSpeedCtrlDisplay::dheadCallback, this);
}

QPushButton* PathSpeedCtrlDisplay::createButton(const QString& text)
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

void PathSpeedCtrlDisplay::onInitialize()
{
  // 如果需要初始化其他内容，可以在这里添加
}

void PathSpeedCtrlDisplay::gearCmdCallback(const std_msgs::UInt8::ConstPtr& msg)
{
  updateButton(gear_button_, "Gear: " + QString::number(msg->data));
}

void PathSpeedCtrlDisplay::brakeCmdCallback(const std_msgs::Float64::ConstPtr& msg)
{
  updateButton(brake_button_, "Brake: " + QString::number(msg->data, 'f', 2));
}

void PathSpeedCtrlDisplay::throttleCmdCallback(const std_msgs::Float64::ConstPtr& msg)
{
  updateButton(speed_button_, "Speed: " + QString::number(msg->data, 'f', 2));
}

void PathSpeedCtrlDisplay::steeringCmdCallback(const std_msgs::Float64::ConstPtr& msg)
{
  updateButton(wheelangle_button_, "Wheelangle: " + QString::number(msg->data, 'f', 2));
}

void PathSpeedCtrlDisplay::clampCmdCallback(const std_msgs::Float64::ConstPtr& msg)
{
  updateButton(clamp_button_, "Clamp: " + QString::number(msg->data, 'f', 2));
}

void PathSpeedCtrlDisplay::widthCmdCallback(const std_msgs::Float64::ConstPtr& msg)
{
  updateButton(width_button_, "Width: " + QString::number(msg->data, 'f', 2));
}

void PathSpeedCtrlDisplay::fyCmdCallback(const std_msgs::Float64::ConstPtr& msg)
{
  updateButton(fy_button_, "FY: " + QString::number(msg->data, 'f', 2));
}

void PathSpeedCtrlDisplay::lateralCmdCallback(const std_msgs::Float64::ConstPtr& msg)
{
  updateButton(lateral_button_, "Lateral: " + QString::number(msg->data, 'f', 2));
}

void PathSpeedCtrlDisplay::cteCallback(const std_msgs::Float64::ConstPtr& msg)
{
  updateButton(fy_button_, "CTE: " + QString::number(msg->data, 'f', 2));
}

void PathSpeedCtrlDisplay::dheadCallback(const std_msgs::Float64::ConstPtr& msg)
{
  updateButton(lateral_button_, "dHead: " + QString::number(msg->data, 'f', 2));
}

void PathSpeedCtrlDisplay::updateButton(QPushButton* button, const QString& text)
{
  button->setText(text);
}

} // end namespace car_ori_display

PLUGINLIB_EXPORT_CLASS(car_ori_display::PathSpeedCtrlDisplay, rviz::Panel)