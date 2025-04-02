/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-09-13 11:20:19
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2024-09-15 14:13:19
 * @FilePath: /src/car_ori_display/src/button_rviz2.cpp
 * @Description: 用于启动更多的按钮
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#include "car_ori_display/button_rviz2.h"
#include <pluginlib/class_list_macros.h>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QProcess>
#include <QDebug>
#include <QString>

namespace button_rviz
{

CustomButton::CustomButton(QWidget* parent)
  : rviz::Panel(parent)
{
  QVBoxLayout* layout = new QVBoxLayout;

  // 定义每个功能的名称
  QStringList function_names = {
    "Nvidia SMI",
    "Roscore",
    "Cotton Sim",
    "Cotton Plan",
    "Keyboard Control",
    "Cotton"
  };

  QStringList start_names = {
    "Start",
    "Start",
    "Start",
    "Start",
    "Start",
    "Start"
  };

  QStringList stop_names = {
    "Stop",
    "Stop",
    "Stop",
    "Stop",
    "Stop",
    "Stop"
  };

  QSize button_size(100, 30);  // 设置按钮的固定大小
  int label_width = 120;  // 设置标签的宽度，以确保文字完全显示

  // 初始化 start_buttons_ 和 stop_buttons_ 的大小
  start_buttons_.resize(6);
  stop_buttons_.resize(6);

  for (int i = 0; i < 6; ++i)
  {
    QHBoxLayout* row_layout = new QHBoxLayout;

    QLabel* function_label = new QLabel(function_names[i]);
    function_label->setFixedWidth(label_width);  // 固定标签的宽度
    function_label->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    function_label->setAlignment(Qt::AlignCenter);  // 设置标签居中对齐

    start_buttons_[i] = new QPushButton(start_names[i]);
    stop_buttons_[i] = new QPushButton(stop_names[i]);

    // 设置按钮的固定大小
    start_buttons_[i]->setFixedSize(button_size);
    stop_buttons_[i]->setFixedSize(button_size);

    row_layout->addWidget(function_label);
    row_layout->addWidget(start_buttons_[i], 0, Qt::AlignCenter);  // 确保按钮居中对齐
    row_layout->addWidget(stop_buttons_[i], 0, Qt::AlignCenter);   // 确保按钮居中对齐

    layout->addLayout(row_layout);

    connect(start_buttons_[i], &QPushButton::clicked, this, [this, i]() { startCommand(i); });
    connect(stop_buttons_[i], &QPushButton::clicked, this, [this, i]() { stopCommand(i); });
  }

  setLayout(layout);
}

void CustomButton::onInitialize()
{
  // 如果需要初始化其他内容，可以在这里添加
  qDebug() << "Start button initialized";
}

void CustomButton::startCommand(int index)
{
  QString command;
  switch (index)
  {
    case 0:
      command = "gnome-terminal --tab --title=\"NVIDIA SMI\" -- bash -c \"watch -n 1 nvidia-smi;\"";
      break;
    case 1:
      command = "gnome-terminal --tab --title=\"Roscore\" -- bash -c \"source ~/.carla_bash.rc;source /home/kemove/forklift_sim_ws/devel/setup.bash; roscore;\"";
      break;
    case 2:
      command = "gnome-terminal --tab --title=\"Cotton Sim\" -- bash -c \"cd /home/cyun/forklift_sim_ws/scripts;./sim1.sh;\"";
      break;
    case 3:
      command = "gnome-terminal --tab --title=\"Cotton Plan\" -- bash -c \"cd /home/cyun/forklift_sim_ws/scripts;./sim2.sh;\"";
      break;
    case 4:
      command = "gnome-terminal --tab --title=\"Keyboard Control\" -- bash -c \"source ~/.carla_bash.rc;source /home/kemove/forklift_sim_ws/devel/setup.bash;rosrun joy_control cotton_keyboard_control.py;\"";
      break;
    case 5:
      command = "gnome-terminal --tab --title=\"Cotton\" -- bash -c \"source /home/kemove/forklift_sim_ws/devel/setup.bash;roslaunch forklift_gazebo spawn_fork.launch;\"";
      break;
  }

  if (!command.isEmpty())
  {
    bool success = QProcess::startDetached(command);
    if (success)
    {
      start_buttons_[index]->setStyleSheet("background-color: #5e35b1; color: white;");
      qDebug() << "Started command:" << command;
    }
    else
    {
      qDebug() << "Failed to start command:" << command;
    }
  }
}

void CustomButton::stopCommand(int index)
{
  QString command;
  switch (index)
  {
    case 0:
      command = "killall -q watch";
      break;
    case 1:
      command = "killall -q roscore";
      break;
    case 2:
      command = "gnome-terminal --tab --title=\"Cotton\" -- bash -c \"pkill -f 'ndt_localizer.launch';\"";
      break;
    case 3:
      command = "pkill -f 'run_hybrid_a_star.launch'";
      break;
    case 4:
      command = "killall -q rosrun";
      break;
    case 5:
      command = "killall -q roslaunch";
      break;
  }

  if (!command.isEmpty())
  {
    QProcess::execute(command);
    stop_buttons_[index]->setStyleSheet("");
    start_buttons_[index]->setStyleSheet("");  // 恢复 start 按钮的颜色
    qDebug() << "Stopped command:" << command;
  }
}

} // end namespace button_rviz

// 插件的注册宏，必须存在
PLUGINLIB_EXPORT_CLASS(button_rviz::CustomButton, rviz::Panel)
