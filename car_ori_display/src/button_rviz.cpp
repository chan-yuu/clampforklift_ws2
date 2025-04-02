/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-09-13 11:20:19
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2024-09-15 14:12:57
 * @FilePath: /src/car_ori_display/src/button_rviz.cpp
 * @Description: 快捷启动的rviz按钮
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#include "car_ori_display/button_rviz.h"
#include <pluginlib/class_list_macros.h>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QProcess>
#include <QDebug>
#include <QFileInfo>
#include <QDir>
#include <QLabel>

namespace button_rviz
{

AutoButton::AutoButton(QWidget* parent)
  : rviz::Panel(parent)
{
  // 设置主布局为垂直布局
  QVBoxLayout* layout = new QVBoxLayout;

  // 定义每个按钮的名称
  QStringList function_names = {
    "Process 1", "Process 2", "Process 3", "Process 4", "Process 5"
  };
  QStringList start_names = {
    "Start", "Start", "Start", "Start", "Start"
  };
  QStringList stop_names = {
    "Stop", "Stop", "Stop", "Stop", "Stop"
  };

  // 定义按钮大小和样式
  // QSize button_size(120, 40);  // 按钮的大小
  QSize button_size(100, 30);  // 设置按钮的固定大小

  // 系统样式按钮，不定义自定义颜色
  QString default_button_style = "";  // 默认系统样式

  int label_width = 120;  // 设置标签的宽度

  // 初始化 start_buttons_ 和 stop_buttons_ 的大小
  start_buttons_.resize(5);
  stop_buttons_.resize(5);

  for (int i = 0; i < 5; ++i)
  {
    // 为每一行创建水平布局
    QHBoxLayout* row_layout = new QHBoxLayout;

    // 创建标签
    QLabel* label = new QLabel(function_names[i]);
    label->setFixedWidth(label_width);  // 固定标签的宽度
    label->setAlignment(Qt::AlignCenter);  // 居中显示

    // 创建启动和停止按钮
    start_buttons_[i] = new QPushButton(start_names[i]);
    stop_buttons_[i] = new QPushButton(stop_names[i]);

    // 设置按钮大小
    start_buttons_[i]->setFixedSize(button_size);
    start_buttons_[i]->setStyleSheet(default_button_style);  // 系统默认样式

    stop_buttons_[i]->setFixedSize(button_size);
    stop_buttons_[i]->setStyleSheet(default_button_style);  // 系统默认样式

    // 将标签和按钮添加到行布局
    row_layout->addWidget(label);
    row_layout->addWidget(start_buttons_[i]);
    row_layout->addWidget(stop_buttons_[i]);

    // 将每一行的布局添加到主布局
    layout->addLayout(row_layout);

    // 连接启动和停止按钮的点击事件到相应的槽函数
    connect(start_buttons_[i], &QPushButton::clicked, this, [this, i]() { startCommand(i); });
    connect(stop_buttons_[i], &QPushButton::clicked, this, [this, i]() { stopCommand(i); });

    // 设置对象名称，用于区分启动和停止按钮
    start_buttons_[i]->setObjectName(QString("start%1").arg(i + 1));
    stop_buttons_[i]->setObjectName(QString("stop%1").arg(i + 1));
  }

  setLayout(layout);  // 设置主布局
}

void AutoButton::onInitialize()
{
  // 如果有需要初始化的内容，可以在这里添加
}

void AutoButton::startCommand(int index)
{
  QString command;
  switch (index)
  {
    case 0:
      command = "gnome-terminal --tab --title=\"Process 1\" -- bash -c \"./process1.sh;\"";
      break;
    case 1:
      command = "gnome-terminal --tab --title=\"Process 2\" -- bash -c \"./process2.sh;\"";
      break;
    case 2:
      command = "gnome-terminal --tab --title=\"Process 3\" -- bash -c \"./process3.sh;\"";
      break;
    case 3:
      command = "gnome-terminal --tab --title=\"Process 4\" -- bash -c \"./process4.sh;\"";
      break;
    case 4:
      command = "gnome-terminal --tab --title=\"Process 5\" -- bash -c \"./process5.sh;\"";
      break;
  }

  if (!command.isEmpty())
  {
    bool success = QProcess::startDetached(command);
    if (success)
    {
      // 启动成功后，修改启动按钮样式
      start_buttons_[index]->setStyleSheet("background-color: #5e35b1; color: white;");
      qDebug() << "Started command:" << command;
    }
    else
    {
      qDebug() << "Failed to start command:" << command;
    }
  }
}

void AutoButton::stopCommand(int index)
{
  QString command;
  switch (index)
  {
    case 0:
      command = "killall -q process1.sh";
      break;
    case 1:
      command = "killall -q process2.sh";
      break;
    case 2:
      command = "killall -q process3.sh";
      break;
    case 3:
      command = "killall -q process4.sh";
      break;
    case 4:
      command = "killall -q process5.sh";
      break;
  }

  if (!command.isEmpty())
  {
    QProcess::execute(command);
    // 停止进程后，恢复启动按钮的样式为默认样式
    start_buttons_[index]->setStyleSheet("");
    qDebug() << "Stopped command:" << command;
  }
}

} // end namespace button_rviz

PLUGINLIB_EXPORT_CLASS(button_rviz::AutoButton, rviz::Panel)
