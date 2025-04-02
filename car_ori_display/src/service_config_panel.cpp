/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-09-13 11:12:05
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2024-09-15 14:21:49
 * @FilePath: /src/car_ori_display/src/service_config_panel.cpp
 * @Description: 服务的快捷创建插件（用于任务调度使用，涉及到决策后不再适用）
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#include "car_ori_display/service_config_panel.h"
#include <QVBoxLayout>
#include <QLabel>
#include <QFile>
#include <QTextStream>
#include <QProcess> // 用于启动外部进程
#include <ros/ros.h>

namespace car_ori_display
{

ServiceConfigPanel::ServiceConfigPanel(QWidget* parent)
  : rviz::Panel(parent), file_path_("/home/kemove/smach_config.txt")
{
  // 初始化服务选择器和参数输入框
  service_selector_ = new QComboBox;
  service_selector_->addItem("SetHeight");  // 设置高度服务
  service_selector_->addItem("SetWidth");   // 设置宽度服务
  service_selector_->addItem("SetStartPose"); // 设置起点服务
  service_selector_->addItem("SetGoalPose");  // 设置终点服务

  param_input1_ = new QLineEdit; // 输入位置X (p_x)
  param_input2_ = new QLineEdit; // 输入位置Y (p_y)
  param_input3_ = new QLineEdit; // 输入位置Z (p_z)
  param_input4_ = new QLineEdit; // 输入方向X (o_x)
  param_input5_ = new QLineEdit; // 输入方向Y (o_y)
  param_input6_ = new QLineEdit; // 输入方向Z (o_z)

  save_button_ = new QPushButton("保存服务配置");
  trigger_button_ = new QPushButton("启动状态机");

  connect(save_button_, SIGNAL(clicked()), this, SLOT(saveServiceConfig()));
  connect(trigger_button_, SIGNAL(clicked()), this, SLOT(triggerStateMachine()));

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(new QLabel("选择服务"));
  layout->addWidget(service_selector_);

  // 为起点/终点服务添加位姿输入框
  layout->addWidget(new QLabel("输入位置X (p_x)"));
  layout->addWidget(param_input1_);
  layout->addWidget(new QLabel("输入位置Y (p_y)"));
  layout->addWidget(param_input2_);
  layout->addWidget(new QLabel("输入位置Z (p_z)"));
  layout->addWidget(param_input3_);
  layout->addWidget(new QLabel("输入方向X (o_x)"));
  layout->addWidget(param_input4_);
  layout->addWidget(new QLabel("输入方向Y (o_y)"));
  layout->addWidget(param_input5_);
  layout->addWidget(new QLabel("输入方向Z (o_z)"));
  layout->addWidget(param_input6_);

  layout->addWidget(save_button_);
  layout->addWidget(trigger_button_);

  setLayout(layout);
}

void ServiceConfigPanel::saveServiceConfig()
{
  QString service = service_selector_->currentText();
  QString p_x = param_input1_->text();
  QString p_y = param_input2_->text();
  QString p_z = param_input3_->text();
  QString o_x = param_input4_->text();
  QString o_y = param_input5_->text();
  QString o_z = param_input6_->text();

  QFile file(file_path_);
  if (file.open(QIODevice::WriteOnly | QIODevice::Append)) {
    QTextStream stream(&file);
    if (service == "SetHeight" || service == "SetWidth") {
      stream << service << " " << p_x << "\n";
    } else if (service == "SetStartPose" || service == "SetGoalPose") {
      stream << service << " " << p_x << " " << p_y << " " << p_z << " "
             << o_x << " " << o_y << " " << o_z << "\n";
    }
    file.close();
  }
}

void ServiceConfigPanel::triggerStateMachine()
{
  // 使用 QProcess 启动 gnome-terminal 并运行 source 和 roslaunch 命令
  QProcess *process = new QProcess(this);
  QString program = "gnome-terminal";  // 使用 gnome-terminal 启动新的终端
  QStringList arguments;
  QString workspace_setup = "source /home/kemove/forklift_sim_ws2/devel/setup.bash";  // 替换为实际的工作空间路径
  QString launch_command = "roslaunch smach_fork smach_service_auto.launch";

  // 在新的 gnome-terminal 中运行 source 和 roslaunch 命令
  arguments << "--" << "bash" << "-c" << QString("%1 && %2; exec bash").arg(workspace_setup).arg(launch_command);

  // 启动进程
  process->start(program, arguments);

  if (process->waitForStarted()) {
    ROS_INFO("State machine started successfully in a new terminal.");
  } else {
    ROS_ERROR("Failed to start the state machine.");
  }

  // 将进程连接到槽，用于在进程结束时执行清理操作
  connect(process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          [process](int exitCode, QProcess::ExitStatus exitStatus) {
              if (exitStatus == QProcess::NormalExit && exitCode == 0) {
                  ROS_INFO("State machine completed successfully.");
              } else {
                  ROS_ERROR("State machine failed with exit code %d.", exitCode);
              }
              process->deleteLater();
          });
}

} // namespace car_ori_display

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(car_ori_display::ServiceConfigPanel, rviz::Panel)
