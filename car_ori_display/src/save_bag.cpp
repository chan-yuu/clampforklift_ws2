/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-09-13 11:20:19
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2024-09-15 14:19:06
 * @FilePath: /src/car_ori_display/src/save_bag.cpp
 * @Description: 用于rviz中存储点云bag包
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#include <pluginlib/class_list_macros.h>
#include <rviz/panel.h>
#include <QVBoxLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QProcess>
#include <QDir>
#include <QDebug>
#include "car_ori_display/save_bag.h"

namespace save_bag
{

SaveBag::SaveBag(QWidget* parent)
  : rviz::Panel(parent),
    bag_process_id_("")
{
  QVBoxLayout* layout = new QVBoxLayout;
  topic_input_ = new QLineEdit;
  topic_input_->setPlaceholderText("输入想要存储的话题...");
  file_name_input_ = new QLineEdit;
  file_name_input_->setPlaceholderText("输入文件名...");
  save_directory_input_ = new QLineEdit;
  save_directory_input_->setPlaceholderText("输入保存目录（默认~/Documents/data）...");
  collect_all_button_ = new QPushButton("收集所有数据");
  collect_topic_button_ = new QPushButton("收集特定话题数据");
  save_button_ = new QPushButton("保存");
  plotjuggler_button_ = new QPushButton("运行 PlotJuggler");

  layout->addWidget(topic_input_);
  layout->addWidget(file_name_input_);
  layout->addWidget(save_directory_input_);
  layout->addWidget(collect_all_button_);
  layout->addWidget(collect_topic_button_);
  layout->addWidget(save_button_);
  layout->addWidget(plotjuggler_button_);
  setLayout(layout);

  connect(collect_all_button_, SIGNAL(clicked()), this, SLOT(collectAll()));
  connect(collect_topic_button_, SIGNAL(clicked()), this, SLOT(collectTopic()));
  connect(save_button_, SIGNAL(clicked()), this, SLOT(saveBag()));
  connect(plotjuggler_button_, SIGNAL(clicked()), this, SLOT(runPlotJuggler()));
}

void SaveBag::onInitialize()
{
  // 如果需要初始化其他内容，可以在这里添加
}

void SaveBag::collectAll()
{
  if (!bag_process_id_.isEmpty())
  {
    QProcess::execute("kill " + bag_process_id_);
    bag_process_id_ = "";
  }

  QString file_name = file_name_input_->text();
  if (file_name.isEmpty())
  {
    file_name = "default_bag";
  }

  QString save_directory = save_directory_input_->text();
  if (save_directory.isEmpty())
  {
    save_directory = QDir::homePath() + "/Documents/data/";
  }
  else
  {
    if (!save_directory.endsWith("/"))
    {
      save_directory += "/";
    }
  }

  QDir().mkpath(save_directory);  // 确保目录存在
  QString output_file = save_directory + file_name + ".bag";
  QString command = "rosbag record -a -O " + output_file;
  QProcess* process = new QProcess(this);
  process->start(command);

  if (process->waitForStarted())
  {
    bag_process_id_ = QString::number(process->processId());
    qDebug() << "Started collecting all data with process ID:" << bag_process_id_;
  }
  else
  {
    qDebug() << "Failed to start collecting all data";
  }
}

void SaveBag::collectTopic()
{
  if (!bag_process_id_.isEmpty())
  {
    QProcess::execute("kill " + bag_process_id_);
    bag_process_id_ = "";
  }

  QString topic = topic_input_->text();
  if (!topic.isEmpty())
  {
    QString file_name = file_name_input_->text();
    if (file_name.isEmpty())
    {
      file_name = "default_bag";
    }

    QString save_directory = save_directory_input_->text();
    if (save_directory.isEmpty())
    {
      save_directory = QDir::homePath() + "/Documents/data/";
    }
    else
    {
      if (!save_directory.endsWith("/"))
      {
        save_directory += "/";
      }
    }

    QDir().mkpath(save_directory);  // 确保目录存在
    QString output_file = save_directory + file_name + ".bag";
    QString command = "rosbag record " + topic + " -O " + output_file;
    QProcess* process = new QProcess(this);
    process->start(command);

    if (process->waitForStarted())
    {
      bag_process_id_ = QString::number(process->processId());
      qDebug() << "Started collecting topic data with process ID:" << bag_process_id_;
    }
    else
    {
      qDebug() << "Failed to start collecting topic data";
    }
  }
  else
  {
    qDebug() << "No topic specified";
  }
}

void SaveBag::saveBag()
{
  if (!bag_process_id_.isEmpty())
  {
    QProcess::execute("kill " + bag_process_id_);
    bag_process_id_ = "";
  }

  qDebug() << "Bag collection stopped and saved.";
}

void SaveBag::runPlotJuggler()
{
  QProcess* process = new QProcess(this);
  QString command = "rosrun plotjuggler plotjuggler";
  process->start(command);

  if (process->waitForStarted())
  {
    qDebug() << "PlotJuggler started successfully";
  }
  else
  {
    qDebug() << "Failed to start PlotJuggler";
  }
}

} // end namespace save_bag

PLUGINLIB_EXPORT_CLASS(save_bag::SaveBag, rviz::Panel)
