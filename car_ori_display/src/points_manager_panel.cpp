/*
 * @Author: JLB 
 * @Date: 2024-09-14 15:42:13
 * @LastEditors: JLB 
 * @LastEditTime: 2024-09-15 14:18:09
 * @FilePath: /src/car_ori_display/src/points_manager_panel.cpp
 * @Description: 用于快捷采集rviz中的起点和终点的位置信息
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#include <pluginlib/class_list_macros.h>
#include <QAction>
#include <QClipboard>
#include <QLayout>
#include <QPalette>
#include <QStyle>
#include <QHeaderView>
#include <QInputDialog>
#include <QLineEdit>
#include <QMessageBox>
#include <QApplication>
#include <QFileDialog>
#include <QTextStream>
#include <QDebug>

#include <unistd.h>

#include "car_ori_display/points_manager_panel.h"
namespace points_manager_panel
{

PointsManagerPanel::PointsManagerPanel(QWidget* parent)
  : rviz::Panel(parent)
{
  Init();
}

void PointsManagerPanel::onInitialize()
{
  // 如果需要初始化其他内容，可以在这里添加
}

void PointsManagerPanel::Init()
{
  initial_pose_callback_enable_ = false;
  std::string points_file_path;
  nh_.getParam("/car_ori_display_config_path", points_file_path);

  odom_sub_ = nh_.subscribe("/odom", 1, &PointsManagerPanel::OdomCallBack, this);
  //map_pose_sub_ = nh_.subscribe("/map_pose", 1, &PointsManagerPanel::MapPoseCallBack, this);
  initial_pose_sub_ = nh_.subscribe("/initialpose", 1, &PointsManagerPanel::InitialPoseCallBack, this);
  end_pose_sub_  = nh_.subscribe("/move_base_simple/goal", 1, &PointsManagerPanel::EndPoseCallBack, this);

  initial_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
  end_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

  arrow_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/points_manager_panel/point", 1);
  timer_pub_arrow_marker_ = new QTimer();
  timer_pub_arrow_marker_->setInterval(200);
  timer_pub_arrow_marker_->start();
  connect(timer_pub_arrow_marker_, &QTimer::timeout, this, &PointsManagerPanel::SlotFunctionTimerPubArrowMarker);
  ui.table_widget_points = new QTableWidget();
  QStringList str_list_labels;
  str_list_labels << "class" << "pos/x" << "pos/y"<<"pos/z"<<"ori/x"<<"ori/y"<<"ori/z"<<"ori/w";
  ui.table_widget_points->setColumnCount(8);
  //ui.table_widget_points->setFont(font);
  ui.table_widget_points->setHorizontalHeaderLabels(str_list_labels);
  ui.table_widget_points->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  ui.table_widget_points->setEditTriggers(QAbstractItemView::NoEditTriggers);
  ui.table_widget_points->setSelectionMode(QAbstractItemView::MultiSelection);
  ui.table_widget_points->setSelectionBehavior(QAbstractItemView::SelectItems);
  ui.table_widget_points->setContextMenuPolicy(Qt::ActionsContextMenu);
  QAction* copy_action = new QAction("复制");
  

  // 设置快捷键 Ctrl+C 进行复制
  copy_action->setShortcut(QKeySequence::Copy);

  // 连接复制操作信号到槽函数
  connect(copy_action, &QAction::triggered, [=]() 
  {
    QItemSelectionModel *selectionModel = ui.table_widget_points->selectionModel();
    if (selectionModel->hasSelection())
    { 
      QModelIndexList selectedIndexes = selectionModel->selectedIndexes();
      QString copiedText;
      int currentRow = -1;
      foreach (const QModelIndex &index, selectedIndexes) 
      {
        if (currentRow != index.row()) 
        {
          if (!copiedText.isEmpty()) 
          {
            copiedText += "\n";
          }
          currentRow = index.row();
        } 
        else 
        {
          copiedText += "\t";
        }
        copiedText += ui.table_widget_points->item(index.row(), index.column())->text();
      }
      // 将文本复制到剪贴板
      QClipboard *clipboard = QApplication::clipboard();
      clipboard->setText(copiedText);
    }
  });

  QAction* copy_row_action = new QAction("复制该行");
  copy_row_action->setShortcut(QKeySequence::Copy);
  connect(copy_row_action, &QAction::triggered, [=]() 
  {
    QItemSelectionModel *selectionModel = ui.table_widget_points->selectionModel();
    if (selectionModel->hasSelection())
    { 
      QModelIndexList selectedIndexes = selectionModel->selectedIndexes();
      QString copiedText;
      int currentRow = -1;

      foreach (const QModelIndex &index, selectedIndexes) 
      {
        // 获取当前行
        if (currentRow != index.row()) 
        {
          if (!copiedText.isEmpty()) 
          {
              copiedText += "\n";
          }

          currentRow = index.row();
          // 读取当前行的8列值
          QString col1 = ui.table_widget_points->item(currentRow, 0)->text();
          QString col2 = ui.table_widget_points->item(currentRow, 1)->text();
          QString col3 = ui.table_widget_points->item(currentRow, 2)->text();
          QString col4 = ui.table_widget_points->item(currentRow, 3)->text();
          QString col5 = ui.table_widget_points->item(currentRow, 4)->text();
          QString col6 = ui.table_widget_points->item(currentRow, 5)->text();
          QString col7 = ui.table_widget_points->item(currentRow, 6)->text();
          QString col8 = ui.table_widget_points->item(currentRow, 7)->text();

          // 按指定格式组合文本
          copiedText += QString("%1:((%2,%3,%4),(%5,%6,%7,%8))")
                          .arg(col1)
                          .arg(col2).arg(col3).arg(col4)
                          .arg(col5).arg(col6).arg(col7).arg(col8);
        } 
      }

      // 将文本复制到剪贴板
      QClipboard *clipboard = QApplication::clipboard();
      clipboard->setText(copiedText);
    }
  });

  QAction* caculate_two_point_dis_action = new QAction("计算两点距离");
  connect(caculate_two_point_dis_action, &QAction::triggered, [=]() 
  {
    QItemSelectionModel *selectionModel = ui.table_widget_points->selectionModel();
    if (selectionModel->hasSelection())
    { 
      QModelIndexList selectedIndexes = selectionModel->selectedIndexes();

    QList<int> selectedRows;
    foreach (const QModelIndex &index, selectedIndexes) 
    {
      if (!selectedRows.contains(index.row())) 
      {
          selectedRows.append(index.row());
      }
    }
    std::sort(selectedRows.begin(), selectedRows.end());
    // 如果选中的行数不是两行，则显示消息框并返回
    if (selectedRows.size() != 2) 
    {
      QMessageBox::warning(this, "警告", "必须选中两行才能计算距离");
      return;
    }

    // 选中两行，继续执行
    int firstRow = selectedRows[0];
    int secondRow = selectedRows[1];
    // qDebug()<<firstRow <<" " << secondRow;

    // 提取并转换第一行的数据
    double col2_row1 = ui.table_widget_points->item(firstRow, 1)->text().toDouble();
    double col3_row1 = ui.table_widget_points->item(firstRow, 2)->text().toDouble();
    double col4_row1 = ui.table_widget_points->item(firstRow, 3)->text().toDouble();
    double col5_row1 = ui.table_widget_points->item(firstRow, 4)->text().toDouble();
    double col6_row1 = ui.table_widget_points->item(firstRow, 5)->text().toDouble();
    double col7_row1 = ui.table_widget_points->item(firstRow, 6)->text().toDouble();
    double col8_row1 = ui.table_widget_points->item(firstRow, 7)->text().toDouble();

    // 提取并转换第二行的数据
    double col2_row2 = ui.table_widget_points->item(secondRow, 1)->text().toDouble();
    double col3_row2 = ui.table_widget_points->item(secondRow, 2)->text().toDouble();
    double col4_row2 = ui.table_widget_points->item(secondRow, 3)->text().toDouble();
    double col5_row2 = ui.table_widget_points->item(secondRow, 4)->text().toDouble();
    double col6_row2 = ui.table_widget_points->item(secondRow, 5)->text().toDouble();
    double col7_row2 = ui.table_widget_points->item(secondRow, 6)->text().toDouble();
    double col8_row2 = ui.table_widget_points->item(secondRow, 7)->text().toDouble();


    geometry_msgs::Point point1,point2;
    {
      point1.x = col2_row1;
      point1.y = col3_row1;
      point1.z = col4_row1;
      point2.x = col2_row2;
      point2.y = col3_row2;
      point2.z = col4_row2;
    }
    double yaw1;
    double yaw2;
    geometry_msgs::Point start_point_row1, end_point_row1;
    {
      start_point_row1.x = col2_row1;
      start_point_row1.y = col3_row1;
      start_point_row1.z = col4_row1;
      // 获取四元数并将其转换为方向向量
      tf2::Quaternion quat(col5_row1, col6_row1, col7_row1, col8_row1);
      tf2::Matrix3x3 mat(quat);
      double roll, pitch, yaw;
      mat.getRPY(roll, pitch, yaw);
      yaw1 = yaw;
      tf2::Vector3 direction_vector(1.0, 0.0, 0.0);  
      direction_vector = tf2::Matrix3x3(quat) * direction_vector;  


      double scale = 2.0;  
      end_point_row1.x = start_point_row1.x + direction_vector.x() * scale;
      end_point_row1.y = start_point_row1.y + direction_vector.y() * scale;
      end_point_row1.z = start_point_row1.z + direction_vector.z() * scale;
    }

    geometry_msgs::Point start_point_row2, end_point_row2;
    {
      start_point_row2.x = col2_row2;
      start_point_row2.y = col3_row2;
      start_point_row2.z = col4_row2;
      // 获取四元数并将其转换为方向向量
      tf2::Quaternion quat(col5_row2, col6_row2, col7_row2, col8_row2);
      tf2::Matrix3x3 mat(quat);
      double roll, pitch, yaw;
      mat.getRPY(roll, pitch, yaw);
      yaw2 = yaw;
      tf2::Vector3 direction_vector(1.0, 0.0, 0.0);  
      direction_vector = tf2::Matrix3x3(quat) * direction_vector;  

      double scale = 2.0;  
      end_point_row2.x = start_point_row2.x + direction_vector.x() * scale;
      end_point_row2.y = start_point_row2.y + direction_vector.y() * scale;
      end_point_row2.z = start_point_row2.z + direction_vector.z() * scale;
    }
    double distance1 = CalculateDistanceToLine(point1, start_point_row2, end_point_row2);
    double distance2 = CalculateDistanceToLine(point2, start_point_row1, end_point_row1);
    double diff_yaw = (yaw1 - yaw2) / M_PI * 180.0;
    QString result_message = QString("距离1: %1\n距离2: %2\n航向差 %3").arg(distance1).arg(distance2).arg(diff_yaw);
    QMessageBox::information(this, "计算结果", result_message);
    }
  });

  ui.table_widget_points->addAction(copy_action);
  ui.table_widget_points->addAction(copy_row_action);
  ui.table_widget_points->addAction(caculate_two_point_dis_action);
  
  ui.push_button_add_initial_pose = new QPushButton("添加Initial");
  ui.push_button_add_end_pose = new QPushButton("添加Goal");

  ui.push_button_add_initial_pose_plan = new QPushButton("添加PlanInitial");
  ui.push_button_add_end_pose_plan = new QPushButton("添加PlanGoal");
  ui.push_button_edit = new QPushButton("编辑");
  ui.push_button_delete = new QPushButton("删除");
  ui.push_button_up = new QPushButton("上移");
  ui.push_button_down = new QPushButton("下移");
  ui.push_button_save = new QPushButton("保存");
  ui.push_button_import_from_file = new QPushButton("导入");

  ui.push_button_publish_one_pose = new QPushButton("发布单点");
  ui.push_button_publish_all_pose = new QPushButton("发布所有");
  ui.push_button_stop_pub_arrow_marker = new QPushButton("停止发布Marker");

  QFont font = ui.push_button_add_initial_pose->font();
  font.setPointSize(12);
  ui.push_button_add_initial_pose->setFont(font);
  ui.push_button_add_initial_pose->setFont(font);
  ui.push_button_add_end_pose->setFont(font);
  ui.push_button_add_initial_pose_plan->setFont(font);
  ui.push_button_add_end_pose_plan->setFont(font);
  ui.push_button_edit->setFont(font);
  ui.push_button_delete->setFont(font);
  ui.push_button_up->setFont(font);
  ui.push_button_down->setFont(font);
  ui.push_button_save->setFont(font);
  ui.push_button_import_from_file->setFont(font);
  ui.push_button_publish_one_pose->setFont(font);
  ui.push_button_publish_all_pose->setFont(font);
  ui.push_button_stop_pub_arrow_marker->setFont(font);

  QGridLayout* grid_layout = new QGridLayout();
  QHBoxLayout* hbox_layout = new QHBoxLayout();
  QVBoxLayout* vbox_layout_buttons = new QVBoxLayout();
  vbox_layout_buttons->addWidget(ui.push_button_add_initial_pose);
  vbox_layout_buttons->addWidget(ui.push_button_add_end_pose);
  vbox_layout_buttons->addWidget(ui.push_button_add_initial_pose_plan);
  vbox_layout_buttons->addWidget(ui.push_button_add_end_pose_plan);
  vbox_layout_buttons->addWidget(ui.push_button_edit);
  vbox_layout_buttons->addWidget(ui.push_button_delete);
  vbox_layout_buttons->addWidget(ui.push_button_up);
  vbox_layout_buttons->addWidget(ui.push_button_down);
  vbox_layout_buttons->addWidget(ui.push_button_save);
  vbox_layout_buttons->addWidget(ui.push_button_import_from_file);
  vbox_layout_buttons->addWidget(ui.push_button_publish_one_pose);
  vbox_layout_buttons->addWidget(ui.push_button_publish_all_pose);
  vbox_layout_buttons->addWidget(ui.push_button_stop_pub_arrow_marker);
  vbox_layout_buttons->addStretch();

  grid_layout->addWidget(ui.table_widget_points, 0, 0, 1, 1);
  grid_layout->addLayout(vbox_layout_buttons, 0, 1, 1, 1);
  grid_layout->setColumnStretch(0,4);
  grid_layout->setColumnStretch(1,1);
  setLayout(grid_layout);

  connect(ui.push_button_add_initial_pose,&QPushButton::clicked,this,&PointsManagerPanel::OnActionAddInitial);
  connect(ui.push_button_add_end_pose,&QPushButton::clicked,this,&PointsManagerPanel::OnActionAddEnd);
  connect(ui.push_button_add_initial_pose_plan,&QPushButton::clicked,this,&PointsManagerPanel::OnActionAddInitialPlan);
  connect(ui.push_button_add_end_pose_plan,&QPushButton::clicked,this,&PointsManagerPanel::OnActionAddEndPlan);
  connect(ui.push_button_edit,&QPushButton::clicked,this,&PointsManagerPanel::OnActionEdit);
  connect(ui.push_button_delete,&QPushButton::clicked,this,&PointsManagerPanel::OnActionDelete);
  connect(ui.push_button_up,&QPushButton::clicked,this,&PointsManagerPanel::OnActionUp);
  connect(ui.push_button_down,&QPushButton::clicked,this,&PointsManagerPanel::OnActionDown);
  connect(ui.push_button_save,&QPushButton::clicked,this,&PointsManagerPanel::OnActionSave);
  connect(ui.push_button_import_from_file,&QPushButton::clicked,this,&PointsManagerPanel::OnActionImportFromFile);
  connect(ui.push_button_publish_one_pose,&QPushButton::clicked,this,&PointsManagerPanel::OnActionPublishOne);
  connect(ui.push_button_publish_all_pose,&QPushButton::clicked,this,&PointsManagerPanel::OnActionPublishAll);
  connect(ui.push_button_stop_pub_arrow_marker,&QPushButton::clicked,this,&PointsManagerPanel::OnActionStopPubArrowMarker);

  ReadFromFile(QString::fromStdString(points_file_path));
  QStringList strListCurrentNodeExeDir = QString::fromStdString(points_file_path).split("/");
  strListCurrentNodeExeDir.removeLast();
  conf_path_ = strListCurrentNodeExeDir.join("/");
}

void PointsManagerPanel::ReadFromFile(const QString filePath)
{
  QFile file(filePath);
  bool okFileInsectionRead = file.open(QIODevice::ReadOnly);

  QStringList fileLineInfo;
  if(okFileInsectionRead)
  {
      while (!file.atEnd())
      {
          QByteArray line = file.readLine();
          QString strLine(line);
          if(!strLine.contains(","))
          {
            QMessageBox::critical(this, tr("出现错误"), tr("导入的路口文件格式错误无法解析！"));
            return;
          }
          if(strLine.contains(","))
          {
            QString strClassId = strLine.split(":")[0];
            QString strlocation = strLine.split(":")[1];
            //((6.2852,3.7627,-0.0334),(-0.0085,-0.0027,-0.0876,0.9961))
            strlocation = strlocation.remove('(');
            strlocation = strlocation.remove(')');
            //qDebug() << strlocation;
            std::cout << strlocation.split(",").size() <<std::endl;
            if(strlocation.split(",").size() != 7)
            {
                QMessageBox::critical(this, tr("出现错误"), tr("导入的路口文件格式错误无法解析！"));
                return;
            }
            fileLineInfo.append(strLine);
        }
    }
    file.close();
  }

  if(!fileLineInfo.size())
  {
    return;
  }

  map_points_data_.clear();
  ui.table_widget_points->setRowCount(0);

  for(int i=0;i<fileLineInfo.size();++i)
  {
    QString strLine = fileLineInfo[i];
    QString strClassId = strLine.split(":")[0];
    QString strlocation = strLine.split(":")[1];
    //((6.2852,3.7627,-0.0334),(-0.0085,-0.0027,-0.0876,0.9961))
    strlocation = strlocation.remove('(');
    strlocation = strlocation.remove(')');

    QString strPosX = strlocation.split(",")[0];
    QString strPosY = strlocation.split(",")[1];
    QString strPosZ = strlocation.split(",")[2];
    QString strOriX = strlocation.split(",")[3];
    QString strOriY = strlocation.split(",")[4];
    QString strOriZ = strlocation.split(",")[5];
    QString strOriW = strlocation.split(",")[6].remove("\n");
    QString strId = QString(strClassId.mid(4));
    bool ok0,ok1,ok2,ok3,ok4,ok5,ok6,ok7;
    PointsData pointData;
    int id = strId.toInt(&ok0);
    pointData.class_name = strClassId.remove(strId).toStdString();
    pointData.pos_x = strPosX.toDouble(&ok1);
    pointData.pos_y = strPosY.toDouble(&ok2);
    pointData.pos_z = strPosZ.toDouble(&ok3);
    pointData.ori_x = strOriX.toDouble(&ok4);
    pointData.ori_y = strOriY.toDouble(&ok5);
    pointData.ori_z = strOriZ.toDouble(&ok6);
    pointData.ori_w = strOriW.toDouble(&ok7);

    if(ok0 && ok1 && ok2 && ok3 && ok4 && ok5 && ok6 && ok7)
    {
        map_points_data_.insert(id, pointData);
    }

    AddTableItem(strClassId.toStdString(),pointData);
  }
}

void PointsManagerPanel::AddTableItem(const std::string class_name,const PointsData point_data, const std::string msg_type)
{
  PointsData pointData;
  if(msg_type == "map_pose")
  {
    if(odom_msg_.header.stamp == ros::Time(0) && point_data == PointsData())
    {
      return;
    }

    pointData.pos_x = odom_msg_.pose.pose.position.x; 
    pointData.pos_y = odom_msg_.pose.pose.position.y; 
    pointData.pos_z = odom_msg_.pose.pose.position.z; 
    pointData.ori_x = odom_msg_.pose.pose.orientation.x;
    pointData.ori_y = odom_msg_.pose.pose.orientation.y;
    pointData.ori_z = odom_msg_.pose.pose.orientation.z;
    pointData.ori_w = odom_msg_.pose.pose.orientation.w;
    // double yaw = map_pose_msg_.yaw/180*M_PI;
    // tf::Quaternion quaternion;
    // quaternion.setRPY(0.0, 0.0, yaw);
    // pointData.pos_x = map_pose_msg_.x;
    // pointData.pos_y = map_pose_msg_.y;
    // pointData.pos_z = map_pose_msg_.z;
    // pointData.ori_x = quaternion.x();
    // pointData.ori_y = quaternion.y();
    // pointData.ori_z = quaternion.z();
    // pointData.ori_w = quaternion.w();
  }
  else
  {
    if(class_name == "init")
    {
      if(initial_pose_msg_ == geometry_msgs::PoseWithCovarianceStamped())
      {
        return;
      }
      pointData.pos_x = initial_pose_msg_.pose.pose.position.x;
      pointData.pos_y = initial_pose_msg_.pose.pose.position.y;
      pointData.pos_z = initial_pose_msg_.pose.pose.position.z;
      pointData.ori_x = initial_pose_msg_.pose.pose.orientation.x;
      pointData.ori_y = initial_pose_msg_.pose.pose.orientation.y;
      pointData.ori_z = initial_pose_msg_.pose.pose.orientation.z;
      pointData.ori_w = initial_pose_msg_.pose.pose.orientation.w;
    }
    else
    {
      if(end_pose_msg_ == geometry_msgs::PoseStamped())
      {
        return;
      }
      pointData.pos_x = end_pose_msg_.pose.position.x;
      pointData.pos_y = end_pose_msg_.pose.position.y;
      pointData.pos_z = end_pose_msg_.pose.position.z;
      pointData.ori_x = end_pose_msg_.pose.orientation.x;
      pointData.ori_y = end_pose_msg_.pose.orientation.y;
      pointData.ori_z = end_pose_msg_.pose.orientation.z;
      pointData.ori_w = end_pose_msg_.pose.orientation.w;
    }
  }


  if(point_data != PointsData())
  {
    pointData = point_data;
  }
  pointData.class_name = class_name;
  QTableWidget* curTableWidget = ui.table_widget_points;

  curTableWidget->setRowCount(curTableWidget->rowCount() + 1);

  QTableWidgetItem* tableWidgetItemId = new QTableWidgetItem();
  tableWidgetItemId->setTextAlignment(Qt::AlignCenter);
  tableWidgetItemId->setText(QString::fromStdString(pointData.class_name) + QString::number(curTableWidget->rowCount() + 1));

  QTableWidgetItem* tableWidgetItemPosX = new QTableWidgetItem();
  tableWidgetItemPosX->setTextAlignment(Qt::AlignCenter);
  tableWidgetItemPosX->setText(QString::number(pointData.pos_x, 'f', 4));

  QTableWidgetItem* tableWidgetItemPosY = new QTableWidgetItem();
  tableWidgetItemPosY->setTextAlignment(Qt::AlignCenter);
  tableWidgetItemPosY->setText(QString::number(pointData.pos_y, 'f', 4));

  QTableWidgetItem* tableWidgetItemPosZ = new QTableWidgetItem();
  tableWidgetItemPosZ->setTextAlignment(Qt::AlignCenter);
  tableWidgetItemPosZ->setText(QString::number(pointData.pos_z, 'f', 4));

  QTableWidgetItem* tableWidgetItemOriX = new QTableWidgetItem();
  tableWidgetItemOriX->setTextAlignment(Qt::AlignCenter);
  tableWidgetItemOriX->setText(QString::number(pointData.ori_x, 'f', 4));

  QTableWidgetItem* tableWidgetItemOriY = new QTableWidgetItem();
  tableWidgetItemOriY->setTextAlignment(Qt::AlignCenter);
  tableWidgetItemOriY->setText(QString::number(pointData.ori_y, 'f', 4));

  QTableWidgetItem* tableWidgetItemOriZ = new QTableWidgetItem();
  tableWidgetItemOriZ->setTextAlignment(Qt::AlignCenter);
  tableWidgetItemOriZ->setText(QString::number(pointData.ori_z, 'f', 4));

  QTableWidgetItem* tableWidgetItemOriW = new QTableWidgetItem();
  tableWidgetItemOriW->setTextAlignment(Qt::AlignCenter);
  tableWidgetItemOriW->setText(QString::number(pointData.ori_w, 'f', 4));

  curTableWidget->setItem(curTableWidget->rowCount() - 1, 0, tableWidgetItemId);
  curTableWidget->setItem(curTableWidget->rowCount() - 1, 1, tableWidgetItemPosX);
  curTableWidget->setItem(curTableWidget->rowCount() - 1, 2, tableWidgetItemPosY);
  curTableWidget->setItem(curTableWidget->rowCount() - 1, 3, tableWidgetItemPosZ);
  curTableWidget->setItem(curTableWidget->rowCount() - 1, 4, tableWidgetItemOriX);
  curTableWidget->setItem(curTableWidget->rowCount() - 1, 5, tableWidgetItemOriY);
  curTableWidget->setItem(curTableWidget->rowCount() - 1, 6, tableWidgetItemOriZ);
  curTableWidget->setItem(curTableWidget->rowCount() - 1, 7, tableWidgetItemOriW);

  map_points_data_.insert(curTableWidget->rowCount() + 1, pointData);
}

void PointsManagerPanel::SwapTableWidgetQueue(int currentrow, int tagrgetrow)
{
  QTableWidget* curTableWidget = ui.table_widget_points;

  QString curItemText = curTableWidget->item(currentrow, 0)->text();
  QString strId = QString(curItemText.mid(4));
  QString curClass = curItemText.remove(strId);
  int curId = strId.toInt();
  QString tarItemText = curTableWidget->item(tagrgetrow, 0)->text();
  strId = QString(tarItemText[tarItemText.size()-1]);
  QString tarClass = tarItemText.remove(strId);
  int targetId = strId.toInt();
  

  auto temp = map_points_data_[targetId];
  map_points_data_[targetId] = map_points_data_[curId];
  map_points_data_[curId] = temp;

  QList<QString> listCurRow;
  QList<QString> listTargetRow;
  listCurRow<<curTableWidget->item(currentrow, 0)->text()
             <<curTableWidget->item(currentrow, 1)->text()
             <<curTableWidget->item(currentrow, 2)->text()
             <<curTableWidget->item(currentrow, 3)->text()
             <<curTableWidget->item(currentrow, 4)->text()
             <<curTableWidget->item(currentrow, 5)->text()
             <<curTableWidget->item(currentrow, 6)->text()
             <<curTableWidget->item(currentrow, 7)->text();

  listTargetRow<<curTableWidget->item(tagrgetrow, 0)->text()
             <<curTableWidget->item(tagrgetrow, 1)->text()
             <<curTableWidget->item(tagrgetrow, 2)->text()
             <<curTableWidget->item(tagrgetrow, 3)->text()
             <<curTableWidget->item(tagrgetrow, 4)->text()
             <<curTableWidget->item(tagrgetrow, 5)->text()
             <<curTableWidget->item(tagrgetrow, 6)->text()
             <<curTableWidget->item(tagrgetrow, 7)->text();

  curTableWidget->item(currentrow, 0)->setText(tarClass + QString::number(curId));
  curTableWidget->item(currentrow, 1)->setText(listTargetRow[1]);
  curTableWidget->item(currentrow, 2)->setText(listTargetRow[2]);
  curTableWidget->item(currentrow, 3)->setText(listTargetRow[3]);
  curTableWidget->item(currentrow, 4)->setText(listTargetRow[4]);
  curTableWidget->item(currentrow, 5)->setText(listTargetRow[5]);
  curTableWidget->item(currentrow, 6)->setText(listTargetRow[6]);
  curTableWidget->item(currentrow, 7)->setText(listTargetRow[7]);

  curTableWidget->item(tagrgetrow, 0)->setText(curClass + QString::number(targetId));
  curTableWidget->item(tagrgetrow, 1)->setText(listCurRow[1]);
  curTableWidget->item(tagrgetrow, 2)->setText(listCurRow[2]);
  curTableWidget->item(tagrgetrow, 3)->setText(listCurRow[3]);
  curTableWidget->item(tagrgetrow, 4)->setText(listCurRow[4]);
  curTableWidget->item(tagrgetrow, 5)->setText(listCurRow[5]);
  curTableWidget->item(tagrgetrow, 6)->setText(listCurRow[6]);
  curTableWidget->item(tagrgetrow, 7)->setText(listCurRow[7]);
}

void PointsManagerPanel::MapPoseCallBack(const car_interfaces::GpsImuInterface& msg)
{
  map_pose_msg_ = msg;
}

void PointsManagerPanel::OdomCallBack(const nav_msgs::Odometry::ConstPtr& msg)
{
  odom_msg_ = *msg;
}

void PointsManagerPanel::InitialPoseCallBack(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  if(!initial_pose_callback_enable_)
  { 
    initial_pose_msg_ = geometry_msgs::PoseWithCovarianceStamped();
    return;
  }
  initial_pose_msg_ = msg;
  AddTableItem("init",PointsData(),"plan");
  initial_pose_callback_enable_ = false;
}

void PointsManagerPanel::EndPoseCallBack(const geometry_msgs::PoseStamped& msg)
{
  if(!end_pose_callback_enable_)
  { 
    end_pose_msg_ = geometry_msgs::PoseStamped();
    return;
  }
  end_pose_msg_ = msg;
  AddTableItem("goal",PointsData(),"plan");
  end_pose_callback_enable_ = false;
}

void PointsManagerPanel::OnActionAddInitial()
{
   AddTableItem("init");
}

void PointsManagerPanel::OnActionAddEnd()
{
  AddTableItem("goal");
}

void PointsManagerPanel::OnActionAddInitialPlan()
{
  initial_pose_callback_enable_ = true;
}

void PointsManagerPanel::OnActionAddEndPlan()
{
  end_pose_callback_enable_ = true;
}

void PointsManagerPanel::OnActionEdit()
{
  QTableWidget* curTableWidget = ui.table_widget_points;
  int curRow = curTableWidget->currentRow();
  int curColumn = curTableWidget->currentColumn();
  QString curItemText = curTableWidget->item(curRow, 0)->text();
  QString strId = QString(curItemText.mid(4));
  int id = strId.toInt();
  PointsData pointData = map_points_data_[id];
 
  bool ok;
  bool okToDouble;
  QString newValue;
  if(curRow == -1)
  {
    return;
  }
  if(curColumn == 0)
  {
    return;
  }

  if(curColumn == 1)
  {
    newValue = QInputDialog::getText(this, "坐标输入", "请输入新的PoseX:", QLineEdit::Normal, "", &ok);
    if(!ok)
    {
      return;
    }
    newValue.toDouble(&okToDouble);
    while(!okToDouble && ok)
    {
      newValue = QInputDialog::getText(this, "坐标输入", "请重新输入,类型为(double):", QLineEdit::Normal, "", &ok);
      newValue.toDouble(&okToDouble);
    }
    if(!newValue.isEmpty() && okToDouble)
    {
      pointData.pos_x = newValue.toDouble(&okToDouble);
    }
  }

  if(curColumn == 2)
  {
    newValue = QInputDialog::getText(this, "坐标输入", "请输入新的PoseY", QLineEdit::Normal, "", &ok);
    if(!ok)
    {
      return;
    }
    newValue.toDouble(&okToDouble);
    while(!okToDouble && ok)
    {
      newValue = QInputDialog::getText(this, "坐标输入", "请重新输入,类型为(double):", QLineEdit::Normal, "", &ok);
      newValue.toDouble(&okToDouble);
    }
    if(!newValue.isEmpty() && okToDouble)
    {
      pointData.pos_y = newValue.toDouble(&okToDouble);
    }
  }

  if(curColumn == 3)
  {
    newValue = QInputDialog::getText(this, "坐标输入", "请输入新的PoseZ", QLineEdit::Normal, "", &ok);
    if(!ok)
    {
      return;
    }
    newValue.toDouble(&okToDouble);
    while(!okToDouble && ok)
    {
      newValue = QInputDialog::getText(this, "坐标输入", "请重新输入,类型为(double):", QLineEdit::Normal, "", &ok);
      newValue.toDouble(&okToDouble);
    }
    if(!newValue.isEmpty() && okToDouble)
    {
      pointData.pos_z = newValue.toDouble(&okToDouble);
    }
  }

  if(curColumn == 4)
  {
    newValue = QInputDialog::getText(this, "坐标输入", "请输入新的OriX", QLineEdit::Normal, "", &ok);
    if(!ok)
    {
      return;
    }
    newValue.toDouble(&okToDouble);
    while(!okToDouble && ok)
    {
      newValue = QInputDialog::getText(this, "坐标输入", "请重新输入,类型为(double):", QLineEdit::Normal, "", &ok);
      newValue.toDouble(&okToDouble);
    }
    if(!newValue.isEmpty() && okToDouble)
    {
      pointData.ori_x = newValue.toDouble(&okToDouble);
    }
  }
  
  if(curColumn == 5)
  {
    newValue = QInputDialog::getText(this, "坐标输入", "请输入新的OriY", QLineEdit::Normal, "", &ok);
    if(!ok)
    {
      return;
    }
    newValue.toDouble(&okToDouble);
    while(!okToDouble && ok)
    {
      newValue = QInputDialog::getText(this, "坐标输入", "请重新输入,类型为(double):", QLineEdit::Normal, "", &ok);
      newValue.toDouble(&okToDouble);
    }
    if(!newValue.isEmpty() && okToDouble)
    {
      pointData.ori_y = newValue.toDouble(&okToDouble);
    }
  }

  if(curColumn == 6)
  {
    newValue = QInputDialog::getText(this, "坐标输入", "请输入新的OriZ", QLineEdit::Normal, "", &ok);
    if(!ok)
    {
      return;
    }
    newValue.toDouble(&okToDouble);
    while(!okToDouble && ok)
    {
      newValue = QInputDialog::getText(this, "坐标输入", "请重新输入,类型为(double):", QLineEdit::Normal, "", &ok);
      newValue.toDouble(&okToDouble);
    }
    if(!newValue.isEmpty() && okToDouble)
    {
      pointData.ori_z = newValue.toDouble(&okToDouble);
    }
  }

  if(curColumn == 7)
  {
    newValue = QInputDialog::getText(this, "坐标输入", "请输入新的OriW", QLineEdit::Normal, "", &ok);
    if(!ok)
    {
      return;
    }
    newValue.toDouble(&okToDouble);
    while(!okToDouble && ok)
    {
      newValue = QInputDialog::getText(this, "坐标输入", "请重新输入,类型为(double):", QLineEdit::Normal, "", &ok);
      newValue.toDouble(&okToDouble);
    }
    if(!newValue.isEmpty() && okToDouble)
    {
      pointData.ori_w = newValue.toDouble(&okToDouble);
    }
  }

  if(newValue.isEmpty())
  {
    return;
  }
  map_points_data_.insert(id,pointData);
  curTableWidget->currentItem()->setText(newValue);
}

void PointsManagerPanel::OnActionDelete()
{
  QTableWidget* curTableWidget = ui.table_widget_points;
  int curRow = curTableWidget->currentRow();
  if(curRow == -1)
  {
    return;
  }
  QString curItemText = curTableWidget->item(curRow, 0)->text();
  QString strId = QString(curItemText.mid(4));
  int id = strId.toInt();
  map_points_data_.remove(id);
  curTableWidget->removeRow(curRow);
}

void PointsManagerPanel::OnActionUp()
{
  QTableWidget* curTableWidget = ui.table_widget_points;
  int curRow = curTableWidget->currentRow();


  //如果选中了数据，（未选择数据之前，返回值是-1）
  if (curRow != -1)
  {
    //如果选定行不在第一行
    if (curRow != 0)
    {
      SwapTableWidgetQueue(curRow, curRow - 1);
      //移动过后继续选定该行
      curTableWidget->setCurrentCell(curRow - 1, QItemSelectionModel::Select);
    }
  }
  else
  {
    //如果有数据，但是currentRow=--1 说明没有选择数据,把焦点定位到第一行
    if (curTableWidget->rowCount() != 0)
    {
      QMessageBox::critical(this, tr("出现错误"), tr("无法移动：未选中数据"));
    }
    else
    {
      QMessageBox::critical(this, tr("出现错误"), tr("无法移动：表格没有数据"));
    }
  }
}

void PointsManagerPanel::OnActionDown()
{
  QTableWidget* curTableWidget = ui.table_widget_points;
  int curRow = curTableWidget->currentRow();

  if (curRow != -1)//如果选中了一行
  {
    if (curRow != (curTableWidget->rowCount() - 1))//如果不是最后一行
    {
      SwapTableWidgetQueue(curRow, curRow + 1);
      //移动过后继续选定该行
      curTableWidget->setCurrentCell(curRow + 1, QItemSelectionModel::Select);
    }
  }
  else
  {
    if (curTableWidget->rowCount() != 0)//如果有数据，但是currentRow=--1 说明没有选择数据,把焦点定位到第一行
    {
      QMessageBox::critical(this, tr("出现错误"), tr("无法移动：未选中数据"));
    }
    else
    {
      QMessageBox::critical(this, tr("出现错误"), tr("无法移动：表格没有数据"));
    }
  }
}

void PointsManagerPanel::OnActionSave()
{
  QString fileName = QFileDialog::getSaveFileName(this,tr("保存数据文件"),conf_path_,"*.txt");
  if(fileName.isEmpty())
  {
    return;
  }

  QFile file(fileName);
  bool okFileInsectionRead = file.open(QFile::WriteOnly|QFile::Truncate);
  if(!okFileInsectionRead)
  {
    QMessageBox::critical(this, tr("出现错误"), tr("保存文件创建失败！"));
    return;
  }

  QString strLine;

  QTextStream streamInsection(&file);
  QMap<int ,PointsData>::Iterator iter = map_points_data_.begin();
  while(iter != map_points_data_.end())
  {
    //init8:((27.6980, 12.2035, 0.0), (0.0, 0.0, -0.9996, 0.0280))
    strLine = QString::fromStdString(iter.value().class_name) + QString::number(iter.key()) + ":"
              + "((" + QString::number(iter.value().pos_x, 'f', 4) + ","
              + QString::number(iter.value().pos_y, 'f', 4) + ","
              + QString::number(iter.value().pos_z, 'f', 4) + "),("
              + QString::number(iter.value().ori_x, 'f', 4) + ","
              + QString::number(iter.value().ori_y, 'f', 4) + ","
              + QString::number(iter.value().ori_z, 'f', 4) + ","
              + QString::number(iter.value().ori_w, 'f', 4) + "))\n";
    streamInsection<<strLine;
    ++iter;
  }
  file.close();
  QMessageBox::information(this, tr("保存"), tr("保存文件写入成功！"));
}

void PointsManagerPanel::OnActionImportFromFile()
{
  QString fileName = QFileDialog::getOpenFileName(this,tr("保存数据文件"),conf_path_,"*.txt");
  if(fileName.isEmpty())
  {
    return;
  }

  QFile file(fileName);
  bool okFileInsectionRead = file.open(QFile::ReadOnly);
  if(!okFileInsectionRead)
  {
    QMessageBox::critical(this, tr("出现错误"), tr("保存文件创建失败！"));
    return;
  }
  file.close();
  ReadFromFile(fileName);
}

void PointsManagerPanel::OnActionPublishOne()
{
  QTableWidget* curTableWidget = ui.table_widget_points;
  int curRow = curTableWidget->currentRow();
  if(curRow == -1)
  {
    return;
  }
  QString curItemText = curTableWidget->item(curRow, 0)->text();
  QString strId = QString(curItemText.mid(4));
  int id = strId.toInt();
  QString class_name = curItemText.remove(strId);
  PointsData pointData = map_points_data_[id];
  vector_points_data_.emplace_back(pointData);
  timer_pub_arrow_marker_->start();
  
  if(class_name == "init")
  {
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";

    msg.pose.pose.position.x = pointData.pos_x;
    msg.pose.pose.position.y = pointData.pos_y;
    msg.pose.pose.position.z = pointData.pos_z;
    msg.pose.pose.orientation.x = pointData.ori_x;
    msg.pose.pose.orientation.y = pointData.ori_y;
    msg.pose.pose.orientation.z = pointData.ori_z;
    msg.pose.pose.orientation.w = pointData.ori_w;

    for (int i = 0; i < 36; ++i) 
    {
        msg.pose.covariance[i] = 0.0;
    }
    initial_pose_pub_.publish(msg);
  }
  else
  {
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";

    msg.pose.position.x = pointData.pos_x;
    msg.pose.position.y = pointData.pos_y;
    msg.pose.position.z = pointData.pos_z;
    msg.pose.orientation.x = pointData.ori_x;
    msg.pose.orientation.y = pointData.ori_y;
    msg.pose.orientation.z = pointData.ori_z;
    msg.pose.orientation.w = pointData.ori_w;
    end_pose_pub_.publish(msg);
  }
}

void PointsManagerPanel::OnActionPublishAll()
{
  for (QMap<int, PointsData>::const_iterator it = map_points_data_.cbegin(); it != map_points_data_.cend(); ++it) 
  {
    int key = it.key();
    PointsData pointData = it.value();
    vector_points_data_.emplace_back(pointData);
    if(pointData.class_name == "init")
    {
      geometry_msgs::PoseWithCovarianceStamped msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "map";

      msg.pose.pose.position.x = pointData.pos_x;
      msg.pose.pose.position.y = pointData.pos_y;
      msg.pose.pose.position.z = pointData.pos_z;
      msg.pose.pose.orientation.x = pointData.ori_x;
      msg.pose.pose.orientation.y = pointData.ori_y;
      msg.pose.pose.orientation.z = pointData.ori_z;
      msg.pose.pose.orientation.w = pointData.ori_w;

      for (int i = 0; i < 36; ++i) 
      {
          msg.pose.covariance[i] = 0.0;
      }
      initial_pose_pub_.publish(msg);
    }
    else
    {
      geometry_msgs::PoseStamped msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "map";

      msg.pose.position.x = pointData.pos_x;
      msg.pose.position.y = pointData.pos_y;
      msg.pose.position.z = pointData.pos_z;
      msg.pose.orientation.x = pointData.ori_x;
      msg.pose.orientation.y = pointData.ori_y;
      msg.pose.orientation.z = pointData.ori_z;
      msg.pose.orientation.w = pointData.ori_w;
      end_pose_pub_.publish(msg);
    }
    sleep(1);
  }
  timer_pub_arrow_marker_->start();
}

void PointsManagerPanel::OnActionStopPubArrowMarker()
{
  timer_pub_arrow_marker_->stop();
  vector_points_data_.clear();
}

void PointsManagerPanel::SlotFunctionTimerPubArrowMarker()
{
  for(int i = 0; i < vector_points_data_.size(); ++i)
  {
    PointsData pointData = vector_points_data_[i];
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map"; // 替换为您的坐标系
    marker.header.stamp = ros::Time::now();
    marker.id = i;
    marker.type = visualization_msgs::Marker::ARROW; // 使用箭头表示法向量
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(1);
    if(pointData.class_name == "init")
    {
      marker.ns = "initialpose";
      marker.color.r = 0.0; 
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0; 
    }
    else
    {
      marker.ns = "goal";
      marker.color.r = 1.0;    
      marker.color.g = 0.0;    
      marker.color.b = 1.0;     
      marker.color.a = 1.0;     
    }
    geometry_msgs::Point start_point;
    start_point.x = pointData.pos_x;
    start_point.y = pointData.pos_y;
    start_point.z = pointData.pos_z;
    // 获取四元数并将其转换为方向向量
    tf2::Quaternion quat(pointData.ori_x, pointData.ori_y, pointData.ori_z, pointData.ori_w);
    tf2::Vector3 direction_vector(1.0, 0.0, 0.0);  
    direction_vector = tf2::Matrix3x3(quat) * direction_vector;  

    geometry_msgs::Point end_point;
    double scale = 2.0;  // 可根据需求调整箭头长度
    end_point.x = start_point.x + direction_vector.x() * scale;
    end_point.y = start_point.y + direction_vector.y() * scale;
    end_point.z = start_point.z + direction_vector.z() * scale;
    marker.points.push_back(start_point);
    marker.points.push_back(end_point);
    marker.scale.x = 0.2; // 箭头的宽度
    marker.scale.y = 0.3; // 箭头的头部宽度
    marker.scale.z = 0.3; // 箭头的头部高度
    arrow_marker_pub_.publish(marker);
  }
}

double PointsManagerPanel::CalculateDistanceToLine(const geometry_msgs::Point& point, const geometry_msgs::Point& start_point, const geometry_msgs::Point& end_point) 
{
    // 计算 P2 - P1
    tf2::Vector3 line_vector(end_point.x - start_point.x, end_point.y - start_point.y, end_point.z - start_point.z);
    
    // 计算 P - P1
    tf2::Vector3 point_vector(point.x - start_point.x, point.y - start_point.y, point.z - start_point.z);
    
    // 计算向量的叉积 (P - P1) × (P2 - P1)
    tf2::Vector3 cross_product = point_vector.cross(line_vector);
    
    // 计算叉积的模 |(P - P1) × (P2 - P1)|
    double cross_product_norm = cross_product.length();
    
    // 计算直线方向向量的模 |P2 - P1|
    double line_length = line_vector.length();
    
    // 点到直线的距离
    double distance = cross_product_norm / line_length;
    qDebug() << " Line vector" << line_vector.x() << " " << line_vector.y() << " " << line_vector.z();
    qDebug() << " Line length" << line_length;
    qDebug() << " Line cross_product" << cross_product.x() << " " << cross_product.y() << " " << cross_product.z();
    qDebug() << " Line cross_product_norm" << cross_product_norm;
    return distance;
}

} // end namespace points_manager_panel

PLUGINLIB_EXPORT_CLASS(points_manager_panel::PointsManagerPanel, rviz::Panel)
