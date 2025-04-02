#include <pluginlib/class_list_macros.h>
#include <QLayout>
#include <QMessageBox>

#include <unistd.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geodesy/utm.h>
#include <geographic_msgs/GeoPoint.h>
#include "car_ori_display/lola_to_utm_panel.h"
namespace lola_to_utm_panel
{

  LoLaToUTMPanel::LoLaToUTMPanel(QWidget* parent)
    : rviz::Panel(parent)
  {
    Init();
  }

  void LoLaToUTMPanel::onInitialize()
  {
    // 如果需要初始化其他内容，可以在这里添加
  }

  void LoLaToUTMPanel::Init()
  {
    double x,y,z;
    double roll,yaw,pitch;
    nh_.getParam("lola_to_utm_x", x);
    nh_.getParam("lola_to_utm_y", y);
    nh_.getParam("lola_to_utm_z", z);
    nh_.getParam("lola_to_utm_roll", roll);
    nh_.getParam("lola_to_utm_pitch", pitch);
    nh_.getParam("lola_to_utm_yaw", yaw);

    // 将角度从度转换为弧度
    double roll_rad = roll * M_PI / 180.0;
    double pitch_rad = pitch * M_PI / 180.0;
    double yaw_rad = yaw * M_PI / 180.0;
    // 创建旋转四元数
    tf2::Quaternion q_tf2;
    q_tf2.setRPY(roll_rad, pitch_rad, yaw_rad);
    q_tf2.normalize();

    transform.setOrigin(tf2::Vector3(x, y, z));
    transform.setRotation(q_tf2);


    // Initialize UI elements
    ui.label_lo = new QLabel("Lo:");
    ui.label_la = new QLabel("La:");
    ui.label_heading = new QLabel("Yaw:");

    ui.lineedit_lo = new QLineEdit;
    ui.lineedit_la = new QLineEdit;
    ui.lineedit_heading = new QLineEdit;

    ui.label_utm_x = new QLabel("U X:");
    ui.label_utm_y = new QLabel("U Y:");
    ui.label_utm_heading = new QLabel("U Yaw:");

    ui.lineedit_utm_x = new QLineEdit;
    ui.lineedit_utm_y = new QLineEdit;
    ui.lineedit_utm_heading = new QLineEdit;

    ui.label_map_x = new QLabel("M X:");
    ui.label_map_y = new QLabel("M Y:");
    ui.label_map_heading = new QLabel("M Yaw:");
    ui.lineedit_map_x = new QLineEdit;
    ui.lineedit_map_y = new QLineEdit;
    ui.lineedit_map_heading = new QLineEdit;
    
    ui.convert_button = new QPushButton("Convert");

    QGridLayout* grid_layout = new QGridLayout();

    grid_layout->addWidget(ui.label_lo, 0, 0, 1, 1);
    grid_layout->addWidget(ui.lineedit_lo, 0, 1, 1, 1);
    grid_layout->addWidget(ui.label_la, 1, 0, 1, 1);
    grid_layout->addWidget(ui.lineedit_la, 1, 1, 1, 1);
    grid_layout->addWidget(ui.label_heading, 2, 0, 1, 1);
    grid_layout->addWidget(ui.lineedit_heading, 2, 1, 1, 1);

    grid_layout->addWidget(ui.label_utm_x, 0, 2, 1, 1);
    grid_layout->addWidget(ui.lineedit_utm_x, 0, 3, 1, 1);
    grid_layout->addWidget(ui.label_utm_y, 1, 2, 1, 1);
    grid_layout->addWidget(ui.lineedit_utm_y, 1, 3, 1, 1);
    grid_layout->addWidget(ui.label_utm_heading, 2, 2, 1, 1);
    grid_layout->addWidget(ui.lineedit_utm_heading, 2, 3, 1, 1);
    grid_layout->addWidget(ui.label_map_x, 0, 4, 1, 1);
    grid_layout->addWidget(ui.lineedit_map_x, 0, 5, 1, 1);
    grid_layout->addWidget(ui.label_map_y, 1, 4, 1, 1);
    grid_layout->addWidget(ui.lineedit_map_y, 1, 5, 1, 1);
    grid_layout->addWidget(ui.label_map_heading, 2, 4, 1, 1);
    grid_layout->addWidget(ui.lineedit_map_heading, 2, 5, 1, 1);
    grid_layout->addWidget(ui.convert_button, 3, 0, 1, 2);
    grid_layout->setColumnStretch(0,2);
    grid_layout->setColumnStretch(1,2);
    grid_layout->setColumnStretch(2,2);
    grid_layout->setColumnStretch(3,3);
    grid_layout->setColumnStretch(4,2);
    grid_layout->setColumnStretch(5,3);
    setLayout(grid_layout);

    connect(ui.convert_button, SIGNAL(clicked()), this, SLOT(onConvertButtonClicked()));
  }

  void LoLaToUTMPanel::onConvertButtonClicked()
  {
    bool ok_lo, ok_la, ok_heading;
    double lo = ui.lineedit_lo->text().toDouble(&ok_lo);
    double la = ui.lineedit_la->text().toDouble(&ok_la);
    double heading = ui.lineedit_heading->text().toDouble(&ok_heading);
    heading = heading * M_PI / 180.0;
    if (!ok_lo || !ok_la || !ok_heading)
    {
      QMessageBox::warning(this, "Input Error", "Please enter valid numerical values for Longitude, Latitude, and Heading.");
      return;
    }
   

    try
    {
      int zone;
      bool northp;
      // 创建一个GeoPoint对象并设置经纬度
      geographic_msgs::GeoPoint geo_point;
      geo_point.latitude = la;  // 设置纬度
      geo_point.longitude = lo;  // 设置经度
      geo_point.altitude = 0.0;  // 设置高度

      // 将GeoPoint转换为UTM坐标
      geodesy::UTMPoint utm_point;
      geodesy::fromMsg(geo_point, utm_point);

      ui.lineedit_utm_x->setText(QString::number(utm_point.easting, 'f', 6));
      ui.lineedit_utm_y->setText(QString::number(utm_point.northing, 'f', 6));
      ui.lineedit_utm_heading->setText(QString::number(heading, 'f', 3));


      tf2::Transform base_to_world;
      base_to_world.setOrigin(tf2::Vector3(utm_point.easting, utm_point.northing, 0));
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, heading);
      base_to_world.setRotation(q);

      // 计算 base_footprint 在 map 坐标系中的位姿
      tf2::Transform world_to_map = transform.inverse();
      tf2::Transform base_to_map = world_to_map * base_to_world;
      tf2::Quaternion q_base_to_map = base_to_map.getRotation();
      double roll, pitch, yaw;
      tf2::Matrix3x3(q_base_to_map).getRPY(roll, pitch, yaw);

     
      // Update UI with results
      ui.lineedit_map_x->setText(QString::number(base_to_map.getOrigin().x(), 'f', 3));
      ui.lineedit_map_y->setText(QString::number(base_to_map.getOrigin().y(), 'f', 3));
      ui.lineedit_map_heading->setText(QString::number(yaw * 180.0 / M_PI, 'f', 3));
    }
    catch (const std::exception& e)
    {
      QMessageBox::critical(this, "Conversion Error", e.what());
    }
  }
}
PLUGINLIB_EXPORT_CLASS(lola_to_utm_panel::LoLaToUTMPanel, rviz::Panel)