#ifndef CUSTOM_RVIZ_PLUGIN_SERVICE_CONFIG_PANEL_H
#define CUSTOM_RVIZ_PLUGIN_SERVICE_CONFIG_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <QPushButton>
#include <QComboBox>
#include <QLineEdit>
#include <std_srvs/Trigger.h>

namespace car_ori_display
{

class ServiceConfigPanel : public rviz::Panel
{
Q_OBJECT
public:
  ServiceConfigPanel(QWidget* parent = 0);

public Q_SLOTS:
  void saveServiceConfig();
  void triggerStateMachine();

protected:
  QComboBox* service_selector_;
  
  // 输入位姿的相关参数
  QLineEdit* param_input1_; // 位置X (p_x)
  QLineEdit* param_input2_; // 位置Y (p_y)
  QLineEdit* param_input3_; // 位置Z (p_z)
  QLineEdit* param_input4_; // 方向X (o_x)
  QLineEdit* param_input5_; // 方向Y (o_y)
  QLineEdit* param_input6_; // 方向Z (o_z)
  
  QPushButton* save_button_;
  QPushButton* trigger_button_;
  QString file_path_;
  ros::NodeHandle nh_;
};

} // namespace car_ori_display

#endif // CUSTOM_RVIZ_PLUGIN_SERVICE_CONFIG_PANEL_H
