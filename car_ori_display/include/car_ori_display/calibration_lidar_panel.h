#ifndef CALIBRATION_LIDAR_PANEL_H
#define CALIBRATION_LIDAR_PANEL_H

#include <rviz/panel.h>
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QLabel>
#include <ros/ros.h>

namespace calibration_lidar_panel
{

class CalibrationLidarPanel : public rviz::Panel
{
Q_OBJECT
public:
  CalibrationLidarPanel(QWidget* parent = 0);

protected Q_SLOTS:
  void saveCalibrationParams();

protected:
  virtual void onInitialize();

  QLineEdit* x_offset_edit_;
  QLineEdit* y_offset_edit_;
  QLineEdit* z_offset_edit_;
  QLineEdit* roll_offset_edit_;
  QLineEdit* pitch_offset_edit_;
  QLineEdit* yaw_offset_edit_;
  QLineEdit* input_topic_edit_;
  QLineEdit* output_topic_edit_;

  QPushButton* save_button_;

  ros::NodeHandle nh_;
};

} // end namespace calibration_lidar_panel

#endif // CALIBRATION_LIDAR_PANEL_H
