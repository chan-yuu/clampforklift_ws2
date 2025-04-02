#ifndef ANGLE_DISPLAY_H
#define ANGLE_DISPLAY_H

#include <rviz/panel.h>
#include <std_msgs/Float64.h>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <ros/ros.h>

namespace angle_display
{

class AngleDisplay : public rviz::Panel
{
Q_OBJECT
public:
  AngleDisplay(QWidget* parent = 0);

protected Q_SLOTS:
  void updateTopic();

protected:
  virtual void onInitialize();

  void angleCallback(const std_msgs::Float64::ConstPtr& msg);
  void hCallback(const std_msgs::Float64::ConstPtr& msg);

  QLineEdit* topic_input_;
  QPushButton* update_button_;
  QLabel* angle_label_;
  QLabel* angle_h_label_;
  ros::NodeHandle nh_;
  ros::Subscriber angle_sub_;
  ros::Subscriber angle_h_sub_;
};

} // end namespace angle_display

#endif // ANGLE_DISPLAY_H
