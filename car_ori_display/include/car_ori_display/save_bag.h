#ifndef SAVE_BAG_H
#define SAVE_BAG_H

#include <rviz/panel.h>
#include <QLineEdit>
#include <QPushButton>
#include <ros/ros.h>
#include <QString>

namespace save_bag
{

class SaveBag : public rviz::Panel
{
Q_OBJECT
public:
  SaveBag(QWidget* parent = 0);

protected Q_SLOTS:
  void collectAll();
  void collectTopic();
  void saveBag();
  void runPlotJuggler();

protected:
  virtual void onInitialize();

  QLineEdit* topic_input_;
  QLineEdit* file_name_input_;
  QLineEdit* save_directory_input_;
  QPushButton* collect_all_button_;
  QPushButton* collect_topic_button_;
  QPushButton* save_button_;
  QPushButton* plotjuggler_button_;
  ros::NodeHandle nh_;

  QString bag_process_id_;
};

} // end namespace save_bag

#endif // SAVE_BAG_H
