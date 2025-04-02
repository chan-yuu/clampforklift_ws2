#ifndef BUTTON_RVIZ2_H
#define BUTTON_RVIZ2_H

#include <rviz/panel.h>
#include <QPushButton>
#include <QVector>

namespace button_rviz
{

class CustomButton : public rviz::Panel
{
  Q_OBJECT
public:
  // 构造函数
  CustomButton(QWidget* parent = 0);

  // 初始化函数
  virtual void onInitialize();

private Q_SLOTS:
  // 启动命令的槽函数
  void startCommand(int index);

  // 停止命令的槽函数
  void stopCommand(int index);

private:
  // 存储按钮的数组
  QVector<QPushButton*> start_buttons_;  // 启动按钮
  QVector<QPushButton*> stop_buttons_;   // 停止按钮
};

} // end namespace button_rviz

#endif // BUTTON_RVIZ2_H
