#ifndef BUTTON_RVIZ_H
#define BUTTON_RVIZ_H

#include <rviz/panel.h>
#include <QPushButton>
#include <QVector>

namespace button_rviz
{

class AutoButton : public rviz::Panel
{
  Q_OBJECT
public:
  // 构造函数
  AutoButton(QWidget* parent = 0);

  // 初始化函数
  virtual void onInitialize();

private Q_SLOTS:
  // 处理启动按钮点击事件
  void startCommand(int index);

  // 处理停止按钮点击事件
  void stopCommand(int index);

private:
  // 存储启动和停止按钮的数组
  QVector<QPushButton*> start_buttons_;  // 启动按钮数组
  QVector<QPushButton*> stop_buttons_;   // 停止按钮数组
};

} // end namespace button_rviz

#endif // BUTTON_RVIZ_H
