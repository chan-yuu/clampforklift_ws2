#ifndef LOLA_TO_UTM_PANEL_H
#define LOLA_TO_UTM_PANEL_H

#include <rviz/panel.h>
#include <QPushButton>
#include <QLabel>
#include <QLineEdit>

#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>


namespace lola_to_utm_panel
{
  class LoLaToUTMPanel : public rviz::Panel
  {
    Q_OBJECT
  public:
    LoLaToUTMPanel(QWidget* parent = 0);

  protected Q_SLOTS:
    void onConvertButtonClicked();

  protected:
    virtual void onInitialize();
    void Init();

    struct LoLaToUTMPanelMembers
    {
      QLabel* label_lo;
      QLabel* label_la;
      QLabel* label_heading;
      QLineEdit* lineedit_lo;
      QLineEdit* lineedit_la;
      QLineEdit* lineedit_heading;
      QLabel* label_utm_x;
      QLabel* label_utm_y;
      QLabel* label_utm_heading;
      QLineEdit* lineedit_utm_x;
      QLineEdit* lineedit_utm_y;
      QLineEdit* lineedit_utm_heading;

      QLabel* label_map_x;
      QLabel* label_map_y;
      QLabel* label_map_heading;
      QLineEdit* lineedit_map_x;
      QLineEdit* lineedit_map_y;
      QLineEdit* lineedit_map_heading;
      QPushButton* convert_button;
    };
    LoLaToUTMPanelMembers ui;
    ros::NodeHandle nh_;

    tf2::Transform transform;
  };

} // end namespace lola_to_utm_panel

#endif // LOLA_TO_UTM_PANEL_H
