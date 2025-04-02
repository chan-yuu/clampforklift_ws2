#ifndef POINTS_MANAGER_PANEL_H
#define POINTS_MANAGER_PANEL_H

#include <rviz/panel.h>
#include <QTableWidget>
#include <QPushButton>
#include <QLabel>
#include <QTimer>

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <visualization_msgs/Marker.h>

#include "car_interfaces/GpsImuInterface.h"

namespace points_manager_panel
{

struct PointsData
{
  std::string class_name;
  //int nmuber;
  double pos_x;
  double pos_y;
  double pos_z;
  double ori_x;
  double ori_y;
  double ori_z;
  double ori_w;

  // 构造函数，初始化所有成员
  PointsData() : 
      class_name(""), pos_x(0.0), pos_y(0.0), pos_z(0.0), 
      ori_x(0.0), ori_y(0.0), ori_z(0.0), ori_w(0.0){}
  // 重载 == 运算符
  bool operator==(const PointsData& other) const
  {
      return 
          class_name == other.class_name &&
          pos_x == other.pos_x &&
          pos_y == other.pos_y &&
          pos_z == other.pos_z &&
          ori_x == other.ori_x &&
          ori_y == other.ori_y &&
          ori_z == other.ori_z &&
          ori_w == other.ori_w;
  }

  // 重载 != 运算符，通常基于 == 实现
  bool operator!=(const PointsData& other) const
  {
      return !(*this == other);
  }
};

class PointsManagerPanel : public rviz::Panel
{
Q_OBJECT
public:
  PointsManagerPanel(QWidget* parent = 0);

protected Q_SLOTS:
  void MapPoseCallBack(const car_interfaces::GpsImuInterface& msg);
  void InitialPoseCallBack(const geometry_msgs::PoseWithCovarianceStamped& msg);
  void EndPoseCallBack(const geometry_msgs::PoseStamped& msg);
  void OdomCallBack(const nav_msgs::Odometry::ConstPtr& msg);
protected slots:
  void OnActionAddInitial();
  void OnActionAddEnd();
  void OnActionAddInitialPlan();
  void OnActionAddEndPlan();
  void OnActionEdit();
  void OnActionDelete();
  void OnActionUp();
  void OnActionDown();
  void OnActionSave();
  void OnActionImportFromFile();
  void OnActionPublishOne();
  void OnActionPublishAll();
  void OnActionStopPubArrowMarker();
  void SlotFunctionTimerPubArrowMarker();
protected:
  virtual void onInitialize();
  void Init();
  void ReadFromFile(const QString filePath);
  void AddTableItem(const std::string class_name, const PointsData point_data = PointsData(), const std::string msg_type = "map_pose");
  void SwapTableWidgetQueue(int currentrow, int tagrgetrow);
  double CalculateDistanceToLine(const geometry_msgs::Point& point, const geometry_msgs::Point& start_point, const geometry_msgs::Point& end_point); 
  QMap<int, PointsData> map_points_data_;
  QString conf_path_;
  bool initial_pose_callback_enable_;
  bool end_pose_callback_enable_;

  struct PointsManagerPanelMembers
  {
    QTableWidget* table_widget_points;
    QPushButton* push_button_edit;
    QPushButton* push_button_delete;
    QPushButton* push_button_up;
    QPushButton* push_button_down;
    QPushButton* push_button_save;

    QPushButton* push_button_add_initial_pose;
    QPushButton* push_button_add_end_pose;
    QPushButton* push_button_import_from_file;
    QPushButton* push_button_publish_one_pose;
    QPushButton* push_button_publish_all_pose;

    QPushButton* push_button_add_initial_pose_plan;
    QPushButton* push_button_add_end_pose_plan;
    QPushButton* push_button_stop_pub_arrow_marker;
  };
  PointsManagerPanelMembers ui;

  ros::NodeHandle nh_;
  ros::Subscriber map_pose_sub_;
  ros::Subscriber initial_pose_sub_;
  ros::Subscriber end_pose_sub_;
  ros::Subscriber odom_sub_;

  ros::Publisher initial_pose_pub_;
  ros::Publisher end_pose_pub_;

  ros::Publisher arrow_marker_pub_;
  QTimer* timer_pub_arrow_marker_;
  std::vector<PointsData> vector_points_data_;
  nav_msgs::Odometry odom_msg_;
  car_interfaces::GpsImuInterface map_pose_msg_;
  geometry_msgs::PoseWithCovarianceStamped initial_pose_msg_;
  geometry_msgs::PoseStamped end_pose_msg_;
};

} // end namespace points_manager_panel

#endif // POINTS_MANAGER_PANEL_H
