
#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>

#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <mutex>
#include <tf/tf.h>
#include<math.h>

// #include <car_interfaces/RoadToControl.h>
#include "control/stanley_control.h"
#include <geometry_msgs/TwistStamped.h>
#include <car_interfaces/ControlTask.h>
#include <car_interfaces/PositionTask.h>

#include <cmath>
#include <algorithm>
#include <Eigen/Dense> // 使用Eigen库来求解多项式系数


// 定位点的结构体
struct Point {
  double x;
  double y;
  double yaw;
};

// 结果输出的结构体
struct outWheelAngle {
  double cte;
  double dhead;
  double wheelangle;
  double gear;
  double throttle;
};

// 全局路径结构体
struct CustomPath {
  std::vector<double> x_list;
  std::vector<double> y_list;
  std::vector<double> yaw_list;
  std::vector<int> gear_list;

  // 添加一个函数来获取路径点的数量
  size_t size() const {
    return x_list.size();  // 假设所有列表大小一致
  }
};


class ControlFSM {

private:
  bool debug_mode;
  //---------------

  ros::ServiceServer service_plan_task_;
  bool handle_control_task(car_interfaces::ControlTask::Request  &req, car_interfaces::ControlTask::Response &res);
  
  ros::ServiceClient position_client;
  car_interfaces::PositionTask position_srv;
  bool callPositionService(int8_t requestData);

  outWheelAngle output_controller, output_controller_rear;

  // MODIFY cyun 
  // 定义状态枚举
  enum class State {
      WAIT_TASK,
      GLOBAL_TASK,
      EXEC_GLOBAL_PATH,
      LOCAL_TASK,
      EXEC_LOCAL_PATH,
      GLOBAL_STOP,
      LOCAL_STOP
  };
  enum class Direction {
    NO=0,
    FORWARD=3,
    BACKWARD=1,
  };
  
  enum class State_control_positionToSmach{
    WAIT_TASK,
    GLOBAL_TASK,
    LOCAL_TASK
  };

  ros::Timer timer_;
  void timerCallback(const ros::TimerEvent&);
  State currentState;
  void transitionState(State newState);
  void coutState();

  // 全局调度
  int task_;
  //全局任务都需要将路径进行一次性处理
  // 是否接收到全局路径
  bool have_global_traj_, have_local_traj_;
  bool global_path_process,local_path_process;
  // 全局路径
  nav_msgs::Path global_path_;

  int fsm_num;

  // 状态发布与反馈
  // std_msgs::Int8 control_state, position_state;
  // 控制程序所处的状态
  // ctrl+; 可以选择剪贴板复制
  int control_state, position_state;
  // ros::Publisher position_state_pub_;
  // ros::Subscriber task_sub_;
  // ros::Publisher cte_pub_, gear_pub_, steering_pub_, brake_pub_;


  std_msgs::Float64 throttle_msg;
  std_msgs::Float64 steering_msg; 
  std_msgs::UInt8 gear_msg;
  std_msgs::Float64 cte_msg;
  std_msgs::Float64 dhead_msg;
  std_msgs::Float64 brake_msg;

  // 定位：
  Point current_position;
  // 计算终点：
  Point dis_current_position;
  Point rear_current_position;
  Point front_current_position;

  // 局部终点距离
  double end_dis, last_end_dis;
  // 额外的结构和函数
  std::vector<CustomPath> map_path;
  std::vector<CustomPath> interpolated_paths;
  
  // 定位数据的x,y临时存储
  Eigen::Vector3d odom_pos_;   // odometry state
  // 车速与航向的存储
  double current_speed_,current_yaw_;   // odometry state
  // 从整个map中找到匹配点的函数
  std::pair<size_t, size_t> findClosestPathPoint(const std::vector<CustomPath>& split_paths, const Point& odom_pos_, double current_yaw);
  size_t map_index, path_index;
  size_t path_index_dis;
  size_t path_index_rear;
  size_t path_index_interpolate;

  size_t interpolate_map_index, interpolate_path_index;
  int controller_location;

  // 计算两点欧式距离
  double calculateDistance(const Point& p1, const Point& p2);
  // 计算航向的偏差，用于过滤不正确的搜索结果
  double calculateYawDifference(double current_yaw, double path_yaw);

  // 已经找到map的最近索引之后，查找指定路径中最近的路径点的索引 
  size_t findClosestPathPointIndex(const CustomPath& path, const Point& odom_pos_);

  // 输出结果
  outWheelAngle CalController(int nearest_index, const CustomPath& path);
  outWheelAngle CalControllerRear(int nearest_index, const CustomPath& path);
  outWheelAngle CalControllerFront(int nearest_index, const CustomPath& path);

  
  void taskCallback(const std_msgs::Int8::ConstPtr& msg);
  // 处理路径
  // std::vector<CustomPath> PathProcess(nav_msgs::Path path);
  // 切分路径
  // std::vector<CustomPath> splitPaths(const CustomPath& path);
  std::pair<std::vector<CustomPath>, std::vector<CustomPath>> splitPaths(const CustomPath& path);
  //速度函数：
  double GetSpeed(double cte, double dhead, double wheel_angle_rad);
  
  std::pair<std::vector<CustomPath>, std::vector<CustomPath>> PathProcess(nav_msgs::Path path);

// -------------------------


  /* planning utils */
//   MPCController::Ptr mpccontroller_;
 std::unique_ptr<StanleyController> stanleycontroller_;  

  /* planning data */
  bool  have_traj_, have_odom_, have_local_;


  // 数据锁
  std::mutex data_mutex_;

  nav_msgs::Path path_,local_path_;  // 路径数据
  int traj_len;  // 路径长度
  int traj_local_len;
  
  std::vector<int> gear_;
  bool is_stop_;    //用于判断是否 已经停稳
  bool emergency_stop_; //用于改变状态 进入停止状态
  int current_gear_; //当前档位
  double max_steering_angle_; // 最大转向角
  double wheelbase_; // 车辆轴距
  int pre_index_;  //档位育苗点，用于判断匹配点和育苗点 之间是否需要换档
  double yawerror;
  double cte;
  double steering_angle_rad;
  double dis_end;
  int pre_match_point; //用于匹配点计算 上一次匹配的点
  double brake;
  double stopDistance;
  double disToend;
  double GlobalTargetSpeed;
  double LocalTargetSpeed;
  

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, test_something_timer_;
  ros::Subscriber path_sub_, odom_sub_ ,stop_sub_,gear_sub_,locale_sub_;
  ros::Publisher throttle_pub_, brake_pub_, steering_pub_, gear_pub_, stop_pub_,cte_pub_,dhead_pub_, ask_fork_ctrl_pub_;
  ros::Subscriber twist_sub_, recv_global_sub_, is_in_local_sub_;
  ros::Subscriber odom_front_sub_, odom_rear_sub_;
  
  double calcHeading(double x1, double y1, double x2, double y2);
  void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);


  void pathCallback(const nav_msgs::Path::ConstPtr& msg);
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void disodometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void rearodometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

  
  void localeCallback(const nav_msgs::Path::ConstPtr& msg);

  void gearCallback(const std_msgs::Int8::ConstPtr& msg);

  // new:
  // 打印的东西进行调整
  size_t total_map_num;
  size_t total_map_interpolated_num;
  
  size_t total_path_size; // 新的map的长度如果发生了变换，那么也要重新处理路了
  std::vector<size_t> interpolated_path_sizes;
  
  // size_t path_size;
  std::vector<size_t> path_sizes;

  // 记录被删除的路径
  std::vector<std::pair<size_t, size_t>> deleted_paths; // 存储 (索引, 长度)
  size_t deleted_count;
  int DeletedPathSize;
  ros::Time lastPrintTime;
  bool sim_mode;

public:
  ControlFSM(/* args */) {
  }
  ~ControlFSM() {
  }

  void init(ros::NodeHandle& nh);
  
};
