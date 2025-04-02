#include "control/Control_FSM.h"

void ControlFSM::init(ros::NodeHandle& nh) 
{
  // exec_state_  = FSM_EXEC_STATE::WAIT_TRAJ;  //执行状态 用于状态机判断

  have_traj_ = false;
  have_odom_ =false;
  have_local_=false;
  debug_mode = false;

  // in_fork_ctrl_=false;
  // is_in_local_ =false;
  // is_in_global_=false;

  pre_match_point=0;
  current_gear_=3; 
  current_gear = 0;

  cte=0; yawerror=0;steering_angle_rad=0;dis_end=0;
  //   /*  fsm param  */
  nh.param("/control/control_main_node/pre_index", pre_index_,0);
  // 重新预瞄
  nh.param("/control/control_main_node/pre_index_front", pre_index_front,0);  
  nh.param("/control/control_main_node/pre_index_normal", pre_index_normal,0);  

  nh.param("/control/control_main_node/debug_mode", debug_mode, false);
  nh.param("/control/control_main_node/have_local_", have_local_, false);

  nh.param("/control/control_main_node/stopDistance", stopDistance, 0.2);
  nh.param("/control/control_main_node/GlobalTargetSpeed", GlobalTargetSpeed, 0.3);
  nh.param("/control/control_main_node/LocalTargetSpeed", LocalTargetSpeed, 0.3);
  nh.param("/control/control_main_node/DeletedPathSize", DeletedPathSize, 5);
  nh.param("/control/stanley_controller/kp_front", kp_front, 0.5);
  nh.param("/control/stanley_controller/kp_normal", kp_normal, 1.0);

  nh.param("/control/control_main_node/Rotation_angle", Rotation_angle, 1.0);
  nh.param("/control/control_main_node/Reverse_distance", Reverse_distance, 1.0);
  nh.param("/control/control_main_node/Throttle_Rotation", Throttle_Rotation, 0.2);
  nh.param("/control/control_main_node/Steering_compensation", Steering_compensation, 0.0);

  // std::cout << Rotation_angle <<" " <<Reverse_distance<<std::endl;
  // std::cout << speed_params_.num_normal_path_speed_max <<" " <<speed_params_.num_normal_path_speed_min<<std::endl;
  nh.param("/speed_params/num_smaller_path_speed_max", speed_params_.num_smaller_path_speed_max, 1.0);
  nh.param("/speed_params/num_smaller_path_speed_min", speed_params_.num_smaller_path_speed_min, 0.4);
  nh.param("/speed_params/num_normal_path_speed_max", speed_params_.num_normal_path_speed_max, 2.0);
  nh.param("/speed_params/num_normal_path_speed_min", speed_params_.num_normal_path_speed_min, 0.25);


  nh.param("/speed_params/speed_very_high", speed_params_.speed_very_high, 2.0);
  nh.param("/speed_params/speed_high", speed_params_.speed_high, 1.7);
  nh.param("/speed_params/speed_medium", speed_params_.speed_medium, 1.5);
  nh.param("/speed_params/speed_medium_medium", speed_params_.speed_medium_medium, 0.7);
  nh.param("/speed_params/speed_low", speed_params_.speed_low, 0.4);
  nh.param("/speed_params/speed_very_low", speed_params_.speed_very_low, 0.25);
  std::cout<<speed_params_.speed_very_low<<std::endl;

  /* initialize main modules */
  //   mpccontroller_.reset(new MPCController(nh)); 
  //释放之前实例，并新创建的MPCController实例,调用构造函数赋值给mpccontroller_
  stanleycontroller_.reset(new StanleyController(nh));
  pdcontroller_.reset(new PDController(nh, 1.77));

  // 订阅话题
  odom_sub_ = nh.subscribe("/odom", 1, &ControlFSM::odometryCallback, this);
  path_sub_ = nh.subscribe("/global_path", 1, &ControlFSM::pathCallback, this);
  // stop_sub_ = nh.subscribe("/stop", 1, &ControlFSM::stopCallback, this);
  gear_sub_ = nh.subscribe("/gear_state", 1, &ControlFSM::gearCallback, this);
  twist_sub_ = nh.subscribe("/twist", 1, &ControlFSM::twistCallback, this); // 速度反馈
  
  locale_sub_ = nh.subscribe("/local_path", 1, &ControlFSM::localeCallback, this);
  odom_front_sub_ = nh.subscribe("/dis_odom", 1, &ControlFSM::disodometryCallback, this);
  odom_front_front_sub_ = nh.subscribe("/front_front_odom", 1, &ControlFSM::frontodometryCallback, this);

  odom_pub = nh.advertise<nav_msgs::Odometry>("/odom_control", 1);

  // 发布控制指令
  throttle_pub_ = nh.advertise<std_msgs::Float64>("/throttle_cmd", 1);
  brake_pub_ = nh.advertise<std_msgs::Float64>("/brake_cmd", 1);
  steering_pub_ = nh.advertise<std_msgs::Float64>("/steering_cmd", 1);
  gear_pub_ = nh.advertise<std_msgs::UInt8>("/gear_cmd", 1);
  // ask_fork_ctrl_pub_ = nh.advertise<std_msgs::Bool>("/position_adjusted", 1);
  service_plan_task_ = nh.advertiseService("control_task", &ControlFSM::handle_control_task, this);
  position_client = nh.serviceClient<car_interfaces::PositionTask>("position_service");
  // 转向反馈：
  steering_state_sub_ = nh.subscribe("/steering_state", 1, &ControlFSM::steeringCallback, this);

  clamp_cmd_pub = nh.advertise<std_msgs::Float64>("/clamp_cmd", 1);
  updown_cmd_pub = nh.advertise<std_msgs::Float64>("/updown_cmd", 1);
  fy_cmd_pub = nh.advertise<std_msgs::Float64>("/fy_cmd", 1);
  lateral_cmd_pub = nh.advertise<std_msgs::Float64>("/lateral_cmd", 1);

  ROS_INFO("Control task service ready.");

  cte_pub_ = nh.advertise<std_msgs::Float64>("/cte_state", 1);
  dhead_pub_ = nh.advertise<std_msgs::Float64>("/dhead_state", 1);

  // 初始化
  currentState = State::WAIT_TASK;
  current_steering_ = 0;
  // TODO 过快导致微分为0
  timer_ = nh.createTimer(ros::Duration(0.02), &ControlFSM::timerCallback, this);
  fsm_num = 0;
  end_dis = std::numeric_limits<double>::max();
  map_index = 0;
  path_index = 0;

  control_state = 0;

  have_global_traj_ = false;
  global_path_process = false;
  exec_kappa = false;

  output_controller.cte = 0;
  output_controller.dhead = 0;
  output_controller.gear = 0;
  output_controller.wheelangle = 0;
  output_controller.throttle = 0;
  deleted_count = 0;
  
  total_map_num = 0;
  total_path_size = 0;
  path_index_dis = 0;
  path_index_front = 0;
  current_path_curvature = 0;
}

bool ControlFSM::callPositionService(int8_t requestData) {
    // if (ros::service::waitForService("position_service")) 
    if (position_client.waitForExistence(ros::Duration(0)))
    {
      ROS_INFO_STREAM("\033[32m Position_service server ready! \033[0m");
      position_srv.request.data = requestData;  // 设置请求值

      if (position_client.call(position_srv)) {
          if (position_srv.response.success) {
              ROS_INFO("Position state sent successfully: %s", position_srv.response.message.c_str());
              return true;
          } else {
              ROS_INFO("Failed to send position state: %s", position_srv.response.message.c_str());
              return false;
          }
      } else {
          ROS_ERROR("Service call failed");
          return false;
      }
    }
    else 
    {
      ROS_ERROR("Service does not exist");
    }
    return false;
}

bool ControlFSM::handle_control_task(car_interfaces::ControlTask::Request  &req, car_interfaces::ControlTask::Response &res)
{
  ROS_INFO("request: control_state = %d", req.data);
  control_state = req.data;
  // 设置响应
  res.success = true;  // 设置success为true
  res.message = "Control state updated successfully.";  // 可以附加一些描述信息
  return true;
}

// 档位回调
void ControlFSM::gearCallback(const std_msgs::Int8::ConstPtr& msg) 
{
  // std::lock_guard<std::mutex> lock(data_mutex_);
  current_gear_ = msg->data;
}


// 局部规划规划回调
void ControlFSM::localeCallback(const nav_msgs::Path::ConstPtr& msg) 
{
  // std::lock_guard<std::mutex> lock(data_mutex_);
  local_path_ = *msg;
  have_local_traj_ = true;
  traj_local_len = static_cast<int>(local_path_.poses.size());
}

// 速度回调
void ControlFSM::twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) 
{
  // std::lock_guard<std::mutex> lock(data_mutex_);
  current_speed_ = msg->twist.linear.x;
}

// 转角回调
void ControlFSM::steeringCallback(const std_msgs::Float64::ConstPtr& msg) 
{
  // std::lock_guard<std::mutex> lock(data_mutex_);
  current_steering_ = msg->data;
}

// 定位回调
void ControlFSM::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) 
{
  // std::lock_guard<std::mutex> lock(data_mutex_);
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  // 从四元数提取航向角
  tf::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  current_yaw_ = yaw; // 弧度
  // current_speed_ = msg->twist.twist.linear.x;
  have_odom_ = true;

  current_position.x = msg->pose.pose.position.x;
  current_position.y = msg->pose.pose.position.y;
  current_position.yaw = yaw; // 更新航向角
}


// 定位回调
void ControlFSM::disodometryCallback(const nav_msgs::Odometry::ConstPtr& msg) 
{
  // 从四元数提取航向角
  tf::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  dis_current_position.x = msg->pose.pose.position.x;
  dis_current_position.y = msg->pose.pose.position.y;
  dis_current_position.yaw = yaw; // 更新航向角
}

void ControlFSM::frontodometryCallback(const nav_msgs::Odometry::ConstPtr& msg) 
{
  // 从四元数提取航向角
  tf::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  front_current_position.x = msg->pose.pose.position.x;
  front_current_position.y = msg->pose.pose.position.y;
  front_current_position.yaw = yaw; // 更新航向角
}


// 计算坐标系下的位移航向 与坐标系航向进行比较来确定档位
double ControlFSM::calcHeading(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return atan2(dy, dx);
}


// 全局路径规划回调
void ControlFSM::pathCallback(const nav_msgs::Path::ConstPtr& msg) 
{
  if(debug_mode)
  {
    std::cout << "pathCallback Succeed!" << std::endl;
  }
  
  have_global_traj_ = true;
  global_path_ = *msg;
  traj_len = static_cast<int>(global_path_.poses.size());
}

double ControlFSM::calculateCurvature(double x1, double y1, double x2, double y2, double x3, double y3) {
    double dx1 = x2 - x1;
    double dy1 = y2 - y1;
    double dx2 = x3 - x2;
    double dy2 = y3 - y2;

    double cross_product = dx1 * dy2 - dy1 * dx2;
    double length1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
    double length2 = std::sqrt(dx2 * dx2 + dy2 * dy2);

    if (length1 * length2 == 0) {
        return 0.0; // 避免除以零
    }
    return cross_product / (length1 * length2);
}
// 全局路径处理 按照档位切分成不同的路径  （少于多少m的路径不要没有加）
// 局部路径处理
// 全局路径需要切分一下，但是局部的应该都是一个档位，不过局部也是会遇到需要倒车的情况
std::vector<CustomPath> ControlFSM::PathProcess(nav_msgs::Path path)
{
  std::vector<double> x_list;
  std::vector<double> y_list;
  std::vector<double> head_list;
  std::vector<double> yaw_list;
  std::vector<int> gear_list;
  std::vector<double> curvature_list;

  geometry_msgs::PoseStamped pose_stamped;

  // 处理路径的x,y,yaw
  for (const auto& pose : path.poses)
  {
    x_list.push_back(pose.pose.position.x);
    y_list.push_back(pose.pose.position.y);
    tf::Quaternion quaternion;
    tf::quaternionMsgToTF(pose.pose.orientation, quaternion);
    double roll, pitch, yaw;
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    yaw_list.push_back(yaw);
  }

  //add 10.17===================================
  // 计算曲率 注意，曲率有正有负，都使用正值。
  for (size_t i = 1; i < x_list.size() - 1; ++i) {
      double curvature = calculateCurvature(x_list[i - 1], y_list[i - 1], x_list[i], y_list[i], x_list[i + 1], y_list[i + 1]);
      curvature_list.push_back(std::fabs(curvature)<1e-5? 0:std::fabs(curvature));
  }
  // 补充首尾点的曲率
  curvature_list.insert(curvature_list.begin(), 0.0);
  curvature_list.push_back(0.0);
  
  
  // 归一化曲率
  double min_curvature = *std::min_element(curvature_list.begin(), curvature_list.end());
  double max_curvature = *std::max_element(curvature_list.begin(), curvature_list.end());
  if (max_curvature - min_curvature > 0) {
    for (double& curvature : curvature_list) {
        curvature = (curvature - min_curvature) / (max_curvature - min_curvature);
    }
  } else {
      // 如果所有曲率都相同，直接归一化为 0
      for (double& curvature : curvature_list) {
          curvature = 0.0;
      }
  }

  //2.0处理档位 cyun ===========================================
  // 计算每个点的航向 head
  for (size_t i = 0; i < x_list.size(); ++i) {
      if (i == 0) {
          // 第一个点的航向与第二个点相同
          head_list.push_back(yaw_list[1]);
      } else if (i == x_list.size() - 1) {
          // 最后一个点的航向与倒数第二个点相同
          head_list.push_back(yaw_list[i - 1]);
      } else {
          // 其他点的航向通过 x, y 的改变来计算
          double dx = x_list[i + 1] - x_list[i - 1];
          double dy = y_list[i + 1] - y_list[i - 1];
          double head = std::atan2(dy, dx);
          head_list.push_back(head);
      }
  }
  // 根据 head 和 yaw 的偏差确定 gear
  for (size_t i = 0; i < head_list.size(); ++i) {
      double yaw = yaw_list[i];
      double head = head_list[i];
      double angle_diff = (head - yaw);
      if (std::fabs(angle_diff) > 1.57) {
          gear_list.push_back(1); // 倒挡
      } else {
          gear_list.push_back(3); // 前进挡
      }
  }
  // 重新给第一个gear:
  // 确保第一个 gear 使用 gear_list[1] 的值
  if (!gear_list.empty()) {
      gear_list[0] = gear_list[1];
      gear_list[gear_list.size()-1] = gear_list[gear_list.size()-2];
  }

  if(debug_mode)
  {
    for (size_t i = 0; i < gear_list.size(); ++i) {
      std::cout <<gear_list[i]<<"\n";
    }
  }

  // 将整个路径作为初始CustomPath
  CustomPath initial_path;
  for (size_t i = 0; i < x_list.size(); ++i) 
  {
    initial_path.x_list.push_back(x_list[i]);
    initial_path.y_list.push_back(y_list[i]);
    initial_path.yaw_list.push_back(yaw_list[i]);
    initial_path.gear_list.push_back(gear_list[i]);
    initial_path.curvature_list.push_back(curvature_list[i]);
    initial_path.throttle_list.push_back(0);
  }
  
  // 切分路径
  return splitPaths(initial_path);
}

int ControlFSM::findNextAvailablePathNumber(const std::string& folderPath) {
    int number = 1;
    std::string filePath;
    do {
        filePath = folderPath + "/path" + std::to_string(number) + ".csv";
        number++;
    } while (std::ifstream(filePath).good());
    return number - 1;
}


// 2.0 按照档位切分路径
std::vector<CustomPath> ControlFSM::splitPaths(const CustomPath& path) {
    std::vector<CustomPath> split_paths;
    CustomPath current_path;
    // 总的点数
    total_path_size = path.gear_list.size() - 1;
    int deleted_path_count = 0;
    // std::vector<std::pair<size_t, size_t>> deleted_paths;

    for (size_t i = 0; i < path.gear_list.size(); ++i) {
        current_path.x_list.push_back(path.x_list[i]);
        current_path.y_list.push_back(path.y_list[i]);
        current_path.yaw_list.push_back(path.yaw_list[i]);
        current_path.curvature_list.push_back(path.curvature_list[i]);
        current_path.gear_list.push_back(path.gear_list[i]);
        current_path.throttle_list.push_back(0);

        if (i < path.gear_list.size() - 1 && path.gear_list[i]!= path.gear_list[i + 1]) {
            if (current_path.size() >= DeletedPathSize) {
                split_paths.push_back(current_path);
            } else {
                // std::cout << "Deleted path with size " << current_path.size() << std::endl;
                deleted_path_count++;
                deleted_paths.push_back({deleted_path_count, current_path.size()});
            }
            current_path = CustomPath();
        }
    }
    // 检查最后一个路径片段
    if (!current_path.x_list.empty() && current_path.size() >= DeletedPathSize) {
        split_paths.push_back(current_path);
    } else if (!current_path.x_list.empty()) {
        // std::cout << "Deleted path with size " << current_path.size() << std::endl;
        deleted_path_count++;
        deleted_paths.push_back({deleted_path_count, current_path.size()});
    }

    // 重新组合所有符合条件的路径点
    CustomPath combined_path;
    for (const auto& sp : split_paths) {
        combined_path.x_list.insert(combined_path.x_list.end(), sp.x_list.begin(), sp.x_list.end());
        combined_path.y_list.insert(combined_path.y_list.end(), sp.y_list.begin(), sp.y_list.end());
        combined_path.yaw_list.insert(combined_path.yaw_list.end(), sp.yaw_list.begin(), sp.yaw_list.end());
        combined_path.gear_list.insert(combined_path.gear_list.end(), sp.gear_list.begin(), sp.gear_list.end());
        combined_path.curvature_list.insert(combined_path.curvature_list.end(), sp.curvature_list.begin(), sp.curvature_list.end());
        combined_path.throttle_list.insert(combined_path.throttle_list.end(),sp.throttle_list.begin(),sp.throttle_list.end());
    }
    
    // for (int i = 0; i <combined_path.x_list.size(); ++i)
    // {
    //     GetSpeed(combined_path.curvature_list,i,combined_path.throttle_list);
    // }
    
    combined_path.throttle_list = GetSpeed(combined_path.curvature_list);
    std::cout<<"current_path size: "<<current_path.size()<<std::endl;

    // 再次按照档位切分
    std::vector<CustomPath> final_split_paths;
    current_path = CustomPath();


    for (size_t i = 0; i < combined_path.gear_list.size(); ++i) {
        current_path.x_list.push_back(combined_path.x_list[i]);
        current_path.y_list.push_back(combined_path.y_list[i]);
        current_path.yaw_list.push_back(combined_path.yaw_list[i]);
        current_path.curvature_list.push_back(combined_path.curvature_list[i]);
        current_path.gear_list.push_back(combined_path.gear_list[i]);
        current_path.throttle_list.push_back(combined_path.throttle_list[i]);
        // file << combined_path.x_list[i] << "," << combined_path.y_list[i] << "," << combined_path.yaw_list[i] << "," << combined_path.curvature_list[i] << "," << combined_path.throttle_list[i] << "," << combined_path.gear_list[i] << "\n";
        if (i < combined_path.gear_list.size() - 1 && combined_path.gear_list[i]!= combined_path.gear_list[i + 1]) {
            final_split_paths.push_back(current_path);
            current_path = CustomPath();
        }
    }
    
    // 检查最后一个路径片段
    // std::cout<<"current_path size: "<<current_path.x_list.size()<<std::endl;

    if (!current_path.x_list.empty()) 
    {
      final_split_paths.push_back(current_path);
    }

    total_map_num = final_split_paths.size();
    for (size_t i = 0; i < final_split_paths.size(); ++i) {
        path_sizes.push_back(final_split_paths[i].size());
    }
    // std::cout << "Total number of deleted paths: " << deleted_path_count << std::endl;
    deleted_count = deleted_path_count;
    // for (const auto& pair : deleted_paths) {
    //     std::cout << "Deleted path " << pair.first << " has size " << pair.second << std::endl;
    // }
    return final_split_paths;
}

// 计算两点之间的欧氏距离
double ControlFSM::calculateDistance(const Point& p1, const Point& p2) 
{
  return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

// 计算两个角度之间的差值，用于判断找到的map索引下的航向是否正常
double ControlFSM::calculateYawDifference(double current_yaw, double path_yaw) 
{
  double diff = (current_yaw - path_yaw);
  if (diff > M_PI)
  {
    diff -= 2*M_PI;
  }
  else if (diff <= -M_PI)
  {
    diff += 2*M_PI;
  }
  return diff;
}

// 查找最近的路径点的索引 复杂度 n^2 可以改进
std::pair<size_t, size_t> ControlFSM::findClosestPathPoint(const std::vector<CustomPath>& split_paths, const Point& odom_pos, double current_yaw) 
{
  double min_distance = std::numeric_limits<double>::max();
  size_t closest_map_index = 0;
  size_t closest_path_index = 0;

  std::cout << "split_paths.size(): " << split_paths.size() << std::endl;
  for (size_t map_index = 0; map_index < split_paths.size(); ++map_index)
  {
      if(debug_mode){
          std::cout << "map_index: " << map_index << std::endl; 
      }
      const CustomPath& path = split_paths[map_index];
      std::cout << "path.x_list.size(): " << path.x_list.size() << std::endl;
      for (size_t path_index = 0; path_index < path.x_list.size(); ++path_index)
      {
          Point current_point = {path.x_list[path_index], path.y_list[path_index], path.yaw_list[path_index]};
          double distance = calculateDistance(odom_pos, current_point);
          double yaw_diff = calculateYawDifference(current_yaw, path.yaw_list[path_index]);
          if(debug_mode)
          {
              std::cout << "map_index: " << map_index << ", path_index: " << path_index
                        << ", distance: " << distance << ", yaw_diff: " << yaw_diff << std::endl; 
          }
          // 如果距离小于最小距离且 yaw 差在可接受范围内
          if (distance < min_distance && yaw_diff <= M_PI / 2) {
              min_distance = distance;
              closest_map_index = map_index;
              closest_path_index = path_index;
              if(debug_mode) {
                  std::cout << "Updated closest_point: map_index: " << closest_map_index 
                            << ", path_index: " << closest_path_index 
                            << ", min_distance: " << min_distance << std::endl;
              }
          }
      }
  }

  return {closest_map_index, closest_path_index};
}


// 已经找到map的最近索引之后，查找指定路径中最近的路径点的索引 
size_t ControlFSM::findClosestPathPointIndex(const CustomPath& path, const Point& odom_pos_) 
{
  double min_distance = std::numeric_limits<double>::max();
  size_t closest_path_index = 0;

  for (size_t path_index = 0; path_index < path.x_list.size(); ++path_index) {
    Point current_point = {path.x_list[path_index], path.y_list[path_index], path.yaw_list[path_index]};
    double distance = calculateDistance(odom_pos_, current_point);

    if (distance < min_distance) 
    {
      min_distance = distance;
      closest_path_index = path_index;
    }
  }
  std::cout << "min_distance: " << min_distance << std::endl;
  return closest_path_index;
}


// 计算控制信息
outWheelAngle ControlFSM::Calkappagear(int nearest_index, const CustomPath& path)
{
  outWheelAngle output_controller;
  output_controller.gear = path.gear_list[nearest_index];
  output_controller.curvature = path.curvature_list[nearest_index];

  return output_controller;
}


// 计算控制信息
outWheelAngle ControlFSM::CalControllerREVERSELocal(int closest_index, int closest_index_modify, double curvature, double pre_curvature, const CustomPath& path, Point loc_pose)
{
  outWheelAngle output_controller;
  // 计算路径点与车辆的横向误差（使用叉乘计算带符号的误差）
  double cte = 0.0;
  // 获取最近点 P1 (x1, y1) 和下一个点 P2 (x2, y2)
  double x1 = path.x_list[std::max(static_cast<size_t>(0), closest_index_modify > 1 ? static_cast<size_t>(closest_index_modify - 2) : static_cast<size_t>(0))];
  double y1 = path.y_list[std::max(static_cast<size_t>(0), closest_index_modify > 1 ? static_cast<size_t>(closest_index_modify - 2) : static_cast<size_t>(0))];

  double x2 = path.x_list[std::min(static_cast<size_t>(path.x_list.size()-1), static_cast<size_t>(closest_index_modify + 2))];
  double y2 = path.y_list[std::min(static_cast<size_t>(path.x_list.size()-1), static_cast<size_t>(closest_index_modify + 2))];

  // 当前车辆位置 (current_x_, current_y_)
  double dx = x2 - x1;
  double dy = y2 - y1;
  // 计算叉乘来判断方向
  double cross_product = (loc_pose.x - x1) * dy - (loc_pose.y - y1) * dx;
  // 计算垂直距离 (带符号的横向误差)
  double distance = std::sqrt(dx * dx + dy * dy);

  if (distance != 0.0)
  {
    cte = cross_product / distance;
  }
  else cte = 0.0;
  if (closest_index_modify >= path.x_list.size()-2){
    cte = 0.0;
  }


  double path_yaw = path.yaw_list[std::min(static_cast<size_t>(path.x_list.size()-1), static_cast<size_t>(closest_index_modify + pre_index_))]; // 预瞄航向误差
  double dhead = path_yaw - loc_pose.yaw;
  if (dhead > M_PI) dhead -= 2 * M_PI;
  if (dhead < -M_PI) dhead += 2 * M_PI;
  
  output_controller.throttle=std::min(GetLowerSpeed(cte, dhead, steering_angle_rad), path.throttle_list[closest_index]);
  // steering_angle_rad = stanleycontroller_->computeSteeringAngle(dhead,cte,current_speed_,curvature, current_steering_, kp_normal);
  steering_angle_rad = pdcontroller_->path_control(cte, dhead*180/M_PI, pre_curvature,curvature, output_controller.throttle);

  output_controller.cte = cte;
  output_controller.dhead = dhead;
  output_controller.gear = path.gear_list[closest_index_modify];
  output_controller.wheelangle = steering_angle_rad;
  
  cte_msg.data = output_controller.cte;
  dhead_msg.data = output_controller.dhead;
  cte_pub_.publish(cte_msg);
  dhead_pub_.publish(dhead_msg);
  // 下发控制信息
  steering_msg.data = -output_controller.wheelangle; // 直接负数，注意，这里不能通过gear给到can中，会导致倒车反转
  steering_pub_.publish(steering_msg);
  gear_msg.data = output_controller.gear;
  gear_pub_.publish(gear_msg);
  // 速度规划部分
  throttle_msg.data = output_controller.throttle;
  throttle_pub_.publish(throttle_msg);

  // brake_msg.data = 0.0;
  // brake_pub_.publish(brake_msg);

  return output_controller;
}

// 计算控制信息
outWheelAngle ControlFSM::CalControllerREVERSE(int closest_index, int closest_index_modify, double curvature, double pre_curvature, const CustomPath& path, Point loc_pose)
{
  outWheelAngle output_controller;
  // 计算路径点与车辆的横向误差（使用叉乘计算带符号的误差）
  double cte = 0.0;

  // 获取最近点 P1 (x1, y1) 和下一个点 P2 (x2, y2)
  double x1 = path.x_list[std::max(static_cast<size_t>(0), closest_index_modify > 1 ? static_cast<size_t>(closest_index_modify - 2) : static_cast<size_t>(0))];
  double y1 = path.y_list[std::max(static_cast<size_t>(0), closest_index_modify > 1 ? static_cast<size_t>(closest_index_modify - 2) : static_cast<size_t>(0))];
  double x2 = path.x_list[std::min(static_cast<size_t>(path.x_list.size()-1), static_cast<size_t>(closest_index_modify + 2))];
  double y2 = path.y_list[std::min(static_cast<size_t>(path.x_list.size()-1), static_cast<size_t>(closest_index_modify + 2))];
  // 当前车辆位置 (current_x_, current_y_)
  double dx = x2 - x1;
  double dy = y2 - y1;
  // 计算叉乘来判断方向
  double cross_product = (loc_pose.x - x1) * dy - (loc_pose.y - y1) * dx;
  // 计算垂直距离 (带符号的横向误差)
  double distance = std::sqrt(dx * dx + dy * dy);
  if (distance != 0.0)
  {
    cte = cross_product / distance;
  }
  else cte = 0.0;
  if (closest_index_modify >= path.x_list.size()-2){
    cte = 0.0;
  }

  double path_yaw = path.yaw_list[std::min(static_cast<size_t>(path.x_list.size()-1), static_cast<size_t>(closest_index_modify + pre_index_))]; // 预瞄航向误差
  double dhead = path_yaw - loc_pose.yaw;
  if (dhead > M_PI) dhead -= 2 * M_PI;
  if (dhead < -M_PI) dhead += 2 * M_PI;

  output_controller.throttle=std::min(GetLowerSpeed(cte, dhead, steering_angle_rad), path.throttle_list[closest_index]);
  steering_angle_rad = pdcontroller_->path_control(cte, dhead*180/M_PI, pre_curvature,curvature, output_controller.throttle);

  cte_msg.data = output_controller.cte;
  dhead_msg.data = output_controller.dhead;
  cte_pub_.publish(cte_msg);
  dhead_pub_.publish(dhead_msg);
  // 下发控制信息
  steering_msg.data = -output_controller.wheelangle; // 直接负数，注意，这里不能通过gear给到can中，会导致倒车反转
  steering_pub_.publish(steering_msg);
  gear_msg.data = output_controller.gear;
  gear_pub_.publish(gear_msg);
  // 速度规划部分
  throttle_msg.data = output_controller.throttle;
  throttle_pub_.publish(throttle_msg);

  // brake_msg.data = 0.0;
  // brake_pub_.publish(brake_msg);

  return output_controller;
}


// 计算控制信息
outWheelAngle ControlFSM::CalControllerLocal(int closest_index, int closest_index_modify,double curvature, double pre_curvature, const CustomPath& path,Point loc_pose)
{
  outWheelAngle output_controller;
  // 计算路径点与车辆的横向误差（使用叉乘计算带符号的误差）
  double cte = 0.0;

  // 获取最近点 P1 (x1, y1) 和下一个点 P2 (x2, y2)
  // double x1 = path.x_list[std::max(0, static_cast<size_t>(nearest_index - 2))];
  // double x1 = path.x_list[std::max(static_cast<size_t>(0), nearest_index > 1 ? nearest_index - 2 : 0)];
  double x1 = path.x_list[std::max(static_cast<size_t>(0), closest_index_modify > 1 ? static_cast<size_t>(closest_index_modify - 2) : static_cast<size_t>(0))];

  // double y1 = path.y_list[nearest_index];
  double y1 = path.y_list[std::max(static_cast<size_t>(0), closest_index_modify > 1 ? static_cast<size_t>(closest_index_modify - 2) : static_cast<size_t>(0))];

  double x2 = path.x_list[std::min(static_cast<size_t>(path.x_list.size()-1), static_cast<size_t>(closest_index_modify + 2))];
  double y2 = path.y_list[std::min(static_cast<size_t>(path.x_list.size()-1), static_cast<size_t>(closest_index_modify + 2))];

  // 当前车辆位置 (current_x_, current_y_)
  double dx = x2 - x1;
  double dy = y2 - y1;

  // 计算叉乘来判断方向
  double cross_product = (loc_pose.x - x1) * dy - (loc_pose.y - y1) * dx;

  // 计算垂直距离 (带符号的横向误差)
  double distance = std::sqrt(dx * dx + dy * dy);

  if (distance != 0.0)
  {
    cte = cross_product / distance;
  }
  else cte = 0.0;
  if (closest_index_modify >= path.x_list.size()-2){
    cte = 0.0;
  }
  double path_yaw = path.yaw_list[std::min(static_cast<size_t>(path.x_list.size()-1), static_cast<size_t>(closest_index_modify + pre_index_))]; // 预瞄航向误差
  double dhead = path_yaw - loc_pose.yaw; // 弧度制
  if (dhead > M_PI) dhead -= 2 * M_PI;
  if (dhead < -M_PI) dhead += 2 * M_PI;

  output_controller.throttle=std::min(GetLowerSpeed(cte, dhead, steering_angle_rad), path.throttle_list[closest_index]);
  // steering_angle_rad = stanleycontroller_->computeSteeringAngle(dhead,cte,current_speed_,curvature, current_steering_, kp_normal);
  steering_angle_rad = pdcontroller_->path_control(cte, dhead*180/M_PI, pre_curvature,curvature, output_controller.throttle);

  output_controller.cte = cte;
  output_controller.dhead = dhead;
  output_controller.gear = path.gear_list[closest_index_modify];
  output_controller.wheelangle = steering_angle_rad;

  cte_msg.data = output_controller.cte;
  dhead_msg.data = output_controller.dhead;
  cte_pub_.publish(cte_msg);
  dhead_pub_.publish(dhead_msg);
  steering_msg.data = output_controller.wheelangle; // 弧度值

  steering_pub_.publish(steering_msg);
  gear_msg.data = output_controller.gear;
  gear_pub_.publish(gear_msg);
  throttle_msg.data = output_controller.throttle;
  throttle_pub_.publish(throttle_msg);
  // brake_msg.data = 0.0; //不要发，需要才发
  // brake_pub_.publish(brake_msg);
  return output_controller;
}


// 计算控制信息
outWheelAngle ControlFSM::CalController(int closest_index, int closest_index_modify, double curvature, double pre_curvature, const CustomPath& path, Point loc_pose)
{
  outWheelAngle output_controller;
  // 计算路径点与车辆的横向误差（使用叉乘计算带符号的误差）
  double cte = 0.0;
  // 获取最近点 P1 (x1, y1) 和下一个点 P2 (x2, y2)

  int index_before = std::max(0, closest_index_modify + 1);
  int index_after = std::min(static_cast<int>(path.x_list.size() - 1), closest_index_modify + 2);
  std::cout << "index_before" << index_before << "index_after" << index_after << std::endl;
  double x1 = path.x_list[index_before];
  double y1 = path.y_list[index_before];
  double x2 = path.x_list[index_after];
  double y2 = path.y_list[index_after];
  // 当前车辆位置 (current_x_, current_y_)
  double dx = x2 - x1;
  double dy = y2 - y1;
  // 计算叉乘来判断方向
  double cross_product = (loc_pose.y - y1) * dx - (loc_pose.x - x1) * dy;
  // std::cout << "cross_product" << cross_product << std::endl;
  // 计算垂直距离 (带符号的横向误差)
  double distance = std::sqrt(dx * dx + dy * dy);
  std::cout << "x1" << x1 << "y1" << y1 << "x2" << x2 << "y2" << y2 << std::endl;
  std::cout << "loc_pose.x" << loc_pose.x << "loc_pose.y" << loc_pose.y << std::endl;
  std::cout << "dx" << dx << "dy" << dy << std::endl;
  std::cout << "distance" << distance << "cross_product" << cross_product << std::endl;

  // std::cout << "distance" << distance << std::endl;
  if (distance != 0.0)
  {
    cte = cross_product / distance;
  }
  else cte = 0.0;
  if (closest_index_modify >= path.x_list.size()-2){
    cte = 0.0;
  }
  double path_yaw = path.yaw_list[std::min(static_cast<size_t>(path.x_list.size()-1), static_cast<size_t>(closest_index_modify + pre_index_))]; // 预瞄航向误差
  double dhead = path_yaw - loc_pose.yaw; // 弧度制
  if (dhead > M_PI) dhead -= 2 * M_PI;
  if (dhead < -M_PI) dhead += 2 * M_PI;
  
  output_controller.throttle=std::min(GetLowerSpeed(cte, dhead, steering_angle_rad),path.throttle_list[closest_index]);  
  // steering_angle_rad = stanleycontroller_->computeSteeringAngle(dhead,cte,current_speed_,curvature, current_steering_, kp_normal);
  steering_angle_rad = pdcontroller_->path_control(cte, dhead*180/M_PI, pre_curvature, curvature, output_controller.throttle);

  output_controller.cte = cte;
  output_controller.dhead = dhead;
  output_controller.gear = path.gear_list[closest_index_modify];
  output_controller.wheelangle = steering_angle_rad;

  cte_msg.data = output_controller.cte;
  dhead_msg.data = output_controller.dhead;
  cte_pub_.publish(cte_msg);
  dhead_pub_.publish(dhead_msg);
  // 下发控制信息
  steering_msg.data = output_controller.wheelangle; // 弧度值

  steering_pub_.publish(steering_msg);
  gear_msg.data = output_controller.gear;
  gear_pub_.publish(gear_msg);
  // 速度规划部分
  throttle_msg.data = output_controller.throttle;
  throttle_pub_.publish(throttle_msg);

  // brake_msg.data = 0.0;
  // brake_pub_.publish(brake_msg);

  return output_controller;
}


// 根据控制效果设置速度
double ControlFSM::GetLowerSpeed(double cte, double dhead, double wheel_angle_rad) {
    // 将dhead从弧度转换为角度
    double dhead_degrees = dhead * (180.0 / M_PI);
    double wheel_angle  = wheel_angle_rad * (180.0 / M_PI);
    
    // 终点减速
    if(end_dis<1.5&&end_dis>=0.5){
        return speed_params_.speed_low;
    }
    else if(end_dis<0.5){
        return speed_params_.speed_very_low;
    }

    // 转向减速
    if(wheel_angle>65) return 0.2;
    else if(wheel_angle>50) return 0.25;
    else if(wheel_angle>35) return 0.35;
    else if(wheel_angle>20) return 0.4;

    // 根据CTE（横向误差）和dhead_degrees确定速度
    if (std::abs(cte) <= 0.15 && std::abs(dhead_degrees) <= 6) {
        return speed_params_.speed_very_high;
    } else if (std::abs(cte) > 0.15 && std::abs(cte) <= 0.5 && std::abs(dhead_degrees) > 6 && std::abs(dhead_degrees) <= 12) {
        return speed_params_.speed_high;
    }
    else if (std::abs(cte) > 0.5 && std::abs(cte) <= 0.6 && std::abs(dhead_degrees) > 12 && std::abs(dhead_degrees) <= 20) {
        return speed_params_.speed_medium_medium;
    }
     else if (std::abs(cte) > 0.6 && std::abs(cte) <= 0.8 && std::abs(dhead_degrees) > 20 && std::abs(dhead_degrees) <= 32) {
        return speed_params_.speed_low;
    } else if (std::abs(cte) > 0.8 && std::abs(dhead_degrees) > 32) {
        return speed_params_.speed_very_low;
    }
    return speed_params_.speed_medium_medium;
}

// 根据路径的信息获取速度
std::vector<double> ControlFSM::GetSpeed(const std::vector<double>& curvature_list) {
    std::vector<double> throttle_list;
    //初始化 0.5m/s速度
    throttle_list.resize(curvature_list.size(), 0.5);

    int num_points = curvature_list.size();
    // 一个点0.1m
    int deceleration_range_before = 30; //往前找30个点
    int deceleration_range_after = 30;  //往后找30个点
    //control_state 1 取货放货 2 长路段；
    // 如果长度不长，那么需要在曲率面前大量减速
    if (num_points < 85) {
        for (int i = 0; i < num_points; ++i) {
            double curvature = curvature_list[i];
            if (curvature <= 0.1) {
                //throttle_list[i] = 0.7;
                throttle_list[i] = speed_params_.num_smaller_path_speed_max;
            }
            if (curvature > 0.1) {
                int start_index = std::max(0, i - deceleration_range_before);
                int end_index = std::min(i + deceleration_range_after, num_points - 1);
                for (int j = start_index; j <= end_index; ++j) {
                  //throttle_list[j] = 0.4;
                  throttle_list[i] = speed_params_.num_smaller_path_speed_min;
                }
            }
        }
    }
    // 如果长度够长，那么曲率不需要特别减速也能继续
    else {
        for (int i = 0; i < num_points; ++i) {
            double curvature = curvature_list[i];
            if (curvature <= 0.1) {
                // throttle_list[i] = 2.0;
                throttle_list[i] = speed_params_.num_normal_path_speed_max;
            }
            if (curvature > 0.1) {
                int start_index = std::max(0, i - deceleration_range_before);
                int end_index = std::min(i + deceleration_range_after, num_points - 1);
                for (int j = start_index; j <= end_index; ++j) {
                    // throttle_list[j] = 0.25;
                    throttle_list[j] = speed_params_.num_normal_path_speed_min;
                }
            }
        }
    }
    throttle_list[throttle_list.size()-1] = 0.2;
    return throttle_list;
}


// 状态转换
void ControlFSM::transitionState(State newState) 
{
  std::string state_str[12] = { "WAIT_TASK", "GLOBAL_TASK", "EXEC_GLOBAL_PATH", "EXEC_GLOBAL_NOT_TURN","EXEC_REVERSE","EXEC_KAPPA_VERTICAL","EXEC_KAPPA","EXEC_KAPPA_ZERO", "LOCAL_TASK","EXEC_LOCAL_PATH","GLOBAL_STOP","LOCAL_STOP"};
  int pre_s  = int(currentState);
  currentState = newState;
  std::cout << "\033[1;33m From " + state_str[pre_s] + " To " + state_str[int(newState)] << std::endl;
}

void ControlFSM::coutState() {
    std::string state_str[12] = { "WAIT_TASK", "GLOBAL_TASK", "EXEC_GLOBAL_PATH","EXEC_GLOBAL_NOT_TURN","EXEC_REVERSE","EXEC_KAPPA_VERTICAL","EXEC_KAPPA","EXEC_KAPPA_ZERO", "LOCAL_TASK","EXEC_LOCAL_PATH","GLOBAL_STOP","LOCAL_STOP"};
    int pre_s = int(currentState);
    ROS_INFO_STREAM("\033[1;32m State is: "<<state_str[pre_s]);
}

Point ControlFSM::interpolatePoints(const Point& start, const Point& end, double k) {
    Point result;
    result.x = start.x + k * (end.x - start.x);
    result.y = start.y + k * (end.y - start.y);
    result.yaw = start.yaw + k * (end.yaw - start.yaw);
    return result;
}

void ControlFSM::publishOdom(const Point& position) {
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "map";
    odom.child_frame_id = "front_control_axle";

    odom.pose.pose.position.x = position.x;
    odom.pose.pose.position.y = position.y;
    odom.pose.pose.position.z = 0.0;

    // Convert yaw to quaternion
    tf::Quaternion q;
    q.setRPY(0, 0, position.yaw);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom_pub.publish(odom);
}

// distance正为前进，负为后退
Point ControlFSM::movePointAlongHeading(const Point& currentPoint, double distance) {
    Point newPoint;
        newPoint.x = currentPoint.x + distance * std::cos(currentPoint.yaw);
        newPoint.y = currentPoint.y + distance * std::sin(currentPoint.yaw);
    newPoint.yaw = currentPoint.yaw;
    return newPoint;
}

double ControlFSM::normalizeAngle(double angle) {
    while (angle > 180) {
        angle -= 360;
    }
    while (angle < -180) {
        angle += 360;
    }
    return angle;
}

std::pair<int, double> ControlFSM::calculateOptimalRotation(double currentYaw, double goalYaw) {
    double rawAngle = goalYaw - currentYaw;
    double normalizedAngle = normalizeAngle(rawAngle);

    double clockwiseAngle, counterclockwiseAngle;
    if (normalizedAngle > 0) {
        clockwiseAngle = 360 - normalizedAngle;
        counterclockwiseAngle = normalizedAngle;
    } else {
        clockwiseAngle = -normalizedAngle;
        counterclockwiseAngle = 360 + normalizedAngle;
    }

    int direction;
    double rotationAngle;
    
    std::cout << "\033[1;33m clockwiseAngle " << clockwiseAngle << " counterclockwiseAngle " << counterclockwiseAngle<< std::endl;
    if (clockwiseAngle < counterclockwiseAngle) {
        direction = 1; // 逆时针
        rotationAngle = clockwiseAngle;
    } else {
        direction = 2; // 顺时针
        rotationAngle = counterclockwiseAngle;
    }

    return std::make_pair(direction, rotationAngle / 180 * M_PI);
}

void ControlFSM::timerCallback(const ros::TimerEvent&) {

  ros::Time start_time = ros::Time::now();

  static int count = 0; // 定义一个静态计数器
  const int print_interval = 100; // 设置打印间隔

  // 全局和局部任务接收
  switch (currentState)
  {
    //1.等待发布任务===================================
    // 如果接受到全局任务，跳转到全局/局部任务状态
    case State::WAIT_TASK:
    {
      // 重新初始化：
      // global_path_process = false;
      // local_path_process = false;
      exec_kappa = false;
      output_controller.gear = 0;
      output_controller.wheelangle = 0.0;
      output_controller.throttle = 0.0;
      brake_msg.data = 0.0;
      map_index = 0;
      path_index = 0;
      end_dis = 0;
      traj_len = 0;  // 只有在wait中才会清理 新的订阅path能够重新有值。
      global_path_.poses.clear();
      traj_local_len = 0;
      total_map_num = 0;
      total_path_size = 0;
      path_sizes.clear();

      // 0.自检部分，需要一直都有定位信息：
      if (!have_odom_)
      {
        ROS_INFO_ONCE("\033[1;31m Wait Odom ");
        // ROS_INFO_STREAM("\033[1;33m control_state: "<<control_state);
        return;
      }
      else
      {
        ROS_INFO_ONCE("\033[1;32m Odom Get");
      }

      if (control_state==1)
      {
        transitionState(State::GLOBAL_TASK);
      }
      else if (control_state==2)
      {
        transitionState(State::LOCAL_TASK);
      }

      else
      { 
        ROS_INFO_ONCE("\033[1;31m Wait Smach ");
      }
      break;
    }

    //2.全局任务处理===================================
    case State::GLOBAL_TASK:
    {
      if (control_state!=1)
      {
        transitionState(State::WAIT_TASK);
      }

      if (global_path_.poses.size()!=0)
      {
        map_path = PathProcess(global_path_); // std::vector<CustomPath> map_path
        if (map_path.size() == 0 ) break;
        // 查找最近的路径点
        std::pair<size_t, size_t> closest_index = findClosestPathPoint(map_path, current_position, current_yaw_);
        map_index = closest_index.first; 
        path_index = closest_index.second;
        if(control_state==3) transitionState(State::EXEC_GLOBAL_PATH); // 原地转向状态
        else if(control_state==1) transitionState(State::EXEC_GLOBAL_NOT_TURN); // 正常循迹状态
        else if(control_state==2) transitionState(State::EXEC_LOCAL_PATH);
      } 
      break;
    }

    //2.全局任务处理===================================
    case State::EXEC_GLOBAL_NOT_TURN:
    {
      if (control_state!=1)
      {
        transitionState(State::WAIT_TASK);
      }
      Point end_point_ = {map_path[map_index].x_list.back(), map_path[map_index].y_list.back(), map_path[map_index].yaw_list.back()};
      // 使用始终front_axle轴的定位计算终点距离 dis_current_position-前轴定位
      end_dis = calculateDistance(dis_current_position, end_point_);
      size_t closest_index_dis = findClosestPathPointIndex(map_path[map_index], dis_current_position);
      path_index_dis = closest_index_dis;
      // 档位
      current_gear = map_path[map_index].gear_list[closest_index_dis];

      if (current_gear==1){
        size_t closest_index = findClosestPathPointIndex(map_path[map_index], current_position);
        current_path_curvature = map_path[map_index].curvature_list[closest_index];
        current_position_modify = movePointAlongHeading(current_position, -1.0);
        size_t closest_index_modify = findClosestPathPointIndex(map_path[map_index], current_position_modify);
        path_index = closest_index;
        publishOdom(current_position_modify);
        output_controller = CalControllerREVERSE(closest_index, closest_index_modify, current_path_curvature, current_path_curvature_modify, map_path[map_index], current_position);
      }else{
        size_t closest_index = findClosestPathPointIndex(map_path[map_index], current_position);
        current_path_curvature = map_path[map_index].curvature_list[closest_index];
        current_position_modify = movePointAlongHeading(current_position, 2.0);
        size_t closest_index_modify = findClosestPathPointIndex(map_path[map_index], front_current_position);
        current_path_curvature_modify = map_path[map_index].curvature_list[closest_index_modify];
        publishOdom(current_position_modify);
        // std::cout << "\033[1;33m closest_index " << closest_index << " closest_index_modify " << closest_index_modify<< std::endl;
        output_controller = CalController(closest_index, closest_index_modify, current_path_curvature, current_path_curvature_modify, map_path[map_index], current_position);
      }

      if (end_dis<0.05 || closest_index_dis == map_path[map_index].x_list.size()-1)
      {
        transitionState(State::GLOBAL_STOP);
      }
      break;
    }


    //2.全局任务结束==================================
    case State::GLOBAL_STOP:
    {
      if (control_state!=1)
      {
        transitionState(State::WAIT_TASK);
      }

      steering_msg.data = 0.0;
      steering_pub_.publish(steering_msg);
      gear_msg.data = int(Direction::NO);  
      gear_pub_.publish(gear_msg);
      throttle_msg.data = 0.0;
      throttle_pub_.publish(throttle_msg);
      brake_msg.data = 4;  
      brake_pub_.publish(brake_msg);

      if(current_speed_<0.001) //判断是否刹停
      {
          if (map_index<(map_path.size()-1)) 
          {
            map_index += 1;
            transitionState(State::EXEC_GLOBAL_PATH); 
          }
          else
          {
            int8_t requestData = 1;
            bool result = callPositionService(requestData);
            std::cout<<"result:"<<result<<std::endl;
            while (!result){
              return;
            }
            if(result){
              control_state = 0;
              transitionState(State::WAIT_TASK);
            }
          }
      }
      break;
    }


    case State::LOCAL_TASK:
    {
      if (control_state!=1&&control_state!=2)
      {
        transitionState(State::WAIT_TASK);
      }

      if (global_path_.poses.size()!=0)
      {
        map_path = PathProcess(global_path_); // std::vector<CustomPath> map_path
        if (map_path.size() == 0 ) break;
        // 查找最近的路径点
        std::pair<size_t, size_t> closest_index = findClosestPathPoint(map_path, current_position, current_yaw_);
        map_index = closest_index.first; 
        path_index = closest_index.second;
        if(control_state==3) transitionState(State::EXEC_GLOBAL_PATH); // 原地转向状态
        else if(control_state==1) transitionState(State::EXEC_GLOBAL_NOT_TURN); // 正常循迹状态
        else if(control_state==2) transitionState(State::EXEC_LOCAL_PATH);
      }
      break;
    }

    //2.全局任务处理===================================
    case State::EXEC_LOCAL_PATH:
    {
      if (control_state!=2)
      {
        transitionState(State::WAIT_TASK);
      }

      Point end_point_ = {map_path[map_index].x_list.back(), map_path[map_index].y_list.back(), map_path[map_index].yaw_list.back()};
      // 使用始终front_axle轴的定位计算终点距离 dis_current_position-前轴定位
      end_dis = calculateDistance(dis_current_position, end_point_);
      size_t closest_index_dis = findClosestPathPointIndex(map_path[map_index], dis_current_position);
      path_index_dis = closest_index_dis;
      // 档位
      current_gear = map_path[map_index].gear_list[closest_index_dis];

      if (current_gear==1){
        size_t closest_index = findClosestPathPointIndex(map_path[map_index], current_position);
        current_path_curvature = map_path[map_index].curvature_list[closest_index];
        current_position_modify = movePointAlongHeading(current_position, -1.0);
        size_t closest_index_modify = findClosestPathPointIndex(map_path[map_index], current_position_modify);
        path_index = closest_index;
        publishOdom(current_position_modify);
        output_controller = CalControllerREVERSELocal(closest_index, closest_index_modify, current_path_curvature, current_path_curvature_modify, map_path[map_index], current_position);
      }else{
        size_t closest_index = findClosestPathPointIndex(map_path[map_index], current_position);
        current_path_curvature = map_path[map_index].curvature_list[closest_index];
        current_position_modify = movePointAlongHeading(current_position, 2.0);
        size_t closest_index_modify = findClosestPathPointIndex(map_path[map_index], current_position_modify);
        current_path_curvature_modify = map_path[map_index].curvature_list[closest_index_modify];
        publishOdom(current_position_modify);
        output_controller = CalControllerLocal(closest_index, closest_index_modify, current_path_curvature, current_path_curvature_modify, map_path[map_index], current_position);
      }
      if (end_dis<0.05 || closest_index_dis == map_path[map_index].x_list.size()-1)
      {
        transitionState(State::LOCAL_STOP);
      }
      break;
    }

    //2.全局任务结束==================================
    case State::LOCAL_STOP:
    {
      if (control_state!=1&& control_state!=2)
      {
        transitionState(State::WAIT_TASK);
      }
      steering_msg.data = 0.0;
      steering_pub_.publish(steering_msg);
      gear_msg.data = int(Direction::NO);  
      gear_pub_.publish(gear_msg);
      throttle_msg.data = 0.0;
      throttle_pub_.publish(throttle_msg);
      brake_msg.data = 4;  
      brake_pub_.publish(brake_msg);

      if(current_speed_<0.001) //判断是否刹停
      {
          if (map_index<(map_path.size()-1)) 
          {
            map_index += 1;
            transitionState(State::EXEC_GLOBAL_PATH); 
          }
          else
          {
            int8_t requestData = 1;
            bool result = callPositionService(requestData);
            std::cout<<"result:"<<result<<std::endl;
            while (!result){
              return;
            }
            if(result){
              control_state = 0;
              transitionState(State::WAIT_TASK);
            }
          }
      }
      break;
    }
  } //switch

  ros::Time end_time = ros::Time::now();
  ros::Duration exec_time = end_time - start_time;  // 计算执行时间

  // 定次数打印，防止刷屏
  ros::Time currentTime = ros::Time::now();
  if ((currentTime - lastPrintTime).toSec() >= 1.0) 
  {
    ROS_INFO("--------- Control FSM ---------");
    // ROS_INFO_STREAM("\033[1;32m Total_map_num: " << total_map_num);
    // ROS_INFO_STREAM("\033[1;32m total_path_size: "<<total_path_size);

    // for (size_t i = 0; i < path_sizes.size(); ++i) 
    // {
    //   ROS_INFO_STREAM("\033[1;32m path_size_ "<<i<< " : " << path_sizes[i]);
    // }
    // // ROS_INFO_STREAM("\033[1;32m current_path_curvature: " << current_path_curvature);
    // if(debug_mode)
    // {
      // ROS_INFO_STREAM("CTE: " << output_controller.cte);
      // ROS_INFO_STREAM("dHead: " << output_controller.dhead);
      // ROS_INFO_STREAM("WheelAngle: " << output_controller.wheelangle);
    // }
    // // 打印被删除的路径信息
    // ROS_INFO_STREAM("\033[1;32m Total number of deleted paths: " << deleted_count);
    // for (const auto& pair : deleted_paths) 
    // {
    //   ROS_INFO_STREAM("\033[1;32m Deleted path "<<pair.first<<" with size: "<< pair.second);
    // }
    // ROS_INFO_STREAM("\033[1;32m Now map_index: " << map_index);
    // ROS_INFO_STREAM("\033[1;32m Now path_index: "<<path_index);
    ROS_INFO_STREAM("\033[1;32m end_dis: " << end_dis);
    coutState();
    ROS_INFO_STREAM("Execution time: " << exec_time.toSec() << " seconds");
    ROS_INFO("--------- Control FSM ---------");
    lastPrintTime = currentTime;
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ControlFSM");
  ros::NodeHandle nh;
  ControlFSM fsm;
  fsm.init(nh);
  ros::spin();
  return 0;
}