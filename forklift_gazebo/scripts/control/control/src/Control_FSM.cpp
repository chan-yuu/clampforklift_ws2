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

  cte=0; yawerror=0;steering_angle_rad=0;dis_end=0;
  //   /*  fsm param  */
  nh.param("/control/control_main_node/pre_index", pre_index_,0);  
  nh.param("/control/control_main_node/debug_mode", debug_mode, false);
  nh.param("/control/control_main_node/have_local_", have_local_, false);

  nh.param("/control/control_main_node/stopDistance", stopDistance, 0.2);
  nh.param("/control/control_main_node/GlobalTargetSpeed", GlobalTargetSpeed, 0.3);
  nh.param("/control/control_main_node/LocalTargetSpeed", LocalTargetSpeed, 0.3);
  nh.param("/control/control_main_node/DeletedPathSize", DeletedPathSize, 5);

  nh.param("/control/control_main_node/SIM", sim_mode, true);


  /* initialize main modules */
  //   mpccontroller_.reset(new MPCController(nh)); 
  //释放之前实例，并新创建的MPCController实例,调用构造函数赋值给mpccontroller_
  stanleycontroller_.reset(new StanleyController(nh));
  
  // 订阅话题
  odom_sub_ = nh.subscribe("/odom", 10, &ControlFSM::odometryCallback, this);
  path_sub_ = nh.subscribe("/global_path", 10, &ControlFSM::pathCallback, this);
  // stop_sub_ = nh.subscribe("/stop", 10, &ControlFSM::stopCallback, this);
  gear_sub_ = nh.subscribe("/gear_state", 10, &ControlFSM::gearCallback, this);
  twist_sub_ = nh.subscribe("/twist", 10, &ControlFSM::twistCallback, this);
  locale_sub_ = nh.subscribe("/local_path", 10, &ControlFSM::localeCallback, this);
  odom_front_sub_ = nh.subscribe("/dis_odom", 10, &ControlFSM::disodometryCallback, this);
  odom_rear_sub_ = nh.subscribe("/rear_odom", 10, &ControlFSM::rearodometryCallback, this);

  // 发布控制指令
  throttle_pub_ = nh.advertise<std_msgs::Float64>("/throttle_cmd", 1);
  brake_pub_ = nh.advertise<std_msgs::Float64>("/brake_cmd", 1);
  steering_pub_ = nh.advertise<std_msgs::Float64>("/steering_cmd", 1);
  gear_pub_ = nh.advertise<std_msgs::UInt8>("/gear_cmd", 1);
  // ask_fork_ctrl_pub_ = nh.advertise<std_msgs::Bool>("/position_adjusted", 1);
  service_plan_task_ = nh.advertiseService("control_task", &ControlFSM::handle_control_task, this);
  position_client = nh.serviceClient<car_interfaces::PositionTask>("position_service");
  
  
  ROS_INFO("Control task service ready.");

  cte_pub_ = nh.advertise<std_msgs::Float64>("/cte_state", 1);
  dhead_pub_ = nh.advertise<std_msgs::Float64>("/dhead_state", 1);

  // 初始化
  currentState = State::WAIT_TASK;
  timer_ = nh.createTimer(ros::Duration(0.01), &ControlFSM::timerCallback, this);
  fsm_num = 0;
  end_dis = std::numeric_limits<double>::max();
  map_index = 0;
  path_index = 0;

  control_state = 0;

  have_global_traj_ = false;
  global_path_process = false;

  output_controller.cte = 0;
  output_controller.dhead = 0;
  output_controller.gear = 0;
  output_controller.wheelangle = 0;
  output_controller.throttle = 0;
  deleted_count = 0;
  
  total_map_num = 0;
  total_path_size = 0;
  controller_location = 0;
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


// 定位回调
void ControlFSM::rearodometryCallback(const nav_msgs::Odometry::ConstPtr& msg) 
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

  rear_current_position.x = msg->pose.pose.position.x;
  rear_current_position.y = msg->pose.pose.position.y;
  rear_current_position.yaw = yaw; // 更新航向角
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

// 全局路径处理 按照档位切分成不同的路径  （少于多少m的路径不要没有加）
// 局部路径处理
// 全局路径需要切分一下，但是局部的应该都是一个档位，不过局部也是会遇到需要倒车的情况
std::pair<std::vector<CustomPath>, std::vector<CustomPath>> ControlFSM::PathProcess(nav_msgs::Path path)
{
  std::vector<double> x_list;
  std::vector<double> y_list;
  std::vector<double> head_list;
  std::vector<double> yaw_list;
  std::vector<int> gear_list;

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
    // std::cout << "Yaw: " << yaw*180/M_PI << " radians";
  }

  //处理档位 ===========================================
  for (int i=1;i <x_list.size();i++)
  {
    //每一点速度方向
    double dx = path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x;
    double dy = path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y;

    // 计算点积
    double dotProduct = dx * cos(yaw_list[i]) + dy * sin(yaw_list[i]);
    // 判断前进还是倒车
    if (dotProduct > 0) {
        //FORWAWRD
        gear_list.push_back(int(Direction::FORWARD));
    } else if (dotProduct < 0) {
        gear_list.push_back(int(Direction::BACKWARD));
    }else{
       gear_list.push_back(int(Direction::FORWARD));
    }
  }
  gear_list.insert(gear_list.begin(), gear_list[0]);
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
  }
  // 切分路径
  return splitPaths(initial_path);
}


// 使用Eigen库来求解五次多项式系数
Eigen::VectorXd solveQuinticCoefficients(double x0, double y0, double x1, double y1, double dx0, double dy0, double dx1, double dy1) {
    Eigen::MatrixXd A(6, 6);
    Eigen::VectorXd B(6);

    // 设置方程组
    A << 0, 0, 0, 0, 0, 1,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 2, 0, 0,
         1, 1, 1, 1, 1, 1,
         5, 4, 3, 2, 1, 0,
         20, 12, 6, 2, 0, 0;

    B << x0, dx0, 0, x1, dx1, 0;

    // 求解系数
    Eigen::VectorXd coefficients = A.colPivHouseholderQr().solve(B);

    return coefficients;
}


// 插入路径点的函数
void insertPoints(CustomPath& path, double target_length) {
  if (path.size() < 2) return; // 至少需要两个点才能进行插值

  // 计算原始路径长度
  double original_length = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    original_length += std::sqrt(std::pow(path.x_list[i] - path.x_list[i-1], 2) +
                                 std::pow(path.y_list[i] - path.y_list[i-1], 2));
  }

  // 如果原始路径已经足够长，直接返回
  if (original_length >= target_length) return;

  // 计算五次多项式系数
  double x0 = path.x_list.front();
  double y0 = path.y_list.front();
  double x1 = path.x_list.back();
  double y1 = path.y_list.back();

  double dx0 = 0.0; // 假设起点的导数为0
  double dy0 = 0.0; // 假设起点的导数为0
  double dx1 = 0.0; // 假设终点的导数为0
  double dy1 = 0.0; // 假设终点的导数为0

  Eigen::VectorXd x_coefficients = solveQuinticCoefficients(x0, y0, x1, y1, dx0, dy0, dx1, dy1);
  Eigen::VectorXd y_coefficients = solveQuinticCoefficients(y0, x0, y1, x1, dy0, dx0, dy1, dx1);

  // 插值
  std::vector<double> new_x_list, new_y_list, new_yaw_list;
  std::vector<int> new_gear_list;

  double current_length = 0.0;
  double step = 0.01; // 插值步长
  for (double t = 0.0; t <= 1.0; t += step) {
    double x = x_coefficients(0) * std::pow(t, 5) + x_coefficients(1) * std::pow(t, 4) + x_coefficients(2) * std::pow(t, 3) +
               x_coefficients(3) * std::pow(t, 2) + x_coefficients(4) * t + x_coefficients(5);
    double y = y_coefficients(0) * std::pow(t, 5) + y_coefficients(1) * std::pow(t, 4) + y_coefficients(2) * std::pow(t, 3) +
               y_coefficients(3) * std::pow(t, 2) + y_coefficients(4) * t + y_coefficients(5);

    if (t > 0) {
      current_length += std::sqrt(std::pow(x - new_x_list.back(), 2) + std::pow(y - new_y_list.back(), 2));
    }

    if (current_length <= target_length) {
      new_x_list.push_back(x);
      new_y_list.push_back(y);
      new_yaw_list.push_back(std::atan2(y - (new_y_list.empty() ? y0 : new_y_list.back()), x - (new_x_list.empty() ? x0 : new_x_list.back())));
      new_gear_list.push_back(path.gear_list.front());
    } else {
      break;
    }
  }

  // 确保最后一个点被添加
  if (current_length < target_length) {
    new_x_list.push_back(x1);
    new_y_list.push_back(y1);
    new_yaw_list.push_back(path.yaw_list.back());
    new_gear_list.push_back(path.gear_list.back());
  }

  // 更新路径
  path.x_list = new_x_list;
  path.y_list = new_y_list;
  path.yaw_list = new_yaw_list;
  path.gear_list = new_gear_list;
}


// // 3.0按照档位切分路径
// std::vector<CustomPath> ControlFSM::splitPaths(const CustomPath& path) {
//     std::vector<CustomPath> split_paths;
//     CustomPath current_path;
//     int deleted_path_count = 0;
//     // 总的点数
//     total_path_size = path.gear_list.size()-1;
//     for (size_t i = 0; i < path.gear_list.size(); ++i) 
//     {
//       current_path.x_list.push_back(path.x_list[i]);
//       current_path.y_list.push_back(path.y_list[i]);
//       current_path.yaw_list.push_back(path.yaw_list[i]);
//       current_path.gear_list.push_back(path.gear_list[i]);

//       if (i < path.gear_list.size() - 1 && path.gear_list[i] != path.gear_list[i + 1]) 
//       {
//         split_paths.push_back(current_path);
//         current_path = CustomPath();
//       }
//     }
//     // 如果还有没有处理完的：
//     if (!current_path.x_list.empty()) 
//     {
//       split_paths.push_back(current_path);
//     }

//     // total_map_num = split_paths.size();
//     // for (size_t i = 0; i < split_paths.size(); ++i) {
//     //   path_sizes.push_back(split_paths[i].size());
//     // }

//     // TODO split_paths再处理：
//     // 舍弃长度较小的之后重新进行一次档位分割，要求最终的还是按照档位划分好的：
//     // for (size_t i = 0; i < split_paths.size(); ++i) {
//     //   if (split_paths[i].size() <= 5) {
//     //     // 删除小路径
        
//     //     deleted_path_count++;
//     //     deleted_paths.push_back({deleted_path_count, split_paths[i].size()});
//     //   }
//     // }

//     auto new_end = std::remove_if(split_paths.begin(), split_paths.end(), [&](const CustomPath& path) {
//         if (path.size() <= 5) {
//             // 当路径中的点数小于等于5时，记录这条路径的信息
//             deleted_path_count++;
//             deleted_paths.push_back({deleted_path_count, path.size()});
//             return true; // 返回 true 表示这条路径将被移除
//         }
//         return false; // 返回 false 表示保留这条路径
//     });
//     // 移除元素后，使用 erase 方法真正地删除这些元素
//     split_paths.erase(new_end, split_paths.end());

//     // 重新组合所有符合条件的路径点
//     CustomPath combined_path;
//     for (const auto& sp : split_paths) {
//         combined_path.x_list.insert(combined_path.x_list.end(), sp.x_list.begin(), sp.x_list.end());
//         combined_path.y_list.insert(combined_path.y_list.end(), sp.y_list.begin(), sp.y_list.end());
//         combined_path.yaw_list.insert(combined_path.yaw_list.end(), sp.yaw_list.begin(), sp.yaw_list.end());
//         combined_path.gear_list.insert(combined_path.gear_list.end(), sp.gear_list.begin(), sp.gear_list.end());
//     }

//     // 再次按照档位切分
//     std::vector<CustomPath> final_split_paths;
//     current_path = CustomPath();
//     for (size_t i = 0; i < combined_path.gear_list.size(); ++i) {
//         current_path.x_list.push_back(combined_path.x_list[i]);
//         current_path.y_list.push_back(combined_path.y_list[i]);
//         current_path.yaw_list.push_back(combined_path.yaw_list[i]);
//         current_path.gear_list.push_back(combined_path.gear_list[i]);

//         if (i < combined_path.gear_list.size() - 1 && combined_path.gear_list[i]!= combined_path.gear_list[i + 1]) {
//             final_split_paths.push_back(current_path);
//             current_path = CustomPath();
//         }
//     }
//     // 检查最后一个路径片段
//     if (!current_path.x_list.empty()) {
//         final_split_paths.push_back(current_path);
//     }

//     total_map_num = final_split_paths.size();
//     for (size_t i = 0; i < final_split_paths.size(); ++i) {
//         path_sizes.push_back(final_split_paths[i].size());
//     }

//     deleted_count = deleted_path_count;
//     for (const auto& pair : deleted_paths) {
//       std::cout << "Deleted path " << pair.first << " has size " << pair.second << std::endl;
//     }

//     return split_paths;
// }


// // 2.0 按照档位切分路径
std::pair<std::vector<CustomPath>, std::vector<CustomPath>> ControlFSM::splitPaths(const CustomPath& path) {
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
        current_path.gear_list.push_back(path.gear_list[i]);

        if (i < path.gear_list.size() - 1 && path.gear_list[i]!= path.gear_list[i + 1]) {
            if (current_path.size() >= DeletedPathSize) {
                split_paths.push_back(current_path);
            } else {
                std::cout << "Deleted path with size " << current_path.size() << std::endl;
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
        std::cout << "Deleted path with size " << current_path.size() << std::endl;
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
    }

    // 再次按照档位切分
    std::vector<CustomPath> final_split_paths;
    std::vector<CustomPath> interpolated_split_paths;
    
    current_path = CustomPath();
    for (size_t i = 0; i < combined_path.gear_list.size(); ++i) {
        current_path.x_list.push_back(combined_path.x_list[i]);
        current_path.y_list.push_back(combined_path.y_list[i]);
        current_path.yaw_list.push_back(combined_path.yaw_list[i]);
        current_path.gear_list.push_back(combined_path.gear_list[i]);

        if (i < combined_path.gear_list.size() - 1 && combined_path.gear_list[i]!= combined_path.gear_list[i + 1]) {
            final_split_paths.push_back(current_path);
            current_path = CustomPath();

            // 对当前路径段进行插值
            CustomPath interpolated_path = current_path;
            insertPoints(interpolated_path, 6.0); // 调用插值函数
            interpolated_split_paths.push_back(interpolated_path);
        }
    }

    // 检查最后一个路径片段
    if (!current_path.x_list.empty()) {
        final_split_paths.push_back(current_path);

        // 对最后一个路径段进行插值
        CustomPath interpolated_path = current_path;
        insertPoints(interpolated_path, 7.0); // 调用插值函数
        interpolated_split_paths.push_back(interpolated_path);
    }

    total_map_num = final_split_paths.size();
    total_map_interpolated_num = interpolated_split_paths.size();
    for (size_t i = 0; i < final_split_paths.size(); ++i) {
        path_sizes.push_back(final_split_paths[i].size());
    }
    for (size_t i = 0; i < interpolated_split_paths.size(); ++i) {
        interpolated_path_sizes.push_back(interpolated_split_paths[i].size());
    }

    // std::cout << "Total number of deleted paths: " << deleted_path_count << std::endl;
    deleted_count = deleted_path_count;
    // for (const auto& pair : deleted_paths) {
    //     std::cout << "Deleted path " << pair.first << " has size " << pair.second << std::endl;
    // }

    // return final_split_paths;
    // 返回原始路径和插值处理过的路径
    return std::make_pair(final_split_paths, interpolated_split_paths);
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
  return closest_path_index;
}


outWheelAngle ControlFSM::CalControllerRear(int nearest_index, const CustomPath& path)
{
  outWheelAngle output_controller;
  // 计算路径点与车辆的横向误差（使用叉乘计算带符号的误差）
  double cte = 0.0;

  // 获取最近点 P1 (x1, y1) 和下一个点 P2 (x2, y2)
  // double x1 = path.x_list[std::max(0, static_cast<size_t>(nearest_index - 2))];
  // double x1 = path.x_list[std::max(static_cast<size_t>(0), nearest_index > 1 ? nearest_index - 2 : 0)];
  double x1 = path.x_list[std::max(static_cast<size_t>(0), nearest_index > 1 ? static_cast<size_t>(nearest_index - 2) : static_cast<size_t>(0))];

  // double y1 = path.y_list[nearest_index];
  double y1 = path.y_list[std::max(static_cast<size_t>(0), nearest_index > 1 ? static_cast<size_t>(nearest_index - 2) : static_cast<size_t>(0))];
  // double x2 = path.x_list[std::min(path.x_list.size()-1, nearest_index + 1)];
  // double y2 = path.y_list[std::min(path.x_list.size()-1, nearest_index + 1)];

  double x2 = path.x_list[std::min(static_cast<size_t>(path.x_list.size()-1), static_cast<size_t>(nearest_index + 2))];
  double y2 = path.y_list[std::min(static_cast<size_t>(path.x_list.size()-1), static_cast<size_t>(nearest_index + 2))];

  // 当前车辆位置 (current_x_, current_y_)
  double dx = x2 - x1;
  double dy = y2 - y1;

  // 计算叉乘来判断方向
  double cross_product = (rear_current_position.x - x1) * dy - (rear_current_position.y - y1) * dx;

  // 计算垂直距离 (带符号的横向误差)
  double distance = std::sqrt(dx * dx + dy * dy);

  if (distance != 0.0)
  {
    cte = cross_product / distance;
  }
  else cte = 0.0;
  if (nearest_index >= path.x_list.size()-2){
    cte = 0.0;
  }
  // 打印信息，查看误差的正负
  // ROS_INFO_STREAM("\033[1;32m Cross track error (signed): " << cross_track_error);
  // return cross_track_error;

  // double path_yaw = path.yaw_list[nearest_index + pre_index_]; // 预瞄航向误差
  double path_yaw = path.yaw_list[std::min(static_cast<size_t>(path.x_list.size()-1), static_cast<size_t>(nearest_index + pre_index_))]; // 预瞄航向误差
  double dhead = path_yaw - current_yaw_;
  if (dhead > M_PI) dhead -= 2 * M_PI;
  if (dhead < -M_PI) dhead += 2 * M_PI;

  output_controller.cte = cte;
  // output_controller.dhead = dhead;
  // output_controller.gear = path.gear_list[nearest_index];
  // output_controller.wheelangle = steering_angle_rad;
  
  // // 速度规划函数add：
  // output_controller.throttle = GetSpeed(cte, dhead, steering_angle_rad);//GlobalTargetSpeed;//0.5; // 导入速度规划函数

  return output_controller;
}


// 计算控制信息
outWheelAngle ControlFSM::CalControllerFront(int nearest_index, const CustomPath& path)
{
  outWheelAngle output_controller;
  // 计算路径点与车辆的横向误差（使用叉乘计算带符号的误差）
  double cte = 0.0;

  // 获取最近点 P1 (x1, y1) 和下一个点 P2 (x2, y2)
  // double x1 = path.x_list[std::max(0, static_cast<size_t>(nearest_index - 2))];
  // double x1 = path.x_list[std::max(static_cast<size_t>(0), nearest_index > 1 ? nearest_index - 2 : 0)];
  double x1 = path.x_list[std::max(static_cast<size_t>(0), nearest_index > 1 ? static_cast<size_t>(nearest_index - 2) : static_cast<size_t>(0))];

  // double y1 = path.y_list[nearest_index];
  double y1 = path.y_list[std::max(static_cast<size_t>(0), nearest_index > 1 ? static_cast<size_t>(nearest_index - 2) : static_cast<size_t>(0))];
  // double x2 = path.x_list[std::min(path.x_list.size()-1, nearest_index + 1)];
  // double y2 = path.y_list[std::min(path.x_list.size()-1, nearest_index + 1)];

  double x2 = path.x_list[std::min(static_cast<size_t>(path.x_list.size()-1), static_cast<size_t>(nearest_index + 2))];
  double y2 = path.y_list[std::min(static_cast<size_t>(path.x_list.size()-1), static_cast<size_t>(nearest_index + 2))];

  // 当前车辆位置 (current_x_, current_y_)
  double dx = x2 - x1;
  double dy = y2 - y1;

  // 计算叉乘来判断方向
  double cross_product = (dis_current_position.x - x1) * dy - (dis_current_position.y - y1) * dx;

  // 计算垂直距离 (带符号的横向误差)
  double distance = std::sqrt(dx * dx + dy * dy);

  if (distance != 0.0)
  {
    cte = cross_product / distance;
  }
  else cte = 0.0;
  if (nearest_index >= path.x_list.size()-2){
    cte = 0.0;
  }
  // 打印信息，查看误差的正负
  // ROS_INFO_STREAM("\033[1;32m Cross track error (signed): " << cross_track_error);
  // return cross_track_error;

  // double path_yaw = path.yaw_list[nearest_index + pre_index_]; // 预瞄航向误差
  double path_yaw = path.yaw_list[std::min(static_cast<size_t>(path.x_list.size()-1), static_cast<size_t>(nearest_index + pre_index_))]; // 预瞄航向误差
  double dhead = path_yaw - dis_current_position.yaw;
  if (dhead > M_PI) dhead -= 2 * M_PI;
  if (dhead < -M_PI) dhead += 2 * M_PI;
  
  steering_angle_rad = stanleycontroller_->computeSteeringAngle(dhead,cte,current_speed_,controller_location);
  output_controller.cte = cte;
  output_controller.dhead = dhead;
  output_controller.gear = path.gear_list[nearest_index];
  output_controller.wheelangle = steering_angle_rad;
  
  // 速度规划函数add：
  output_controller.throttle = GetSpeed(cte, dhead, steering_angle_rad);//GlobalTargetSpeed;//0.5; // 导入速度规划函数

  return output_controller;
}



// 计算控制信息
outWheelAngle ControlFSM::CalController(int nearest_index, const CustomPath& path)
{
  outWheelAngle output_controller;
  // 计算路径点与车辆的横向误差（使用叉乘计算带符号的误差）
  double cte = 0.0;

  // 获取最近点 P1 (x1, y1) 和下一个点 P2 (x2, y2)
  // double x1 = path.x_list[std::max(0, static_cast<size_t>(nearest_index - 2))];
  // double x1 = path.x_list[std::max(static_cast<size_t>(0), nearest_index > 1 ? nearest_index - 2 : 0)];
  double x1 = path.x_list[std::max(static_cast<size_t>(0), nearest_index > 1 ? static_cast<size_t>(nearest_index - 2) : static_cast<size_t>(0))];

  // double y1 = path.y_list[nearest_index];
  double y1 = path.y_list[std::max(static_cast<size_t>(0), nearest_index > 1 ? static_cast<size_t>(nearest_index - 2) : static_cast<size_t>(0))];
  // double x2 = path.x_list[std::min(path.x_list.size()-1, nearest_index + 1)];
  // double y2 = path.y_list[std::min(path.x_list.size()-1, nearest_index + 1)];

  double x2 = path.x_list[std::min(static_cast<size_t>(path.x_list.size()-1), static_cast<size_t>(nearest_index + 2))];
  double y2 = path.y_list[std::min(static_cast<size_t>(path.x_list.size()-1), static_cast<size_t>(nearest_index + 2))];

  // 当前车辆位置 (current_x_, current_y_)
  double dx = x2 - x1;
  double dy = y2 - y1;

  // 计算叉乘来判断方向
  double cross_product = (current_position.x - x1) * dy - (current_position.y - y1) * dx;

  // 计算垂直距离 (带符号的横向误差)
  double distance = std::sqrt(dx * dx + dy * dy);

  if (distance != 0.0)
  {
    cte = cross_product / distance;
  }
  else cte = 0.0;
  if (nearest_index >= path.x_list.size()-2){
    cte = 0.0;
  }
  // 打印信息，查看误差的正负
  // ROS_INFO_STREAM("\033[1;32m Cross track error (signed): " << cross_track_error);
  // return cross_track_error;

  // double path_yaw = path.yaw_list[nearest_index + pre_index_]; // 预瞄航向误差
  double path_yaw = path.yaw_list[std::min(static_cast<size_t>(path.x_list.size()-1), static_cast<size_t>(nearest_index + pre_index_))]; // 预瞄航向误差
  double dhead = path_yaw - current_position.yaw;
  if (dhead > M_PI) dhead -= 2 * M_PI;
  if (dhead < -M_PI) dhead += 2 * M_PI;

  steering_angle_rad = stanleycontroller_->computeSteeringAngle(dhead, cte, current_speed_);

  output_controller.cte = cte;
  output_controller.dhead = dhead;
  output_controller.gear = path.gear_list[nearest_index];
  output_controller.wheelangle = steering_angle_rad;
  
  // 速度规划函数add：
  output_controller.throttle = GetSpeed(cte, dhead, steering_angle_rad);//GlobalTargetSpeed;//0.5; // 导入速度规划函数

  return output_controller;
}

double ControlFSM::GetSpeed(double cte, double dhead, double wheel_angle_rad) {
    // 定义不同的速度层级
    // const double SPEED_VERY_HIGH = GlobalTargetSpeed + 0.3; // 非常低的速度
    const double SPEED_HIGH = GlobalTargetSpeed; // 高速 1.0
    const double SPEED_MEDIUM = 0.6; // 中速
    const double SPEED_LOW = 0.4; // 低速
    const double SPEED_VERY_LOW = 0.2; // 非常低的速度
    const double SPEED_VERY_VERY_LOW = 0.1;

    // 将dhead从弧度转换为角度
    double dhead_degrees = dhead * (180.0 / M_PI);
    double wheel_angle  = wheel_angle_rad * (180.0 / M_PI);
    // 根据CTE和dhead的大小来确定速度
    // if (cte > 0.5 && dhead_degrees > 30) {
    //     return SPEED_VERY_LOW;
    // } else if (cte < 0.1 && dhead_degrees < 5) {
    //     return SPEED_HIGH;
    // } else if (cte > 0.1 && cte < 0.3 && dhead_degrees > 5 && dhead_degrees < 15) {
    //     return SPEED_MEDIUM;
    // } else if (cte > 0.3 && cte < 0.5 && dhead_degrees > 10 && dhead_degrees < 30) {
    //     return SPEED_LOW;
    // }
    // else if(end_dis<3&&end_dis>=1){
    //   return SPEED_LOW;
    // }
    // else if(end_dis<1){
    //   return 0.2;
    // }
    // 先由终点距离来限速

    // 仿真0.2
    if(sim_mode){
      if (wheel_angle >=35){
        return 0.1;
      }
      else{
        return 0.2; 
      }
    }
    
    if(end_dis<3.0&&end_dis>=0.5){
      return SPEED_LOW;
    }
    else if(end_dis<0.5){
      return SPEED_VERY_LOW;
    }
    // 终点还早则由转角限速
    // else if (wheel_angle < 5&&wheel_angle>=0){
    //   return SPEED_VERY_HIGH;
    // }
    else if (wheel_angle < 10&&wheel_angle>=0){
      return SPEED_HIGH;
    }
    else if (wheel_angle < 25&&wheel_angle>=10){
      return SPEED_MEDIUM;
    }
    else if (wheel_angle < 40&&wheel_angle>=25){
      return SPEED_LOW;
    }
    else if (wheel_angle < 60&&wheel_angle>=50){
      return SPEED_VERY_LOW;
    }
    else if (wheel_angle>=60){
      return SPEED_VERY_VERY_LOW;
    }
    // 默认情况下返回中速
    return SPEED_MEDIUM;
}

// 状态转换
void ControlFSM::transitionState(State newState) 
{
  std::string state_str[7] = { "WAIT_TASK", "GLOBAL_TASK", "EXEC_GLOBAL_PATH","LOCAL_TASK","EXEC_LOCAL_PATH","GLOBAL_STOP","LOCAL_STOP"};
  int pre_s  = int(currentState);
  currentState = newState;
  std::cout << "\033[1;33m From " + state_str[pre_s] + " To " + state_str[int(newState)] << std::endl;
}

void ControlFSM::coutState() {
    std::string state_str[7] = { "WAIT_TASK", "GLOBAL_TASK", "EXEC_GLOBAL_PATH","LOCAL_TASK","EXEC_LOCAL_PATH","GLOBAL_STOP","LOCAL_STOP"};
    int pre_s = int(currentState);
    // currentState         = newState;
    // std::cout << "State is:  " + state_str[int(newState)] << std::endl;
    ROS_INFO_STREAM("\033[1;32m State is: "<<state_str[pre_s]);
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
      output_controller.gear = 0;
      output_controller.wheelangle = 0.0;
      output_controller.throttle = 0.0;
      brake_msg.data = 0.0;
      map_index = 0;
      path_index = 0;
      end_dis = 0;

      traj_len = 0;  // 只有在wait中才会清理 新的订阅path能够重新有值。
      // DEBUG 全局规划器和局部规划器的通信并不全面，现在建立规划期和控制器之间的服务通信：
      
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
      { // 新规则，所有状态都发布对应的消息，几乎不给不发的时候
        // // 下发控制信息
        steering_msg.data = 0.0; // 弧度值
        // std::cout<<"steering_angle_rad:"<<steering_angle_rad*180/M_PI<<std::endl;
        steering_pub_.publish(steering_msg);
        gear_msg.data = 0;
        gear_pub_.publish(gear_msg);
        // 速度规划部分
        throttle_msg.data = 0.0;
        throttle_pub_.publish(throttle_msg);
        brake_msg.data = 0;  // 刹车力度
        brake_pub_.publish(brake_msg);
        ROS_INFO_ONCE("\033[1;31m Wait Smach ");
      }
      break;
    }

    //2.全局任务处理===================================
    // 处理一次路径，反馈控制处于全局控制状态（是否需要一直发送知道完成全局任务！！！）
    case State::GLOBAL_TASK:
    {
      if (control_state!=1)
      {
        transitionState(State::WAIT_TASK);
      }

      if(debug_mode)
      {
        std::cout<<"have_odom:"<<have_odom_<<std::endl;
        std::cout<<"global_path_process:"<<global_path_process<<std::endl;
        std::cout<<"have_global_traj_: "<<have_global_traj_<<std::endl;
        std::cout<<"global_path_.poses.size(): "<<global_path_.poses.size()<<std::endl;
      }

      // 需要路径有长度才能进 global_path_process保证只处理一次??? 处理完都是要跳转的，不需要加一次性的处理
      if (global_path_.poses.size()!=0)
      {
        // map_path = PathProcess(global_path_); // std::vector<CustomPath> map_path

        std::pair<std::vector<CustomPath>, std::vector<CustomPath>> map_path_split = PathProcess(global_path_);
        // 获取原始路径和插值处理过的路径
        map_path = map_path_split.first;
        interpolated_paths = map_path_split.second;

        // 查找最近的路径点
        std::pair<size_t, size_t> closest_index = findClosestPathPoint(map_path, current_position, current_yaw_);
        // 打印 closest_index
        map_index = closest_index.first; //size_t path_index
        path_index = closest_index.second; //size_t path_index

        std::pair<size_t, size_t> interpolate_closest_index = findClosestPathPoint(interpolated_paths, current_position, current_yaw_);
        // 打印 closest_index
        interpolate_map_index = interpolate_closest_index.first; //size_t path_index
        interpolate_path_index = interpolate_closest_index.second; //size_t path_index

        std::cout << "Closest map index: " << closest_index.first << std::endl;
        std::cout << "Closest path index: " << closest_index.second << std::endl;

        // global_path_process = true;
        transitionState(State::EXEC_GLOBAL_PATH);
      }
      break;
    }

    //2.全局任务处理===================================
    case State::EXEC_GLOBAL_PATH:
    {
      if (control_state!=1)
      {
        transitionState(State::WAIT_TASK);
      }

      // NEED
      // 如果当前路径和发送来的路径长度不一致，说明规划任务更新了  也要回到全局处理->也就是说不止听全局状态机的，局部和局部之间也有通信
      // 也就是有新的traj_len之后就可以
      // if (traj_len!=total_path_size)
      // {
      //   transitionState(State::GLOBAL_TASK);
      // }

      Point end_point_ = {map_path[map_index].x_list.back(), map_path[map_index].y_list.back(), map_path[map_index].yaw_list.back()};
      // 计算整条路径的终点
      // Point end_point_ = {map_path[-1].x_list.back(), map_path[-1].y_list.back(), map_path[-1].yaw_list.back()};
      Point last_end_point_ = {map_path.back().x_list.back(), map_path.back().y_list.back(), map_path.back().yaw_list.back()};
      // 整条路终点的距离： 还是使用qianqian轴
      last_end_dis = calculateDistance(dis_current_position, last_end_point_);

      // 使用始终qianqian轴的定位计算局部终点距离
      end_dis = calculateDistance(dis_current_position, end_point_);

      size_t closest_index = findClosestPathPointIndex(map_path[map_index], current_position);
      size_t closest_index_dis = findClosestPathPointIndex(map_path[map_index], dis_current_position);

      path_index = closest_index;
      path_index_dis = closest_index_dis;
      // path_index_rear = closest_index_rear;
      // path_index_interpolate = closest_index_interpolate;
      // path_index_front = closest_index_front;


      // 判断到达终点的距离：
      // 快要到达终点时，使用qianqian轴的定位来计算
      // 未到终点时，使用front_axle和后轴来循迹 终点的停车使用qianqian轴
      // 所以odom就是front_axle 和base_footprint了 
      if(last_end_dis>3) // 不行，再路径比较少的时候还是应该front_front_axle使用（即只有一条路的时候） 另外，前轴特别容易产生震荡
      {
        output_controller = CalController(closest_index, map_path[map_index]);
        controller_location = 1;
      }
      else
      {
        output_controller = CalControllerFront(closest_index_dis, map_path[map_index]);
        controller_location = 2;
      }

      if(abs(last_end_dis-end_dis)>0.1&&last_end_dis>3)
      {
        output_controller = CalController(closest_index, map_path[map_index]);
        controller_location = 1;
      }
      else
      {
        output_controller = CalControllerFront(closest_index_dis, map_path[map_index]);
        controller_location = 2;
      }

      cte_msg.data = output_controller.cte;
      dhead_msg.data = output_controller.dhead;
      cte_pub_.publish(cte_msg);
      dhead_pub_.publish(dhead_msg);

      // 下发控制信息
      steering_msg.data = output_controller.wheelangle; // 弧度值
      // std::cout<<"steering_angle_rad:"<<steering_angle_rad*180/M_PI<<std::endl;
      steering_pub_.publish(steering_msg);
      gear_msg.data = output_controller.gear;
      gear_pub_.publish(gear_msg);
      // 速度规划部分
      throttle_msg.data = output_controller.throttle;
      throttle_pub_.publish(throttle_msg);
      brake_msg.data = 0;  // 刹车力度
      brake_pub_.publish(brake_msg);

      // if (end_dis<0.1 || closest_index == map_path[map_index].x_list.size()-1)
      // 由于现在计算终点是用的dis_odom，也就是在前轴，所以循迹偏差非常容易超过end_dis 所以现在就不要用end_dis了，然后找匹配也得重新计算：
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
              control_state = 0; //BUG 一定要清零，不然通信会重复几次导致出问题
              transitionState(State::WAIT_TASK);
            }
          }
      }
      break;
    }

    case State::LOCAL_TASK:
    {
      if (control_state!=2)
      {
        transitionState(State::WAIT_TASK);
      }

      if(traj_local_len!=0)
      {
        // map_path = PathProcess(local_path_);
        
        std::pair<std::vector<CustomPath>, std::vector<CustomPath>> map_path_split = PathProcess(local_path_);
        // 获取原始路径和插值处理过的路径
        map_path = map_path_split.first;
        // interpolated_paths = map_path_split.second;
        
        // 查找最近的路径点
        std::pair<size_t, size_t> closest_index = findClosestPathPoint(map_path, current_position, current_yaw_);
        // 打印 closest_index
        map_index = closest_index.first;
        path_index = closest_index.second; 
        transitionState(State::EXEC_LOCAL_PATH);
        // local_path_process = true;
      }
      // if (local_path_process)
      // {
        // transitionState(State::EXEC_LOCAL_PATH);
      // }
      break;
    }

    //2.全局任务处理===================================
    // 全局路径已经切分完成，按照档位分成了多个路径，每个路径的x,y,gear，vel,yaw都知道
    case State::EXEC_LOCAL_PATH:
    {
      if (control_state!=2)
      {
        transitionState(State::WAIT_TASK);
      }

      size_t closest_index = findClosestPathPointIndex(map_path[map_index], current_position);
      
      // 根据最近点来计算偏差情况：
      // outWheelAngle output_controller = CalController(closest_index, map_path[map_index]);
      output_controller = CalController(closest_index, map_path[map_index]);

      Point end_point_ = {map_path[map_index].x_list.back(), map_path[map_index].y_list.back(), map_path[map_index].yaw_list.back()};
      
      // 局部应该都是前进档，不分定位情况
      end_dis = calculateDistance(dis_current_position, end_point_);

      cte_msg.data = output_controller.cte;
      dhead_msg.data = output_controller.dhead;
      cte_pub_.publish(cte_msg);
      dhead_pub_.publish(dhead_msg);
      // 下发控制信息
      steering_msg.data = output_controller.wheelangle; // 弧度值
      // std::cout<<"steering_angle_rad:"<<steering_angle_rad*180/M_PI<<std::endl;
      steering_pub_.publish(steering_msg);
      gear_msg.data = output_controller.gear;
      gear_pub_.publish(gear_msg);
      // 速度规划部分
      throttle_msg.data = output_controller.throttle;
      throttle_pub_.publish(throttle_msg);
      brake_msg.data = 0;  // 刹车力度
      brake_pub_.publish(brake_msg);

      if (end_dis<0.1 || closest_index == map_path[map_index].x_list.size()-1)
      {
        transitionState(State::LOCAL_STOP);
      }
      break;
    }

    //2.局部任务结束===================================
    case State::LOCAL_STOP:
    {
      if (control_state!=2)
      {
        transitionState(State::WAIT_TASK);
        local_path_process = false; //任务都需要将路径进行一次性处理
        return;
      }

      // 下发控制信息
      steering_msg.data = 0; // 弧度值
      steering_pub_.publish(steering_msg);
      gear_msg.data = int(Direction::NO);  // N档位
      gear_pub_.publish(gear_msg);
      throttle_msg.data = 0;
      throttle_pub_.publish(throttle_msg);
      brake_msg.data = 4;  // 刹车力度
      brake_pub_.publish(brake_msg);

      if(current_speed_<0.08) //判断是否刹停
      {
        int8_t requestData = 2;
        bool result = callPositionService(requestData);
        transitionState(State::WAIT_TASK);
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
    ROS_INFO("----------------------- Control FSM ----------------------");
    ROS_INFO_STREAM("\033[1;32m Total_map_num: " << total_map_num <<"total_path_size: "<<total_path_size);
    for (size_t i = 0; i < path_sizes.size(); ++i) 
    {
      ROS_INFO_STREAM("\033[1;32m path_size_ "<<i<<":"<< path_sizes[i]);
    }
    
    if(controller_location==1)
    {
      ROS_INFO_STREAM("\033[1;32m Now Controller Location: Front_Axle");
    }
    else if(controller_location==2)
    {
      ROS_INFO_STREAM("\033[1;32m Now Controller Location: Front_Front_Axle");
    }

    if(debug_mode)
    {
      ROS_INFO_STREAM("CTE: " << output_controller.cte);
      ROS_INFO_STREAM("dHead: " << output_controller.dhead);
      ROS_INFO_STREAM("WheelAngle: " << output_controller.wheelangle);
    }
    

    // 打印被删除的路径信息
    ROS_INFO_STREAM("\033[1;32m Total number of deleted paths: " << deleted_count);
    for (const auto& pair : deleted_paths) 
    {
      ROS_INFO_STREAM("\033[1;32m Deleted path "<<pair.first<<" with size: "<< pair.second);
    }
    ROS_INFO_STREAM("\033[1;32m Now map_index: " << map_index<<"now path_index: "<<path_index);
    ROS_INFO_STREAM("\033[1;32m Now path_index_rear: " << path_index_rear<<"now path_index_dis: "<<path_index_dis);

    ROS_INFO_STREAM("\033[1;32m end_dis: " << end_dis);
    ROS_INFO_STREAM("\033[1;32m last_end_dis: " << last_end_dis);

    coutState();
    ROS_INFO_STREAM("Execution time: " << exec_time.toSec() << " seconds");
    ROS_INFO("----------------------- Control FSM ----------------------");
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