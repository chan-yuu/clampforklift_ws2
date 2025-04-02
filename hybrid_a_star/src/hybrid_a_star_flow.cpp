#include "hybrid_a_star/hybrid_a_star_flow.h"

#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <thread>
#include <hybrid_a_star/state_node.h>

// 重要修改日历
// TODO 创建服务接收目标点： ok
// 修改服务，需要传入目标点之外，还要将目标点的个数传入
// 把目标点一个一个存起来？？？？
// 注意这里提供的服务是让点一个一个存下来的，所以具体要存多少个需要通过task来提前告知！！，要么就是还用之前的topic->不够稳定
// 等到目标点个数满足之后才能进行规划？->不对，只有收到了终点的信息之后才能往下即可
// 接收到目标点之后再发送一个状态：
// 服务添加完毕，只需要判读是否获得了终点的：
// TODO 保证rviz 也能够重新进行规划 ok


double Mod2Pi(const double &x)
{
    double v = fmod(x, 2 * M_PI);
    if (v < -M_PI) {
        v += 2.0 * M_PI;
    } else if (v > M_PI) {
        v -= 2.0 * M_PI;
    }
    return v;
}

HybridAStarFlow::HybridAStarFlow(ros::NodeHandle &nh) {
    // 全局参数
    steering_angle = nh.param("global_planner/steering_angle", 10);
    steering_angle_discrete_num = nh.param("global_planner/steering_angle_discrete_num", 1);
    wheel_base = nh.param("global_planner/wheel_base", 1.0); //==steering_discrete_num_ 用于
    segment_length = nh.param("global_planner/segment_length", 1.6);
    segment_length_discrete_num = nh.param("global_planner/segment_length_discrete_num", 8);
    steering_penalty = nh.param("global_planner/steering_penalty", 1.05);
    steering_change_penalty = nh.param("global_planner/steering_change_penalty", 1.5);
    reversing_penalty = nh.param("global_planner/reversing_penalty", 2.0);
    shot_distance = nh.param("global_planner/shot_distance", 5.0);
    angle_penalty=nh.param("global_planner/angle_penalty", 1);
    RSPointDis=nh.param("global_planner/RSPointDis", 0.1);
    //加入车辆的参数
    vehicle_length = nh.param("global_planner/vhicle_length", 2);
    vehicle_width = nh.param("global_planner/vihcle_width", 1);
    vehicle_rear_axle_to_tailstock = nh.param("global_planner/vehicle_rear_axle_to_tailstock", 0.3);

    // 局部参数
    local_steering_angle = nh.param("local_planner/steering_angle", 10);
    local_steering_angle_discrete_num = nh.param("local_planner/steering_angle_discrete_num", 1);
    local_wheel_base = nh.param("local_planner/wheel_base", 1.0); //==steering_discrete_num_ 用于
    local_segment_length = nh.param("local_planner/segment_length", 1.6);
    local_segment_length_discrete_num = nh.param("local_planner/segment_length_discrete_num", 8);
    local_steering_penalty = nh.param("local_planner/steering_penalty", 1.05);
    local_steering_change_penalty = nh.param("local_planner/steering_change_penalty", 1.5);
    local_reversing_penalty = nh.param("local_planner/reversing_penalty", 2.0);
    local_shot_distance = nh.param("local_planner/shot_distance", 5.0);
    local_angle_penalty=nh.param("local_planner/angle_penalty", 1);
    local_vehicle_length = nh.param("local_planner/vhicle_length", 2);
    local_vehicle_width = nh.param("local_planner/vihcle_width", 1);
    local_vehicle_rear_axle_to_tailstock = nh.param("local_planner/vehicle_rear_axle_to_tailstock", 0.3);

    // int grid_size_phi = nh.param("planner/grid_size_phi", 75); 

    debug_mode = nh.param("debug_mode", false);
    use_fsm = nh.param("use_fsm", false);
    init_pose_sub_ptr_ = std::make_shared<InitPoseSubscriber2D>(nh, "/initialpose", 1);
    goal_pose_sub_ptr_ = std::make_shared<GoalPoseSubscriber2D>(nh, "/move_base_simple/goal", 1);

    // 初始化 全局参数的A*  received_path_是点
    kinodynamic_astar_searcher_ptr_ = std::make_shared<HybridAStar>(
        steering_angle, steering_angle_discrete_num, segment_length, segment_length_discrete_num, wheel_base,
        steering_penalty, reversing_penalty, steering_change_penalty,angle_penalty,shot_distance, vehicle_length, vehicle_width, vehicle_rear_axle_to_tailstock,RSPointDis
    );

    // 初始化 局部参数的A*
    local_astar_searcher_ptr_ = std::make_shared<HybridAStar>(
        local_steering_angle, local_steering_angle_discrete_num, local_segment_length, local_segment_length_discrete_num, local_wheel_base,
        local_steering_penalty, local_reversing_penalty, local_steering_change_penalty,local_angle_penalty,local_shot_distance, local_vehicle_length, local_vehicle_width, local_vehicle_rear_axle_to_tailstock,RSPointDis
    );
    

    global_path_pub_ = nh.advertise<nav_msgs::Path>("global_path", 1);
    local_path_pub_ = nh.advertise<nav_msgs::Path>("local_path", 1);
    costmap_sub_ptr_ = std::make_shared<CostMapSubscriber>(nh, "/global_map", 1);
    costmap_local_sub_ptr_ = std::make_shared<CostMapSubscriber>(nh, "/local_map", 1);
    // 棉包位姿
    cotton_sub_ = nh.subscribe("/lidar_camera_det/detections", 1, &HybridAStarFlow::cameraCameracallback, this); 

    searched_tree_pub_ = nh.advertise<visualization_msgs::Marker>("searched_tree", 1);
    vehicle_path_pub_ = nh.advertise<visualization_msgs::MarkerArray>("vehicle_path", 1);
    plan_flag_ = nh.advertise<std_msgs::Int32>("plan_flag", 1);

    // 订阅定位
    // add cyun 10.16 规划的起点都是front_front_axle开始的
    odom_sub_ = nh.subscribe("/dis_odom", 1, &HybridAStarFlow::odometryCallback, this);
    // odom_sub_ = nh.subscribe("/front_middle_odom", 1, &HybridAStarFlow::odometryCallback, this);


    currentState = State::WAIT_TASK;
    // 规划状态机主函数：
    timer_ = nh.createTimer(ros::Duration(0.1), &HybridAStarFlow::timerCallback, this);

    // 服务端添加：
    service_init_pose_ = nh.advertiseService("get_init_pose", &HybridAStarFlow::handle_get_init_pose, this);
    service_tar_pose_ = nh.advertiseService("get_target_pose", &HybridAStarFlow::handle_get_target_pose, this);
    service_plan_task_ = nh.advertiseService("plan_task", &HybridAStarFlow::handle_plan_task, this);
    ROS_INFO("Plan task service ready.");
    ROS_INFO("Init pose service ready.");
    ROS_INFO("Target pose service ready.");

    has_map_ = false;  //进入地图的标志位
    received_camera_ = false;
    have_odom_=false;
    global_path_process = false;
    received_target_pose_ =false;
    received_init_pose_ = false;
    plan_state = 0;
}

bool HybridAStarFlow::handle_plan_task(car_interfaces::PlanTask::Request  &req, car_interfaces::PlanTask::Response &res)
{
    ROS_INFO("\033[1;33m request: plan_state = %d", req.data);
    plan_state = req.data;
    // 设置响应
    res.success = true;  // 设置success为true
    res.message = "Control state updated successfully.";  // 可以附加一些描述信息
  return true;
}

bool HybridAStarFlow::handle_get_init_pose(car_interfaces::GetInitPose::Request  &req,
                                          car_interfaces::GetInitPose::Response &res) {
    ROS_INFO("Received init pose request");
    ROS_INFO("Position: x=%f, y=%f, z=%f", req.init_pose.pose.pose.position.x, req.init_pose.pose.pose.position.y, req.init_pose.pose.pose.position.z);
    ROS_INFO("Orientation: x=%f, y=%f, z=%f, w=%f", req.init_pose.pose.pose.orientation.x, req.init_pose.pose.pose.orientation.y, req.init_pose.pose.pose.orientation.z, req.init_pose.pose.pose.orientation.w);

    // 将起点添加到路径中

    geometry_msgs::PoseStamped start_pose;
    start_pose.header = req.init_pose.header;
    start_pose.pose = req.init_pose.pose.pose;
    received_path_.poses.push_back(start_pose);

    res.success = true;
    res.message = "Init pose received successfully.";
    received_init_pose_ = true;
    return true;
}

bool HybridAStarFlow::handle_get_target_pose(car_interfaces::GetTargetPose::Request  &req,
                                            car_interfaces::GetTargetPose::Response &res) {

    std::stringstream ss;
    // 打印请求的位姿
    ss << "Received target pose:" << std::endl;
    ss << "Header: " << req.tar_pose.header.frame_id << " @ " << req.tar_pose.header.stamp.toSec() << std::endl;
    ss << "Position: x=" << req.tar_pose.pose.position.x << ", y=" << req.tar_pose.pose.position.y << ", z=" << req.tar_pose.pose.position.z << std::endl;
    ss << "Orientation: x=" << req.tar_pose.pose.orientation.x << ", y=" << req.tar_pose.pose.orientation.y << ", z=" << req.tar_pose.pose.orientation.z << ", w=" << req.tar_pose.pose.orientation.w << std::endl;
    ROS_INFO_STREAM(ss.str());

    // 将终点添加到路径中
    geometry_msgs::PoseStamped target_pose;
    target_pose.header = req.tar_pose.header;
    target_pose.pose = req.tar_pose.pose;
    received_path_.poses.push_back(target_pose);

    received_target_pose_ = true;  // 标记接收到了终点
    
    res.success = true;
    res.message = "Target pose received successfully.";
    return true;
}

// 状态转换
void HybridAStarFlow::transitionState(State newState) {
    std::string state_str[3] = { "WAIT_TASK", "GLOBAL_TASK", "LOCAL_TASK"};
    int    pre_s        = int(currentState);
    currentState        = newState;
    //   std::cout << "From " + state_str[pre_s] + " To " + state_str[int(newState)] << std::endl;
    ROS_INFO_STREAM("\033[1;33m From: "<<state_str[pre_s]<<" To: "<<state_str[int(newState)]);
}

void HybridAStarFlow::coutState() {
    std::string state_str[3] = { "WAIT_TASK", "GLOBAL_TASK", "LOCAL_TASK"};
    int pre_s = int(currentState);
    // currentState         = newState;
    // std::cout << "State is:  " + state_str[int(newState)] << std::endl;
    ROS_INFO_STREAM("\033[1;32m State is: "<<state_str[pre_s]);
}

void HybridAStarFlow::publishEmptyPath(ros::Publisher& path_pub) {
  nav_msgs::Path path;
  path.header.frame_id = "map";  // 设置路径的参考坐标系
  path.header.stamp = ros::Time::now();  // 设置时间戳
  // 发布空路径
  path_pub.publish(path);
}

// 状态机调用 0.01s 
void HybridAStarFlow::timerCallback(const ros::TimerEvent&) 
{
    std_msgs::Int32 plan_flag;
    plan_flag.data = 1; 
    plan_flag_.publish(plan_flag);
    ros::Time start_time = ros::Time::now();
    static int count = 0; // 定义一个静态计数器
    const int print_interval = 50; // 设置打印间隔

    if (use_fsm)
    {
        switch (currentState)
        {
            case State::WAIT_TASK:
            {
                // 重新初始化：
                path.clear();
                path_tree.clear();

                //处理一次地图==============
                costmap_sub_ptr_->ParseData(costmap_deque_);
                if (!has_map_)
                {
                    if (costmap_deque_.empty()) 
                    {
                        return;
                    }
                    current_costmap_ptr_ = costmap_deque_.front();

                    costmap_deque_.pop_front();

                    const double map_resolution = 0.2;
                    kinodynamic_astar_searcher_ptr_->Init(
                            current_costmap_ptr_->info.origin.position.x,
                            1.0 * current_costmap_ptr_->info.width * current_costmap_ptr_->info.resolution,
                            current_costmap_ptr_->info.origin.position.y,
                            1.0 * current_costmap_ptr_->info.height * current_costmap_ptr_->info.resolution,
                            current_costmap_ptr_->info.resolution,
                            map_resolution
                    );

                    // 修改：
                    unsigned int map_w = std::floor(current_costmap_ptr_->info.width *current_costmap_ptr_->info.resolution / map_resolution);  
                    unsigned int map_h = std::floor(current_costmap_ptr_->info.height *current_costmap_ptr_->info.resolution/ map_resolution);
                    if(debug_mode){
                        std::cout<<"map_w"<<map_w<<std::endl;
                        std::cout<<"map_h"<<map_h<<std::endl;
                    }

                    for (unsigned int w = 0; w < map_w; ++w) 
                    {
                        for (unsigned int h = 0; h < map_h; ++h) 
                        {
                            auto x = static_cast<unsigned int> ((w + 0.5) * map_resolution
                                                                / current_costmap_ptr_->info.resolution);
                            auto y = static_cast<unsigned int> ((h + 0.5) * map_resolution
                                                                / current_costmap_ptr_->info.resolution);
                            if (current_costmap_ptr_->data[y * current_costmap_ptr_->info.width + x]) 
                            {
                                kinodynamic_astar_searcher_ptr_->SetObstacle(w, h);
                            }
                        }
                    }
                    has_map_ = true;
                }
                costmap_deque_.clear();

                // global_path_process = false;
                // has_map_ = false;
                // received_target_pose_ = false; // 结束任务后，终点可以继续请求

                // 0.自检部分，需要一直都有定位信息：
                if (!have_odom_)
                {
                    ROS_INFO_ONCE("\033[1;31m Wait Odom ");
                    return;
                }
                else
                {
                    ROS_INFO_ONCE("\033[1;32m Odom Get");
                }
                // 全局任务
                if (plan_state== static_cast<int>(State::GLOBAL_TASK))
                {
                    transitionState(State::GLOBAL_TASK);
                }
                // 局部规划任务
                else if(plan_state== static_cast<int>(State::LOCAL_TASK))
                {
                    transitionState(State::LOCAL_TASK);
                }
                else{
                    ROS_INFO_ONCE("\033[1;31m Wait Smach ");
                    publishEmptyPath(global_path_pub_);
                }

                break;
            }

            case State::GLOBAL_TASK:
            {
                // std::cout<<"GLOBAL_TASK"<<received_target_pose_<<":"<<has_map_<<std::endl;
                // 最开始就中断
                if (plan_state !=  static_cast<int>(State::GLOBAL_TASK))
                {
                    kinodynamic_astar_searcher_ptr_->Reset();
                    transitionState(State::WAIT_TASK);
                }

                // 接收点不再需要起点，起点将从定位的odom中直接读取一个出来，然后如果有途经点则会继续往里面添加
                if (received_target_pose_)
                {
                    auto start_total = std::chrono::high_resolution_clock::now();  // 记录整个处理过程的结束时间
                    // 将当前位置添加到路径的最前面
                    geometry_msgs::PoseStamped current_pose;
                    current_pose.header.stamp = ros::Time::now();
                    current_pose.header.frame_id = "map";
                    current_pose.pose.position.x = current_position.x;
                    current_pose.pose.position.y = current_position.y;
                    current_pose.pose.position.z = 0.0;
                    current_pose.pose.orientation = tf::createQuaternionMsgFromYaw(current_position.yaw);
                    received_path_.poses.insert(received_path_.poses.begin(), current_pose);

                    path.clear();
                    // path_tree.clear();
                    size_t path_length = received_path_.poses.size();

                    ROS_INFO_STREAM("\033[1;32m Get Global Plan Points Nums: "<<path_length);
                    for (size_t i = 0; i < path_length - 1; ++i) 
                    {
                        auto start_total1 = std::chrono::high_resolution_clock::now();  // 记录整个处理过程的结束时间
                        ROS_INFO_STREAM("\033[1;32m Hybrid A Star Start: "<<i);
                        const geometry_msgs::PoseStamped& start_pose = received_path_.poses[i];
                        const geometry_msgs::PoseStamped& goal_pose = received_path_.poses[i + 1];
                        double start_yaw = tf::getYaw(start_pose.pose.orientation);
                        double goal_yaw = tf::getYaw(goal_pose.pose.orientation);
                        const geometry_msgs::Point& start_position = start_pose.pose.position;
                        const geometry_msgs::Point& goal_position = goal_pose.pose.position;

                        Vec3d start_state = Vec3d(start_position.x, start_position.y, start_yaw);
                        Vec3d goal_state = Vec3d(goal_position.x, goal_position.y, goal_yaw);

                        // 打印起点信息
                        ROS_INFO_STREAM("\033[1;33m Start Pose [" << i << "]: ("
                                        << start_position.x << ", " << start_position.y << ", " << start_yaw << ") rad");

                        // 打印终点信息
                        ROS_INFO_STREAM("\033[1;33m Goal Pose [" << i << "]: ("
                                        << goal_position.x << ", " << goal_position.y << ", " << goal_yaw << ") rad");

                        if (kinodynamic_astar_searcher_ptr_->Search(start_state, goal_state)) 
                        {
                            VectorVec3d current_path = kinodynamic_astar_searcher_ptr_->GetPath();
                            // VectorVec4d current_path_tree = kinodynamic_astar_searcher_ptr_->GetSearchedTree();
                            
                            // path_tree.insert(path_tree.end(), current_path_tree.begin(), current_path_tree.end());
                            path.insert(path.end(), current_path.begin(), current_path.end());
                        }
                        auto end_total = std::chrono::high_resolution_clock::now();  // 记录整个处理过程的结束时间
                        std::chrono::duration<double> elapsed_total = end_total - start_total1;
                        ROS_INFO_STREAM("\033[1;34m Total time taken for processing: " << elapsed_total.count() << " s");
                    }
                    received_target_pose_ = false;
                    // 清理path
                    received_path_.poses.clear();
                }

                // if (path.size()>1&&path_tree.size()>1)
                if (path.size()>1)
                {
                    nav_msgs::Path nav_path;

                    geometry_msgs::PoseStamped pose_stamped;
                    for (const auto &pose: path)
                    {
                        pose_stamped.header.frame_id = "map";
                        pose_stamped.pose.position.x = pose.x();
                        pose_stamped.pose.position.y = pose.y();

                        pose_stamped.pose.position.z = 0.0;
                        pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose.z());
                        // 打印Yaw角度（回转四元数到Yaw）
                        tf::Quaternion quaternion;
                        tf::quaternionMsgToTF(pose_stamped.pose.orientation, quaternion);
                        double roll, pitch, yaw;
                        tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw); // 提取roll, pitch, yaw
                        nav_path.poses.emplace_back(pose_stamped);
                    }

                    nav_path.header.frame_id = "map";
                    nav_path.header.stamp = ros::Time::now();

                    global_path_pub_.publish(nav_path);
                    // global_path_process = true;
                }
                else
                {
                    path.clear();
                    // path_tree.clear();
                    publishEmptyPath(global_path_pub_);
                    // ROS_INFO("\033[1;32m <----------------search failed---------------> 033[0m\n");
                }
            break;
            }

            case State::LOCAL_TASK:
            {
                // 最开始就中断
                // 判断现在应该进行什么任务
                if (plan_state !=  static_cast<int>(State::LOCAL_TASK))
                {
                    transitionState(State::WAIT_TASK);
                    has_map_ = false;
                    local_path_process = false;
                }

                // 没有棉包位置或者定位信息
                if (!received_camera_ || !have_odom_) 
                //没人重置false 进一次相机callback 就一直true 即使相机寄了
                {
                    ROS_INFO_ONCE("\033[1;32m wait location or camera");
                    publishEmptyPath(local_path_pub_);
                }

                else
                {
                    // 处理一次地图========================
                    costmap_local_sub_ptr_->ParseData(costmap_deque_);
                    // ReadData();
                    if (!has_map_)
                    {
                        if (costmap_deque_.empty()) 
                        {
                            return;
                        }
                        current_costmap_ptr_ = costmap_deque_.front();
                        costmap_deque_.pop_front();

                        const double map_resolution = 0.2;
                        local_astar_searcher_ptr_->Init(
                                current_costmap_ptr_->info.origin.position.x,
                                1.0 * current_costmap_ptr_->info.width * current_costmap_ptr_->info.resolution,
                                current_costmap_ptr_->info.origin.position.y,
                                1.0 * current_costmap_ptr_->info.height * current_costmap_ptr_->info.resolution,
                                current_costmap_ptr_->info.resolution,
                                map_resolution
                        );

                        // 修改：
                        unsigned int map_w = std::floor(current_costmap_ptr_->info.width *current_costmap_ptr_->info.resolution / map_resolution);  
                        unsigned int map_h = std::floor(current_costmap_ptr_->info.height *current_costmap_ptr_->info.resolution/ map_resolution);

                        for (unsigned int w = 0; w < map_w; ++w) 
                        {
                            for (unsigned int h = 0; h < map_h; ++h) 
                            {
                                auto x = static_cast<unsigned int> ((w + 0.5) * map_resolution
                                                                    / current_costmap_ptr_->info.resolution);
                                auto y = static_cast<unsigned int> ((h + 0.5) * map_resolution
                                                                    / current_costmap_ptr_->info.resolution);
                                if (current_costmap_ptr_->data[y * current_costmap_ptr_->info.width + x]) 
                                {
                                    local_astar_searcher_ptr_->SetObstacle(w, h);
                                }
                            }
                        }
                        has_map_ = true;
                    }
                    costmap_deque_.clear();

                    double pallet_heading = Detection_yolo.objects[0].yaw;
                    double pallet_x = Detection_yolo.objects[0].x;
                    double pallet_y = Detection_yolo.objects[0].y;
                    ROS_INFO_STREAM("\033[1;32m Cotton: "<<pallet_x<<"\t"<<pallet_y<<"\t"<<Detection_yolo.objects[0].yaw*180/M_PI);
                    ROS_INFO_STREAM("\033[1;32m vehicle: "<<current_position.x<<"\t"<<current_position.y<<"\t"<<current_position.yaw*180/M_PI);
                    
                    // 记录此时的定位点，每次都作为起点：
                    if(!local_path_process){
                        local_x = current_position.x;
                        local_y = current_position.y;
                        local_yaw = current_position.yaw;
                        local_path_process = true;
                    }

                    Vec3d start_state = Vec3d(local_x, local_y, local_yaw);
                    Vec3d goal_state = Vec3d(pallet_x, pallet_y, pallet_heading);

                    // find path:
                    if (local_astar_searcher_ptr_->Search(start_state, goal_state)) 
                    {
                        ROS_INFO("\033[1;32m ------------------------- 033[0m\n");
                        VectorVec3d current_path = local_astar_searcher_ptr_->GetPath();
                        VectorVec4d current_path_tree = local_astar_searcher_ptr_->GetSearchedTree();
                        path_tree.insert(path_tree.end(), current_path_tree.begin(), current_path_tree.end());
                        path.insert(path.end(), current_path.begin(), current_path.end());
                        local_astar_searcher_ptr_->Reset();
                        if(path.size()>1 && path_tree.size()>1)
                        {
                            nav_msgs::Path nav_path;

                            geometry_msgs::PoseStamped pose_stamped;
                            for (const auto &pose: path)
                            {
                                pose_stamped.header.frame_id = "map";
                                pose_stamped.pose.position.x = pose.x();
                                pose_stamped.pose.position.y = pose.y();

                                pose_stamped.pose.position.z = 0.0;
                                pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose.z());
                                // 打印Yaw角度（回转四元数到Yaw）
                                tf::Quaternion quaternion;
                                tf::quaternionMsgToTF(pose_stamped.pose.orientation, quaternion);
                                double roll, pitch, yaw;
                                tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw); // 提取roll, pitch, yaw
                                nav_path.poses.emplace_back(pose_stamped);
                            }

                            nav_path.header.frame_id = "map";
                            nav_path.header.stamp = timestamp_;

                            local_path_pub_.publish(nav_path);
                        }
                        else
                        {
                            path.clear();
                            path_tree.clear();
                            ROS_INFO("\033[1;32m <----------------search failed---------------> 033[0m\n");
                        }
                    }
                    else
                    {
                        std::cout<<"not plan & pass"<<std::endl;
                        path.clear();
                        path_tree.clear();
                        publishEmptyPath(local_path_pub_);
                    }
                }
                break;
            }
        } // switch
        ros::Time end_time = ros::Time::now();
        ros::Duration exec_time = end_time - start_time;  // 计算执行时间

        // // 定次数打印，防止刷屏
        // count++;
        // if (count >= print_interval)
        {
            coutState();
            ROS_INFO_STREAM("Execution time: " << exec_time.toSec() << " seconds");
            ROS_INFO_STREAM("------------------");
            count = 0; // 重置计数器
        }
    } // if use_fsm
    else
    {
        //处理一次地图==============
        costmap_sub_ptr_->ParseData(costmap_deque_);
        if (!has_map_)
        {
            if (costmap_deque_.empty()) 
            {
                return;
            }
            current_costmap_ptr_ = costmap_deque_.front();

            costmap_deque_.pop_front();

            const double map_resolution = 0.2;
            kinodynamic_astar_searcher_ptr_->Init(
                    current_costmap_ptr_->info.origin.position.x,
                    1.0 * current_costmap_ptr_->info.width * current_costmap_ptr_->info.resolution,
                    current_costmap_ptr_->info.origin.position.y,
                    1.0 * current_costmap_ptr_->info.height * current_costmap_ptr_->info.resolution,
                    current_costmap_ptr_->info.resolution,
                    map_resolution
            );

            // 修改：
            unsigned int map_w = std::floor(current_costmap_ptr_->info.width *current_costmap_ptr_->info.resolution / map_resolution);  
            unsigned int map_h = std::floor(current_costmap_ptr_->info.height *current_costmap_ptr_->info.resolution/ map_resolution);
            if(debug_mode){
                std::cout<<"map_w"<<map_w<<std::endl;
                std::cout<<"map_h"<<map_h<<std::endl;
            }

            for (unsigned int w = 0; w < map_w; ++w) 
            {
                for (unsigned int h = 0; h < map_h; ++h) 
                {
                    auto x = static_cast<unsigned int> ((w + 0.5) * map_resolution
                                                        / current_costmap_ptr_->info.resolution);
                    auto y = static_cast<unsigned int> ((h + 0.5) * map_resolution
                                                        / current_costmap_ptr_->info.resolution);
                    if (current_costmap_ptr_->data[y * current_costmap_ptr_->info.width + x]) 
                    {
                        kinodynamic_astar_searcher_ptr_->SetObstacle(w, h);
                    }
                }
            }
            has_map_ = true;
        }
        costmap_deque_.clear();

        // 接收点不再需要起点，起点将从定位的odom中直接读取一个出来，然后如果有途经点则会继续往里面添加
        if (received_target_pose_)
        {
            // 将当前位置添加到路径的最前面
            geometry_msgs::PoseStamped current_pose;
            current_pose.header.stamp = ros::Time::now();
            current_pose.header.frame_id = "map";
            current_pose.pose.position.x = current_position.x;
            current_pose.pose.position.y = current_position.y;
            current_pose.pose.position.z = 0.0;
            current_pose.pose.orientation = tf::createQuaternionMsgFromYaw(current_position.yaw);
            received_path_.poses.insert(received_path_.poses.begin(), current_pose);

            path.clear();
            path_tree.clear();
            size_t path_length = received_path_.poses.size();

            ROS_INFO_STREAM("\033[1;32m Get Global Plan Points Nums: "<<path_length);
            for (size_t i = 0; i < path_length - 1; ++i) 
            {
                ROS_INFO_STREAM("\033[1;32m Hybrid A Star Start: "<<i);
                const geometry_msgs::PoseStamped& start_pose = received_path_.poses[i];
                const geometry_msgs::PoseStamped& goal_pose = received_path_.poses[i + 1];
                double start_yaw = tf::getYaw(start_pose.pose.orientation);
                double goal_yaw = tf::getYaw(goal_pose.pose.orientation);
                const geometry_msgs::Point& start_position = start_pose.pose.position;
                const geometry_msgs::Point& goal_position = goal_pose.pose.position;

                Vec3d start_state = Vec3d(start_position.x, start_position.y, start_yaw);
                Vec3d goal_state = Vec3d(goal_position.x, goal_position.y, goal_yaw);

                // 打印起点信息
                ROS_INFO_STREAM("\033[1;33m Start Pose [" << i << "]: ("
                                << start_position.x << ", " << start_position.y << ", " << start_yaw << ") rad");

                // 打印终点信息
                ROS_INFO_STREAM("\033[1;33m Goal Pose [" << i << "]: ("
                                << goal_position.x << ", " << goal_position.y << ", " << goal_yaw << ") rad");

                if (kinodynamic_astar_searcher_ptr_->Search(start_state, goal_state)) 
                {
                    VectorVec3d current_path = kinodynamic_astar_searcher_ptr_->GetPath();
                    VectorVec4d current_path_tree = kinodynamic_astar_searcher_ptr_->GetSearchedTree();
                    
                    path_tree.insert(path_tree.end(), current_path_tree.begin(), current_path_tree.end());
                    path.insert(path.end(), current_path.begin(), current_path.end());
                }
                kinodynamic_astar_searcher_ptr_->Reset();
            }
            received_target_pose_ = false;
            // 清理path
            received_path_.poses.clear();
        }

        if (path.size()>1&&path_tree.size()>1)
        {
            nav_msgs::Path nav_path;

            geometry_msgs::PoseStamped pose_stamped;
            for (const auto &pose: path)
            {
                pose_stamped.header.frame_id = "map";
                pose_stamped.pose.position.x = pose.x();
                pose_stamped.pose.position.y = pose.y();

                pose_stamped.pose.position.z = 0.0;
                pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose.z());
                // 打印Yaw角度（回转四元数到Yaw）
                tf::Quaternion quaternion;
                tf::quaternionMsgToTF(pose_stamped.pose.orientation, quaternion);
                double roll, pitch, yaw;
                tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw); // 提取roll, pitch, yaw
                nav_path.poses.emplace_back(pose_stamped);
            }

            nav_path.header.frame_id = "map";
            nav_path.header.stamp = ros::Time::now();

            global_path_pub_.publish(nav_path);
            // global_path_process = true;
        }
        else
        {
            path.clear();
            path_tree.clear();
            publishEmptyPath(global_path_pub_);
            // ROS_INFO("\033[1;32m <----------------search failed---------------> 033[0m\n");
        }
        ros::Time end_time = ros::Time::now();
        ros::Duration exec_time = end_time - start_time;  // 计算执行时间
        // 定次数打印，防止刷屏
        count++;
        // if (count >= print_interval)
        // {
        //     ROS_INFO_STREAM("Execution time: " << exec_time.toSec() << " seconds");
        //     ROS_INFO_STREAM("------------------");
        //     count = 0; // 重置计数器
        // }
    }
}

void HybridAStarFlow::cameraCameracallback(const car_interfaces::TargetDetectionInterface::ConstPtr& msg) 
{
    Detection_yolo = *msg;
    received_camera_ = true;
}

// 定位回调
void HybridAStarFlow::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) 
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    // 从四元数提取航向角
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    // current_speed_ = msg->twist.twist.linear.x;
    have_odom_ = true;

    current_position.x = msg->pose.pose.position.x;
    current_position.y = msg->pose.pose.position.y;
    current_position.yaw = yaw;

    
    // 将终点添加到路径中
    // geometry_msgs::PoseStamped pose;
    // pose.header.stamp = ros::Time::now();
    // pose.header.frame_id = "map"; 
    // pose.pose.position.x = current_position.x;
    // pose.pose.position.y = current_position.y;
    // pose.pose.position.z = 0;
    // pose.pose.orientation = tf::createQuaternionMsgFromYaw(current_position.yaw);
    // // 将终点添加到路径中
    // received_path_.poses.push_back(pose);
}


void HybridAStarFlow::Run() {
    ReadData();
    if (!has_map_) {
        if (costmap_deque_.empty()) {
            return;
        }

        current_costmap_ptr_ = costmap_deque_.front();
        costmap_deque_.pop_front();

        const double map_resolution = 0.2;
        kinodynamic_astar_searcher_ptr_->Init(
                current_costmap_ptr_->info.origin.position.x,
                1.0 * current_costmap_ptr_->info.width * current_costmap_ptr_->info.resolution,
                current_costmap_ptr_->info.origin.position.y,
                1.0 * current_costmap_ptr_->info.height * current_costmap_ptr_->info.resolution,
                current_costmap_ptr_->info.resolution,
                map_resolution
        );

        // unsigned int map_w = std::floor(current_costmap_ptr_->info.width / map_resolution);
        // unsigned int map_h = std::floor(current_costmap_ptr_->info.height / map_resolution);
        // 修改：
        unsigned int map_w = std::floor(current_costmap_ptr_->info.width *current_costmap_ptr_->info.resolution / map_resolution);  
        unsigned int map_h = std::floor(current_costmap_ptr_->info.height *current_costmap_ptr_->info.resolution/ map_resolution);

        for (unsigned int w = 0; w < map_w; ++w) {
            for (unsigned int h = 0; h < map_h; ++h) {
                auto x = static_cast<unsigned int> ((w + 0.5) * map_resolution
                                                    / current_costmap_ptr_->info.resolution);
                auto y = static_cast<unsigned int> ((h + 0.5) * map_resolution
                                                    / current_costmap_ptr_->info.resolution);

                if (current_costmap_ptr_->data[y * current_costmap_ptr_->info.width + x]) {
                    kinodynamic_astar_searcher_ptr_->SetObstacle(w, h);
                }
            }
        }
        has_map_ = true;
    }
    costmap_deque_.clear();
}

void HybridAStarFlow::ReadData() {
    costmap_sub_ptr_->ParseData(costmap_deque_);
    // init_pose_sub_ptr_->ParseData(init_pose_deque_);
    // goal_pose_sub_ptr_->ParseData(goal_pose_deque_);
}


void HybridAStarFlow::InitPoseData() {
    current_init_pose_ptr_ = init_pose_deque_.front();
    init_pose_deque_.pop_front();

    current_goal_pose_ptr_ = goal_pose_deque_.front();
    goal_pose_deque_.pop_front();
}


bool HybridAStarFlow::HasGoalPose() {
    return !goal_pose_deque_.empty();
}


bool HybridAStarFlow::HasStartPose() {
    return !init_pose_deque_.empty();
}

double calcHeading(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return atan2(dy, dx);
}


void HybridAStarFlow::PublishPath(const VectorVec3d &path) {
    nav_msgs::Path nav_path;

    geometry_msgs::PoseStamped pose_stamped;
    for (const auto &pose: path)
    {
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();

        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose.z());
        // 打印Yaw角度（回转四元数到Yaw）
        tf::Quaternion quaternion;
        tf::quaternionMsgToTF(pose_stamped.pose.orientation, quaternion);
        double roll, pitch, yaw;
        tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw); // 提取roll, pitch, yaw
        nav_path.poses.emplace_back(pose_stamped);
    }

    nav_path.header.frame_id = "map";
    nav_path.header.stamp = timestamp_;

    path_pub_.publish(nav_path);
}


void HybridAStarFlow::PublishVehiclePath(const VectorVec3d &path, double width,
                                         double length, unsigned int vehicle_interval = 5u) {
    visualization_msgs::MarkerArray vehicle_array;

    for (unsigned int i = 0; i < path.size(); i += vehicle_interval) {
        visualization_msgs::Marker vehicle;

        if (i == 0) {
            vehicle.action = 3;
        }

        vehicle.header.frame_id = "map";
        vehicle.header.stamp = ros::Time::now();
        vehicle.type = visualization_msgs::Marker::CUBE;
        vehicle.id = static_cast<int>(i / vehicle_interval);
        vehicle.scale.x = width;
        vehicle.scale.y = length;
        vehicle.scale.z = 0.01;
        vehicle.color.a = 0.1;

        vehicle.color.r = 1.0;
        vehicle.color.b = 0.0;
        vehicle.color.g = 0.0;

        vehicle.pose.position.x = path[i].x();
        vehicle.pose.position.y = path[i].y();
        vehicle.pose.position.z = 0.0;

        vehicle.pose.orientation = tf::createQuaternionMsgFromYaw(path[i].z());
        vehicle_array.markers.emplace_back(vehicle);
    }

    vehicle_path_pub_.publish(vehicle_array);
}

void HybridAStarFlow::PublishSearchedTree(const VectorVec4d &searched_tree) {
    visualization_msgs::Marker tree_list;
    tree_list.header.frame_id = "map";
    tree_list.header.stamp = ros::Time::now();
    tree_list.type = visualization_msgs::Marker::LINE_LIST;
    tree_list.action = visualization_msgs::Marker::ADD;
    tree_list.ns = "searched_tree";
    tree_list.scale.x = 0.02;

    tree_list.color.a = 1.0;
    tree_list.color.r = 0;
    tree_list.color.g = 0;
    tree_list.color.b = 0;

    tree_list.pose.orientation.w = 1.0;
    tree_list.pose.orientation.x = 0.0;
    tree_list.pose.orientation.y = 0.0;
    tree_list.pose.orientation.z = 0.0;

    geometry_msgs::Point point;
    for (const auto &i: searched_tree) {
        point.x = i.x();
        point.y = i.y();
        point.z = 0.0;
        tree_list.points.emplace_back(point);

        point.x = i.z();
        point.y = i.w();
        point.z = 0.0;
        tree_list.points.emplace_back(point);
    }

    searched_tree_pub_.publish(tree_list);
}
