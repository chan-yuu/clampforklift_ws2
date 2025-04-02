#ifndef HYBRID_A_STAR_HYBRID_A_STAR_FLOW_H
#define HYBRID_A_STAR_HYBRID_A_STAR_FLOW_H

#include "hybrid_a_star.h"
#include "costmap_subscriber.h"
#include "init_pose_subscriber.h"
#include "goal_pose_subscriber.h"
#include <nav_msgs/Path.h>
#include "PathSubscriber.h" // 确保包含新创建的订阅器类的头文件
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <mutex>
#include <std_msgs/Int8.h>
#include <car_interfaces/TargetDetectionInterface.h>

#include <nav_msgs/Odometry.h>
#include "car_interfaces/GetTargetPose.h"
#include "car_interfaces/GetInitPose.h"
#include "car_interfaces/PlanTask.h"
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>  // 包含tf库的头文件
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>


// 定位点的结构体
struct Point {
  double x;
  double y;
  double yaw;
};


class HybridAStarFlow {
public:
    HybridAStarFlow() = default;

    explicit HybridAStarFlow(ros::NodeHandle &nh);

    void Run();

    void PublishPathAndTransform();


private:
    void InitPoseData();

    // void pathCallback(const car_interfaces::PathGlobal::ConstPtr& msg); // 订阅路径

    void ReadData();

    bool HasStartPose();

    bool HasGoalPose();

    void PublishPath(const VectorVec3d &path);

    void PublishSearchedTree(const VectorVec4d &searched_tree);

    void PublishVehiclePath(const VectorVec3d &path, double width,
                            double length, unsigned int vehicle_interval);
  
private:
    std::shared_ptr<HybridAStar> kinodynamic_astar_searcher_ptr_;
    std::shared_ptr<HybridAStar> local_astar_searcher_ptr_;
    std::shared_ptr<CostMapSubscriber> costmap_sub_ptr_;
    std::shared_ptr<CostMapSubscriber> costmap_local_sub_ptr_;

    std::shared_ptr<InitPoseSubscriber2D> init_pose_sub_ptr_;
    std::shared_ptr<GoalPoseSubscriber2D> goal_pose_sub_ptr_;
    // std::shared_ptr<PathSubscriber> path_sub_ptr_;
    
    ros::Subscriber location_sub_;
    // hybrid_a_star::GpsImuInterface EgoContent;

    // void GpsCallback(const hybrid_a_star::GpsImuInterface::ConstPtr& msg); // 根据实际消息类型进行更改

    ros::Publisher path_pub_;
    ros::Publisher searched_tree_pub_;
    ros::Publisher vehicle_path_pub_;
    ros::Publisher path_add_gear_;

    // 服务添加内容：
    ros::ServiceServer service_init_pose_, service_tar_pose_;
    bool received_target_pose_, received_init_pose_;  // 新增标志位

    ros::ServiceServer service_plan_task_;
    bool handle_plan_task(car_interfaces::PlanTask::Request  &req, car_interfaces::PlanTask::Response &res);

    bool handle_get_init_pose(car_interfaces::GetInitPose::Request  &req,
                            car_interfaces::GetInitPose::Response &res);

    bool handle_get_target_pose(car_interfaces::GetTargetPose::Request  &req,
                             car_interfaces::GetTargetPose::Response &res);


    std::deque<geometry_msgs::PoseWithCovarianceStampedPtr> init_pose_deque_;
    std::deque<geometry_msgs::PoseStampedPtr> goal_pose_deque_;
    std::deque<nav_msgs::OccupancyGridPtr> costmap_deque_;

    geometry_msgs::PoseWithCovarianceStampedPtr current_init_pose_ptr_;
    geometry_msgs::PoseStampedPtr current_goal_pose_ptr_;
    nav_msgs::OccupancyGridPtr current_costmap_ptr_;

    ros::Time timestamp_;

    ros::Subscriber from_rivz_points_sub_; // 订阅者

    nav_msgs::Path received_path_;
    // car_interfaces::PathGlobal global_path;
    bool goal_received;

    VectorVec3d path;
    VectorVec4d path_tree;

    bool has_map_{};
    ros::Publisher plan_flag_;

    double steering_angle;
    int steering_angle_discrete_num;
    double wheel_base;
    double segment_length;
    int segment_length_discrete_num;
    double steering_penalty;
    double steering_change_penalty;
    double angle_penalty;
    double reversing_penalty;
    double shot_distance;
    double vehicle_length;
    double vehicle_width;
    double vehicle_rear_axle_to_tailstock;
    double RSPointDis;

    //local param
    double local_steering_angle;
    int local_steering_angle_discrete_num;
    double local_wheel_base;
    double local_segment_length;
    int local_segment_length_discrete_num;
    double local_steering_penalty;
    double local_steering_change_penalty;
    double local_angle_penalty;
    double local_reversing_penalty;
    double local_shot_distance;
    double local_vehicle_length;
    double local_vehicle_width;
    double local_vehicle_rear_axle_to_tailstock;

    geometry_msgs::Point pre_route_StartPoint;


    // 添加状态机相关内容===========================
    enum class State {
        WAIT_TASK,
        GLOBAL_TASK,
        LOCAL_TASK
    };
    enum class State_SendToSmach{ //0-等待 1-全局规划 2-全局规划收到目标点 3-局部规划
        WAIT_TASK,
        GLOBAL_TASK,
        GLOBAL_TASK_RECEIVED_TARGET,
        LOCAL_TASK
    };

    void publishEmptyPath(ros::Publisher& path_pub);

    ros::Timer timer_;
    void timerCallback(const ros::TimerEvent&);
    State currentState;
    void transitionState(State newState);
    void coutState();

    // 全局调度
    int task_;
    // 规划期状态反馈：
    // std_msgs::Int8 plan_state;
    int plan_state;

    // 定位：
    Point current_position;
    ros::Subscriber odom_sub_, cotton_sub_;
    ros::Publisher global_path_pub_, local_path_pub_;
    bool have_odom_;
    bool received_camera_;
    // 数据锁
    std::mutex data_mutex_;
    // 进入局部任务当时的定位点记录：
    double local_x, local_y, local_yaw;
    bool global_path_process, local_path_process;
    void cameraCameracallback(const car_interfaces::TargetDetectionInterface::ConstPtr& msg);
    void taskCallback(const std_msgs::Int8::ConstPtr& msg);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    car_interfaces::TargetDetectionInterface Detection_yolo;

    bool debug_mode;
    bool use_fsm;

    //====================


};

#endif //HYBRID_A_STAR_HYBRID_A_STAR_FLOW_H
