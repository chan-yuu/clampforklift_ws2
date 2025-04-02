#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <car_interfaces/GetInitPose.h>
#include <car_interfaces/GetTargetPose.h>

class PoseManager {
private:
    ros::NodeHandle nh_;
    ros::Subscriber initial_pose_sub_;
    ros::Subscriber goal_sub_;

public:
    PoseManager() {
        initial_pose_sub_ = nh_.subscribe("/initialpose", 1, &PoseManager::initialPoseCallback, this);
        goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &PoseManager::goalCallback, this);
    }

    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        // 处理初始位置
        sendInitPose(msg->pose.pose.position, msg->pose.pose.orientation);
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 处理目标位置
        sendTargetPose(msg->pose.position, msg->pose.orientation);
    }

    void sendInitPose(const geometry_msgs::Point& point, const geometry_msgs::Quaternion& quaternion) {
        ROS_INFO("\033[91m Waiting for Sending init pose server... \033[0m");
        ros::service::waitForService("get_init_pose");
        ROS_INFO("\033[32mSending init pose server ready!\033[0m");

        car_interfaces::GetInitPose srv;
        srv.request.init_pose.header.stamp = ros::Time::now();
        srv.request.init_pose.header.frame_id = "map";
        srv.request.init_pose.pose.pose.position = point;
        srv.request.init_pose.pose.pose.orientation = quaternion;

        ros::ServiceClient client = nh_.serviceClient<car_interfaces::GetInitPose>("get_init_pose");
        if (client.call(srv)) {
            if (srv.response.success) {
                ROS_INFO("Init pose sent successfully: %s", srv.response.message.c_str());
            } else {
                ROS_INFO("Failed to send init pose: %s", srv.response.message.c_str());
            }
        } else {
            ROS_ERROR("Failed to call service get_init_pose");
        }
    }

    void sendTargetPose(const geometry_msgs::Point& point, const geometry_msgs::Quaternion& quaternion) {
        ROS_INFO("\033[91m Waiting for Sending target pose server... \033[0m");
        ros::service::waitForService("get_target_pose");
        ROS_INFO("\033[32mSending target pose server ready!\033[0m");

        car_interfaces::GetTargetPose srv;
        srv.request.tar_pose.header.stamp = ros::Time::now();
        srv.request.tar_pose.header.frame_id = "map";
        srv.request.tar_pose.pose.position = point;
        srv.request.tar_pose.pose.orientation = quaternion;

        ros::ServiceClient client = nh_.serviceClient<car_interfaces::GetTargetPose>("get_target_pose");
        if (client.call(srv)) {
            if (srv.response.success) {
                ROS_INFO("Target pose sent successfully: %s", srv.response.message.c_str());
            } else {
                ROS_INFO("Failed to send target pose: %s", srv.response.message.c_str());
            }
        } else {
            ROS_ERROR("Failed to call service get_target_pose");
        }
    }

    void sendPredefinedPoses() {
        // 初始点
        geometry_msgs::Point init_point;
        init_point.x = 13.0841;
        init_point.y = 2.7900;
        init_point.z = 0.0;

        geometry_msgs::Quaternion init_quat;
        init_quat.x = 0.0;
        init_quat.y = 0.0;
        init_quat.z = 0.9998;
        init_quat.w = 0.0195;

        sendInitPose(init_point, init_quat);

        // 途径点 (这里使用相同的点作为示例)
        sendInitPose(init_point, init_quat);

        // 终点
        geometry_msgs::Point target_point;
        target_point.x = 27.0779;
        target_point.y = -6.6748;
        target_point.z = 0.0;

        geometry_msgs::Quaternion target_quat;
        target_quat.x = 0.0;
        target_quat.y = 0.0;
        target_quat.z = -0.6937;
        target_quat.w = 0.7202;

        sendTargetPose(target_point, target_quat);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_manager_node");
    PoseManager pose_manager;
    // pose_manager.sendPredefinedPoses();
    ros::spin();
    return 0;
}