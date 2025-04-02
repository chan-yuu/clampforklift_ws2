#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <car_interfaces/PathGlobal.h>

/*
path发布的是设置的点位的数据，也就是一堆起点的和一个goal点
如何中断消息？
新的init
*/

// nav_msgs::Path path;
car_interfaces::PathGlobal global_path;
bool goal_received = false;
ros::Publisher path_publisher;
nav_msgs::Path new_path;

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    // 创建新的路径  收到新的init时清空路径
    if(goal_received == true){
        goal_received = false;
        // nav_msgs::Path new_path;
        // nav_msgs::Path path;
        new_path.poses.clear();
        // path.poses.clear();
    }

    new_path.header = msg->header;

    // 获取最后一个点的坐标和方向
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = msg->header;
    pose_stamped.pose.position = msg->pose.pose.position;
    pose_stamped.pose.orientation = msg->pose.pose.orientation;

    // 添加点到路径
    new_path.poses.push_back(pose_stamped);

    // 打印最后一个点的坐标和方向
    geometry_msgs::Point point = pose_stamped.pose.position;
    geometry_msgs::Quaternion orientation = pose_stamped.pose.orientation;
    ROS_INFO("Received initial pose: (%f, %f, %f)", point.x, point.y, point.z);
    ROS_INFO("Orientation: (%f, %f, %f, %f)", orientation.x, orientation.y, orientation.z, orientation.w);

    // 清空旧路径并更新为新路径
    // path = new_path;
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // 创建新的路径
    // nav_msgs::Path new_path;
    // new_path.header = msg->header;

    // 添加最后一个点到路径
    new_path.poses.push_back(*msg);

    // 更新路径为新路径
    // path = new_path;

    // 标记接收到目标点
    goal_received = true;
    ROS_INFO("Received goal pose. Task completed.");
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "task_publisher");
    // 私有命名空间的NodeHandle，用于发布
    ros::NodeHandle nh_private;
    // 全局命名空间的NodeHandle，用于订阅全局命名空间下的主题
    ros::NodeHandle nh_global;
    path_publisher = nh_private.advertise<car_interfaces::PathGlobal>("/rviz_points", 1);

    // 服务：客户端，通过initialpose获得多个起点，/move_base_simple/goal获得终点后就再请求一个服务给规划：
    ros::Subscriber initial_pose_subscriber = nh_global.subscribe("/initialpose", 1, initialPoseCallback);
    ros::Subscriber goal_subscriber = nh_global.subscribe("/move_base_simple/goal", 1, goalCallback);
    ros::Rate rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        // 判断是否已经接收到过目标点
        if(!goal_received)
            continue;

            // 发布当前路径
            global_path.path = new_path;
            global_path.goal_received = goal_received; //meiyong
            new_path.header.stamp = ros::Time::now();
            std::cout<<"pub ok"<<"rviz_points size is "<<new_path.poses.size()<<std::endl;
            path_publisher.publish(global_path);


        rate.sleep();
    }

    return 0;
}