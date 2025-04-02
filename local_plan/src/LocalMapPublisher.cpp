/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-09-24 09:13:47
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-10-06 11:02:16
 * @FilePath: /undefined/home/cyun/learn_ws/plan_ws3/src/local_plan/src/LocalMapPublisher.cpp
 * @Description: 
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

class LocalMapPublisher
{
public:
    LocalMapPublisher(ros::NodeHandle& nh)
    {
        nh.param("map_width", map_width_, 10.0);
        nh.param("map_height", map_height_, 10.0);
        nh.param("resolution", resolution_, 0.1);
        nh.param("front_axle_frame", front_axle_frame_, std::string("base_link"));

        map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/local_map", 10);

        initializeMap();
    }

    void update() 
    {
        tf::StampedTransform transform;
        try
        {
            // 获取map到front_axle的变换
            listener_.lookupTransform("map", front_axle_frame_
            , ros::Time(0), transform);

            double vehicle_x = transform.getOrigin().x();
            double vehicle_y = transform.getOrigin().y();

            map_msg_.info.origin.position.x = vehicle_x - map_width_ / 2.0;
            map_msg_.info.origin.position.y = vehicle_y - map_height_ / 2.0;
            publishMap(); 
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
        }
    }

    void initializeMap() 
    {
        int width_in_cells = static_cast<int>(map_width_ / resolution_);
        int height_in_cells = static_cast<int>(map_height_ / resolution_);

        // 设置地图消息头
        map_msg_.header.frame_id = "map";  // 地图参考的坐标系
        map_msg_.info.resolution = resolution_;  // 地图的分辨率
        map_msg_.info.width = width_in_cells;  // 地图宽度（栅格数）
        map_msg_.info.height = height_in_cells;  // 地图高度（栅格数）
        map_msg_.info.origin.position.z = 0.0;  // 地图原点的z坐标
        map_msg_.info.origin.orientation.w = 1.0;  // 地图原点的方向（单位四元数）

        // 填充地图数据，设置为无障碍物
        map_msg_.data.resize(width_in_cells * height_in_cells, 0);  // 所有栅格单元初始值设为0（可通行）
    }

    void publishMap()
    {
        map_msg_.header.stamp = ros::Time::now();  // 设置当前时间戳
        map_pub_.publish(map_msg_);  // 发布地图消息
    }

private:
    ros::Publisher map_pub_;
    tf::TransformListener listener_;
    nav_msgs::OccupancyGrid map_msg_;

    double map_width_;
    double map_height_;
    double resolution_;
    std::string front_axle_frame_; // 用于存储前轴框架名称
};

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "local_map_publisher");
    ros::NodeHandle nh;

    // 创建地图发布器对象
    LocalMapPublisher map_publisher(nh);

    ros::Rate rate(10); // 设置更新频率
    while (ros::ok())
    {
        map_publisher.update(); // 更新地图
        ros::spinOnce(); // 处理回调
        rate.sleep(); // 等待下一次循环
    }

    return 0;
}
