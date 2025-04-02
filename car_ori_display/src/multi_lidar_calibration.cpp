/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2024-09-13 11:20:19
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2024-09-15 14:17:39
 * @FilePath: /src/car_ori_display/src/multi_lidar_calibration.cpp
 * @Description: 用于对雷达的联合标定
 * 
 * Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
 */
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>
#include <XmlRpcValue.h>
struct LidarCalibration
{
    std::string topic;
    tf::Vector3 translation;
    tf::Quaternion rotation;
};

class MultiLidarCalibration
{
public:
    MultiLidarCalibration(ros::NodeHandle& nh) :
        nh_(nh),
        tf_listener_(tf_buffer_)
    {
        nh_.param<std::string>("main_lidar_topic", main_lidar_topic_, "/main_lidar");

        XmlRpc::XmlRpcValue lidars;
        nh_.getParam("other_lidars", lidars);

        for (int i = 0; i < lidars.size(); ++i)
        {
            LidarCalibration calibration;
            calibration.topic = static_cast<std::string>(lidars[i]["topic"]);
            calibration.translation = tf::Vector3(
                static_cast<double>(lidars[i]["x"]),
                static_cast<double>(lidars[i]["y"]),
                static_cast<double>(lidars[i]["z"])
            );
            double roll = static_cast<double>(lidars[i]["roll"]);
            double pitch = static_cast<double>(lidars[i]["pitch"]);
            double yaw = static_cast<double>(lidars[i]["yaw"]);
            calibration.rotation = tf::createQuaternionFromRPY(roll, pitch, yaw);

            other_lidar_calibrations_.push_back(calibration);
            ros::Subscriber sub = nh_.subscribe(calibration.topic, 1, &MultiLidarCalibration::otherLidarCallback, this);
            other_lidar_subs_.push_back(sub);
        }

        main_lidar_sub_ = nh_.subscribe(main_lidar_topic_, 1, &MultiLidarCalibration::mainLidarCallback, this);
        fused_lidar_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/fused_lidar", 1);
    }

private:
    void mainLidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        for (const auto& point : cloud)
        {
            transformed_cloud->points.push_back(point);
        }

        for (const auto& other_cloud : other_clouds_)
        {
            *transformed_cloud += *other_cloud;
        }

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*transformed_cloud, output);
        output.header.frame_id = msg->header.frame_id;
        fused_lidar_pub_.publish(output);

        other_clouds_.clear();
    }

    void otherLidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        std::string frame_id = msg->header.frame_id;
        auto it = std::find_if(other_lidar_calibrations_.begin(), other_lidar_calibrations_.end(),
                               [frame_id](const LidarCalibration& cal) { return cal.topic == frame_id; });

        if (it == other_lidar_calibrations_.end())
        {
            ROS_WARN("No calibration data found for topic: %s", frame_id.c_str());
            return;
        }

        const LidarCalibration& calibration = *it;
        tf::Transform transform(calibration.rotation, calibration.translation);

        for (const auto& point : cloud)
        {
            tf::Vector3 tf_point(point.x, point.y, point.z);
            tf::Vector3 transformed_point = transform * tf_point;
            transformed_cloud->points.push_back(pcl::PointXYZ(transformed_point.x(), transformed_point.y(), transformed_point.z()));
        }

        other_clouds_.push_back(transformed_cloud);
    }

    ros::NodeHandle& nh_;
    ros::Subscriber main_lidar_sub_;
    std::vector<ros::Subscriber> other_lidar_subs_;
    ros::Publisher fused_lidar_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string main_lidar_topic_;
    std::vector<LidarCalibration> other_lidar_calibrations_;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> other_clouds_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "multi_lidar_calibration");
    ros::NodeHandle nh;

    MultiLidarCalibration multi_lidar_calibration(nh);

    ros::spin();

    return 0;
}
