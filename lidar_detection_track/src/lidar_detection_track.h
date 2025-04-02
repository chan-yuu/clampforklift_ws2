/*
 * @Author: CYUN && cyun@tju.enu.cn
 * @Date: 2025-01-13 14:46:23
 * @LastEditors: CYUN && cyun@tju.enu.cn
 * @LastEditTime: 2025-01-14 16:00:36
 * @FilePath: /undefined/home/nvidia/clamp_forklift_ws2/src/lidar_detection_track/src/lidar_detection_track.h
 * @Description: 
 * 
 * Copyright (c) 2025 by Tianjin University, All Rights Reserved. 
 */

#ifndef LIDAR_DETECTION_TRACK_H_
#define LIDAR_DETECTION_TRACK_H_

#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "autoware_msgs/Centroids.h"
#include "autoware_msgs/CloudCluster.h"
#include "autoware_msgs/CloudClusterArray.h"
#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp> // 窗口与图像显示模块
#include <opencv2/core/version.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include "../lib/ground_detector/patchwork/patchwork.h"
#include "../lib/visualization/visualize_detected_objects.h"
#include "../lib/pre_process/roi_clip/roi_clip.h"
#include "../lib/cluster/euclideanCluster/euclideanCluster.h"
#include "../lib/pre_process/voxel_grid_filter/voxel_grid_filter.h"
#include "../lib/bounding_box/bounding_box.h"
#include "../lib/visualization/visualize_detected_objects.h"

class lidarPerception
{
public:
    lidarPerception(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~lidarPerception(){};

    RoiClip roiClip_;
    VoxelGridFilter voxelGridFilter_;
    PatchWork patchWork_;
    EuclideanCluster cluster_;

    BoundingBox boundingBox_;
    VisualizeDetectedObjects vdo_;

    ros::Publisher _pub_in_cloud;
    ros::Publisher _pub_clip_cloud;
    ros::Publisher _pub_ground_cloud;
    ros::Publisher _pub_noground_cloud;
    ros::Publisher _pub_cluster_cloud;
    ros::Publisher _pub_clusters_message;

    ros::Publisher _pub_detected_objects;
    ros::Publisher _pub_detected_3Dobjects;

    ros::Publisher _pub_cluster_visualize_markers;
    ros::Publisher _pub_3Dobjects_visualize_markers;

    VisualizeDetectedObjects vdto;
    tf2_ros::TransformListener tf_listener;
    tf2_ros::Buffer tf_buffer;

    // lidar::PointPillars pointpillars;

private:
    void ClusterCallback(const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud);
    // void pointPillarsCallback(const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud);
    void publishDetectedObjects(const autoware_msgs::CloudClusterArray &in_clusters, autoware_msgs::DetectedObjectArray &detected_objects);
    // void Bbox3DToObjectArray(std_msgs::Header header, std::vector<lidar::Bbox3D> &boxes,autoware_msgs::DetectedObjectArray &detected_objects);
};

#endif
