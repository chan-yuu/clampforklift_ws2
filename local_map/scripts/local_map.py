#!/usr/bin/env python
import rospy
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from nav_msgs.msg import OccupancyGrid
import math
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2


def point_cloud_callback(data):
    start_time = rospy.Time.now()  # 记录开始时间

    # 从参数服务器获取参数
    x_min = rospy.get_param('~x_min', -1)
    x_max = rospy.get_param('~x_max', 1)
    y_min = rospy.get_param('~y_min', -1)
    y_max = rospy.get_param('~y_max', 1)
    z_min = rospy.get_param('~z_min', -2.0)
    z_max = rospy.get_param('~z_max', 3)

    filtered_points = []
    min_z = float('inf')

    for point in pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True):
        if z_min <= point[2] <= z_max and (
            (point[0] >= x_max or point[0] <= x_min) or
            (point[1] >= y_max or point[1] <= y_min)
        ):
            filtered_points.append((point[0], point[1]))
            if point[2] < min_z:
                min_z = point[2]
    end_time = rospy.Time.now()  # 记录结束时间
    process_time = (end_time - start_time).to_sec()  # 计算处理时间（单位：秒）
    rospy.loginfo("滤波时间: %f 秒", process_time)

    if filtered_points:
        resolution = 0.05
        width = 400
        height = 400
        min_x = -10.0
        min_y = -10.0

        # 创建TF监听器和缓冲区
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        try:
            # 获取从rslidar坐标系到base_footprint坐标系的坐标变换
            transform = tf_buffer.lookup_transform('base_footprint', data.header.frame_id, rospy.Time(0), rospy.Duration(1.0))

            # 对每个过滤后的点进行坐标变换
            transformed_points = []
            for point in filtered_points:
                point_stamped = PointStamped()
                point_stamped.point.x = point[0]
                point_stamped.point.y = point[1]
                point_stamped.point.z = 0.0
                point_stamped.header.frame_id = data.header.frame_id
                point_stamped.header.stamp = rospy.Time(0)

                transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
                transformed_points.append((transformed_point.point.x, transformed_point.point.y))

            grid_map = OccupancyGrid()
            grid_map.header.frame_id = 'base_footprint'  # 设置为base_footprint坐标系
            grid_map.header.stamp = rospy.Time.now()
            grid_map.info.width = width
            grid_map.info.height = height
            grid_map.info.resolution = resolution
            grid_map.info.origin.position.x = min_x
            grid_map.info.origin.position.y = min_y
            grid_map.info.origin.position.z = 0.0
            grid_map.info.origin.orientation.x = 0.0
            grid_map.info.origin.orientation.y = 0.0
            grid_map.info.origin.orientation.z = 0.0
            grid_map.info.origin.orientation.w = 1.0

            grid_data = np.zeros(width * height, dtype=np.int8)

            for point in transformed_points:
                grid_x = int((point[0] - min_x) / resolution)
                grid_y = int((point[1] - min_y) / resolution)
                if 0 <= grid_x < width and 0 <= grid_y < height:
                    grid_data[grid_y * width + grid_x] = 100

            grid_map.data = grid_data.tolist()
            pub.publish(grid_map)
            end_time = rospy.Time.now()  # 记录结束时间
            process_time = (end_time - start_time).to_sec()  # 计算处理时间（单位：秒）
            rospy.loginfo("处理并发布栅格地图耗时: %f 秒", process_time)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF lookup failed: %s", str(e))
            end_time = rospy.Time.now()  # 记录异常时的结束时间
            process_time = (end_time - start_time).to_sec()  # 计算处理时间（单位：秒）
            rospy.logwarn("整体处理耗时（含异常情况）: %f 秒", process_time)
    else:
        rospy.loginfo("failed! ")
        end_time = rospy.Time.now()  # 记录结束时间（处理失败情况）
        process_time = (end_time - start_time).to_sec()  # 计算处理时间（单位：秒）
        rospy.loginfo("处理失败，耗时: %f 秒", process_time)


if __name__ == '__main__':
    rospy.init_node('rslidar_points_processor', anonymous=True)
    pub = rospy.Publisher('local_map', OccupancyGrid, queue_size=1)
    rospy.Subscriber('rslidar_points', PointCloud2, point_cloud_callback)
    rospy.spin()