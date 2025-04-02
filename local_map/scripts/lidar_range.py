#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np


def callback(data):
    """
    回调函数，用于处理接收到的点云数据

    参数：
    data：接收到的sensor_msgs.msg.PointCloud2类型的点云消息
    """
    # 用于存储z坐标值的列表
    z_values = []
    # 遍历点云数据中的每个点
    for point in pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True):
        z_values.append(point[2])

    if z_values:
        # 将列表转换为numpy数组，方便计算最大值和最小值
        z_array = np.array(z_values)
        max_z = np.max(z_array)
        min_z = np.min(z_array)
        rospy.loginfo_once("Z坐标的最大值为: %f，最小值为: %f", max_z, min_z)
    else:
        rospy.loginfo("接收到的点云数据为空，无法获取Z坐标的最值。")


def listener():
    """
    初始化ROS节点并订阅点云话题，开始循环监听
    """
    rospy.init_node('rslidar_points_listener', anonymous=True)
    # 订阅名为rslidar_points的点云话题，当有新消息时，调用callback函数处理
    rospy.Subscriber('rslidar_points', PointCloud2, callback)
    # 保持节点运行，持续监听话题消息
    rospy.spin()


if __name__ == '__main__':
    listener()