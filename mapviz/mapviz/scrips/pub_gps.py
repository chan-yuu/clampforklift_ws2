#!/usr/bin/env python3
'''
Author: CYUN && cyun@tju.enu.cn
Date: 2024-12-13 20:12:55
LastEditors: CYUN && cyun@tju.enu.cn
LastEditTime: 2025-02-21 15:27:47
FilePath: /scripts/home/cyun/forklift_sim_ws3/src/mapviz/mapviz/scrips/pub_gps.py
Description: 

Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
'''
import rospy
from sensor_msgs.msg import NavSatFix
import utm
import math

# 天津的一个经纬度坐标示例
latitude = 38.9973678  # 纬度
longitude = 117.2978343  # 经度
# 定义生成轨迹的点数
num_points = 1000
# 定义轨迹点之间的距离间隔（单位：米，这里只是示例值，可按需调整）
distance_interval = 1

# 用于将经纬度转换为UTM坐标
def latlon_to_utm(lat, lon):
    return utm.from_latlon(lat, lon)

# 用于将UTM坐标转换回经纬度
def utm_to_latlon(easting, northing, zone_number, zone_letter):
    return utm.to_latlon(easting, northing, zone_number, zone_letter)

# 基于起始坐标生成轨迹（在UTM坐标下简单模拟，这里是直线轨迹示例，可按需扩展更复杂逻辑）
def generate_utm_trajectory(lat, lon, num_points, distance_interval):
    utm_start = latlon_to_utm(lat, lon)
    zone_number = utm_start[2]
    zone_letter = utm_start[3]
    utm_trajectory = []
    x = utm_start[0]
    y = utm_start[1]
    for _ in range(num_points):
        utm_trajectory.append((x, y, zone_number, zone_letter))
        # 简单地在x方向上按照距离间隔增加，模拟轨迹移动（可按实际需求调整轨迹生成逻辑）
        x += distance_interval
    return utm_trajectory

# 将UTM轨迹转换回经纬度轨迹
def utm_trajectory_to_latlon(utm_trajectory):
    latlon_trajectory = []
    for point in utm_trajectory:
        lat, lon = utm_to_latlon(point[0], point[1], point[2], point[3])
        latlon_trajectory.append((lat, lon))
    return latlon_trajectory

# 模拟发布经纬度轨迹信息（通过sensor_msgs/NavSatFix消息类型，实际需在ROS环境运行）
def publish_latlon_trajectory(latlon_trajectory):
    rospy.init_node('trajectory_publisher', anonymous=True)
    pub = rospy.Publisher('trajectory', NavSatFix, queue_size=10)
    rate = rospy.Rate(10)  # 1Hz的发布频率，可按需调整
    for lat, lon in latlon_trajectory:
        navsatfix_msg = NavSatFix()
        navsatfix_msg.latitude = lat
        navsatfix_msg.longitude = lon
        navsatfix_msg.altitude = 0.0  # 这里简单设为0，可按实际补充准确高度信息
        pub.publish(navsatfix_msg)
        rate.sleep()

if __name__ == "__main__":
    utm_trajectory = generate_utm_trajectory(latitude, longitude, num_points, distance_interval)
    latlon_trajectory = utm_trajectory_to_latlon(utm_trajectory)
    publish_latlon_trajectory(latlon_trajectory)
    
    rospy.spin()