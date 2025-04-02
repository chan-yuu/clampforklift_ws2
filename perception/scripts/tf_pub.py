#!/usr/bin/python3
# -*- coding: utf-8 -*-

# 作者： cyun
import rospy
import tf2_ros
import tf_conversions
import geometry_msgs.msg
from math import radians
import math

def publish_transform(x, y, yaw):
    # print(1)
    looprate = rospy.Rate(100)
    # 创建 broadcaster
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    # 设置 header
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "world"

    # 设置变换的平移部分
    t.transform.translation.x = -x
    
    t.transform.translation.y = -y
    t.transform.translation.z = 0

    # 设置变换的旋转部分
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, -radians(yaw))
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    # 发布变换
    br.sendTransform(t)
    looprate.sleep()


def convert(north_bearing):
    '''
    gps_angle -> map_angle
    '''
    east_bearing = 90 - north_bearing
    
    # Adjust if the result is negative to ensure it's within 0-360 degrees
    east_bearing = (east_bearing + 360) % 360

    return east_bearing

def convert_to_rad(angle):
    """
    0-360 -> -pi - pi
    """
    if angle > 180:
        angle -= 360
    angle = angle*math.pi/180
    return angle

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')

    x = rospy.get_param('x',-1.56133)
    y = rospy.get_param('y',-4.49949)
    yaw = rospy.get_param('yaw',188.56)  # rad 188.56 ->
    print("yaw", yaw)
    yaw =  convert(yaw)
    yaw = convert_to_rad(yaw)
    # print(x,y,yaw)

    while not rospy.is_shutdown():
        publish_transform(x, y, yaw)  # 假设点(x, y, yaw) = (1, 2, 30度)
        # rospy.spin()


# <launch>
#     <!-- 定义节点 -->
#     <node name="tf_broadcaster" pkg="your_package_name" type="your_script_name.py" output="screen">
#         <!-- 传递参数 -->
#         <param name="x" value="1"/>
#         <param name="y" value="2"/>
#         <param name="yaw" value="30"/>  <!-- Yaw 角度 -->
#     </node>
# </launch>