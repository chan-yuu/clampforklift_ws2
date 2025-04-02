#!/usr/bin/env python3
'''
Author: CYUN && cyun@tju.edu.cn
Date: 2025-01-12 01:13:38
LastEditors: CYUN && cyun@tju.enu.cn
LastEditTime: 2025-01-13 22:08:15
FilePath: /undefined/home/cyun/forklift_sim_ws3/src/v2n/scripts/set_model.py
Description: 

Copyright (c) 2025 by Tianjin University, All Rights Reserved. 
'''
import rospy
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from geometry_msgs.msg import Pose, Point, Quaternion
import paho.mqtt.client as mqtt
import json
import time
import tf.transformations as transformations
from math import sqrt
import numpy as np


# 最大重连次数
MAX_RETRIES = 10
# 重连间隔时间（秒）
RETRY_INTERVAL = 5
# 移动距离阈值
DISTANCE_THRESHOLD = 0.1

# 真实世界
map_point = Pose(
    Point(-18.61746488697827, 3.7149446653784253, -0.010833443688170519),
    Quaternion(0.017790569670201747, -0.010301347747396106, -0.03497640639087085, 0.9991766754990719)
)

# sim中
world_point = Pose(
    Point(-6.875441083288509, 11.793098296190207, -0.010052702574130075),
    Quaternion(-4.333471353001964e-06, 0.00012189079419565796, 0.035296221324580775, 0.9993768868070051)
)
# 给定的world坐标系下的点



def create_translation_matrix(translation):
    """创建平移矩阵"""
    matrix = np.eye(4)
    matrix[0, 3] = translation.x
    matrix[1, 3] = translation.y
    matrix[2, 3] = translation.z
    return matrix


def create_rotation_matrix(quaternion):
    """创建旋转矩阵"""
    rotation_matrix = np.array(transformations.quaternion_matrix([quaternion.x, quaternion.y, quaternion.z, quaternion.w]))
    return rotation_matrix


def create_transform_matrix(map_pose, world_pose):
    """创建从map到world的坐标转换矩阵"""
    map_translation_matrix = create_translation_matrix(map_pose.position)
    map_rotation_matrix = create_rotation_matrix(map_pose.orientation)
    world_translation_matrix = create_translation_matrix(world_pose.position)
    world_rotation_matrix = create_rotation_matrix(world_pose.orientation)

    map_to_world_matrix = np.dot(world_rotation_matrix, np.linalg.inv(map_rotation_matrix))
    map_to_world_matrix[0:3, 3] = world_pose.position.x - map_pose.position.x, world_pose.position.y - map_pose.position.y, world_pose.position.z - map_pose.position.z
    return map_to_world_matrix


# 计算矩阵
map_to_world_matrix = create_transform_matrix(map_point, world_point)


def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    client.subscribe("/odom/v2n")


class ModelStateUpdater:
    def __init__(self):
        self.latest_pose = None
        self.previous_pose = None
        self.is_mqtt_connected = False

        rospy.init_node('model_state_updater')
        # 等待 set_model_state 服务可用
        rospy.wait_for_service('/gazebo/set_model_state')

        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.username_pw_set(username="tju_me", password="tjuME!@#")
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        for attempt in range(MAX_RETRIES):
            try:
                self.mqtt_client.connect("60.28.24.166", 1883)
                print("Successfully connected to MQTT broker.")
                self.is_mqtt_connected = True
                break
            except Exception as e:
                print(f"Failed to connect to MQTT broker: {e}. Retrying in {RETRY_INTERVAL} seconds...")
                time.sleep(RETRY_INTERVAL)
        else:
            print("Failed to connect to MQTT broker after multiple attempts.")

        if self.is_mqtt_connected:
            self.mqtt_client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        print(f"Connected with result code {rc}")
        client.subscribe("/odom/v2n")

    def on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
            print(f"Received message: {data}")
            pose = data.get("pose")
            if pose:
                position = pose.get("position")
                orientation = pose.get("orientation")
                if position and orientation:
                    x = position.get("x")
                    y = position.get("y")
                    z = 0#position.get("z")
                    qx = orientation.get("x")
                    qy = orientation.get("y")
                    qz = orientation.get("z")
                    qw = orientation.get("w")

                    (roll, pitch, yaw) = transformations.euler_from_quaternion([qx, qy, qz, qw],'sxyz')
                    roll = 0.0
                    pitch = 0.0
                    # 将修改后的欧拉角转换回四元数
                    quaternion = transformations.quaternion_from_euler(roll, pitch, yaw,'sxyz')
                    qx, qy, qz, qw = quaternion

                    # 订阅到的定位x y z qx  qy  qz qw
                    # 计算转换后的定位
                    map_pose = Pose(
                        Point(x, y, z),
                        Quaternion(qx, qy, qz, qw)
                    )
                    map_pose_np = np.array([[map_pose.position.x],
                                            [map_pose.position.y],
                                            [map_pose.position.z],
                                            [1]])
                    world_pose_np = np.dot(map_to_world_matrix, map_pose_np)
                    world_pose = Pose(
                        Point(world_pose_np[0, 0], world_pose_np[1, 0], world_pose_np[2, 0]),
                        Quaternion(0, 0, 0, 1)
                    )
                    self.latest_pose = world_pose
        except json.JSONDecodeError:
            pass

    def _calculate_distance(self, pose1, pose2):
        """Calculate the Euclidean distance between two poses."""
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        dz = pose1.position.z - pose2.position.z
        return sqrt(dx * dx + dy * dy + dz * dz)

    def update_model_state_at_rate(self, rate_hz = 1):
        rate = rospy.Rate(rate_hz)  # 设置更新频率为1Hz
        while not rospy.is_shutdown():
            if self.latest_pose is not None:
                if self.previous_pose is None or self._calculate_distance(self.latest_pose, self.previous_pose) >= DISTANCE_THRESHOLD:
                    model_state_req = SetModelStateRequest()
                    model_state_req.model_state.model_name = "smart"
                    model_state_req.model_state.pose = self.latest_pose
                    try:
                        self.set_model_state(model_state_req)
                        print("Updated model state in Gazebo.")
                        self.previous_pose = self.latest_pose  # 更新上一个位置
                    except rospy.ServiceException as e:
                        rospy.logerr("Service call failed: %s", e)
                else:
                    print("Movement less than threshold, skipping update.")
            rate.sleep()


if __name__ == "__main__":
    updater = ModelStateUpdater()
    if updater.is_mqtt_connected:
        try:
            updater.update_model_state_at_rate(rate_hz = 1)
        finally:
            updater.mqtt_client.loop_stop()
