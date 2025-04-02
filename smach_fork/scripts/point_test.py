#!/usr/bin/env python3

import rospy
import smach
from std_msgs.msg import Float64,UInt8, Int8
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import sys
import time
import tf.transformations as tf
from nav_msgs.msg import Odometry
import math
from nav_msgs.msg import Path
from scipy.spatial.transform import Rotation
import numpy as np
import threading

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
import math
from tf.transformations import quaternion_from_euler

import rospkg
import csv
import os

rospack = rospkg.RosPack()
package_path = rospack.get_path('smach_fork')

points_file_path = f'{package_path}/data/saved_points.txt'
# 保存点到txt文件的函数
def save_point_to_txt(point_name, point_data):
    """
    保存点到txt文件中。
    :param point_name: 点的名称，例如 'point1'
    :param point_data: 点的数据，格式为 ((x, y, z), (qx, qy, qz, qw))
    """
    # with open(points_file_path, 'a') as txtfile:
    #     txtfile.write(f"{point_name}={point_data}\n")
    # rospy.loginfo(f"Point {point_name} saved to {points_file_path}")
    point_str = f"{point_name}:{point_data}\n"
    with open(points_file_path, 'a') as txtfile:
        txtfile.write(point_str)
    rospy.loginfo(f"Point {point_name} saved to {points_file_path}")

# 存储的文件
csv_file_path = f'{package_path}/data/state_times.csv'
csv_file_point_path = f'{package_path}/data/current_point.csv'

if not os.path.exists(csv_file_path):
    with open(csv_file_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["State Name", "Start Time", "End Time", "Specific Start Time", "Specific End Time", "Elapsed Time"])

if not os.path.exists(csv_file_point_path):
    with open(csv_file_point_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["x", "y", "yaw"])


# 仿真的状态：
from smach_sim_state import SetHeightSim, SetWidthSim
from sensor_msgs.msg import JointState


position_ = 0
bales_num_ = 0
current_x = 0.0
current_y = 0.0
current_yaw = 0.0
delivery_point = ()
task_id  = None
shelf_name = None

# 循环作业使用:
num_work = 0
num_work_max = 0
num_work_height = 0
work_direction = 0

cotton_4_num = 0


first_time_observer = True
observer_point = ((0,0,0.0000),(0, 0, 0.0, 0.0))
cotton_x=0
cotton_y=0
cotton_z=0
cotton_yaw=0
cotton_width=0
cotton_height=0
conveyer_belt_point = None

move_distance = 0 # 延迟下降

# 父类,初始化  话题和函数
# 因为服务的特殊性，不能在这里初始化
# 状态中的重复内容全部定义到这里
class SerManager:
    def __init__(self):
        self.position_state = 0  # 初始化 position_state 属性
        self.brake_enable = 0
        self.position_state_service = None
        self.bales_num_in_camera_server = None
        self.bales_num_in_camera = 0
        self.camera_state = 0
        self.control_state = 0
        self.plan_state = 0

        self.set_tar_width_pub = rospy.Publisher('/clamp_cmd', Float64, queue_size=1)
        self.set_tar_height_pub = rospy.Publisher('/updown_cmd', Float64, queue_size=1)
        self.set_tar_height_pub_ = rospy.Publisher('/updown_mode', Int8, queue_size=1)

        self.set_tar_fy_pub = rospy.Publisher('/fy_cmd', Float64, queue_size=1)
        self.set_tar_lateral_pub = rospy.Publisher('/lateral_cmd', Float64, queue_size=1)
        self.path_publisher = rospy.Publisher('/smach_path', Path, queue_size=1)
        self.steering_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=1)
        self.throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=1)
        self.gear_pub = rospy.Publisher('/gear_cmd', UInt8, queue_size=1)
        self.brake_pub = rospy.Publisher('/brake_cmd', Float64, queue_size=1)

        self.current_yaw_reverse_distance = rospy.get_param("/smach/current_yaw_reverse_distance", 0.8)
        self.goal_yaw_reverse_distance = rospy.get_param("/smach/goal_yaw_reverse_distance", -0.8)

        self.adjust_throttle = rospy.get_param("/smach/adjust_throttle", 0.05)
        self.lateral_distance = rospy.get_param("/smach/lateral_distance", 1.0)


    #=============================================================================
    #---------------------------lib-------------------------------
    #--------------------------------------------------------------------------

    def create_state_machine(self, cotton_4_num):

        delivery_point=[[-5.0265,-24.9731,0.0000],[0.0000,0.0000,-0.7121,0.7020]]
        delivery_point1 = self.calculate_reverse_point(delivery_point, 1)
        # 放货点再向后
        # 标定
        delivery_point2 = self.calculate_reverse_point(delivery_point1, self.goal_yaw_reverse_distance)
        print("delivery_point2", delivery_point2)
        # 旋转
        delivery_point3 = self.calculate_new_point_rotation(delivery_point2, 1)
        print("delivery_point3", delivery_point3)
        # 旋转过后再向后
        print("self.current_yaw_reverse_distance",self.current_yaw_reverse_distance)
        delivery_point4 = self.calculate_reverse_point(delivery_point3, self.current_yaw_reverse_distance - cotton_4_num * 1.5)
        print("delivery_point4",delivery_point4)
        save_point_to_txt("delivery_point4", delivery_point4)
        delivery_point6 = self.calculate_reverse_point(delivery_point4, 4)
        delivery_point7 = self.calculate_reverse_point(delivery_point6, 8)
        print("point6: ", delivery_point6)


    #@ 四元数转为角度
    def get_yaw_from_quaternion(self, quaternion):
        _, _, yaw = tf.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return yaw
    

    #@ yaw 转为四元数
    def yaw_to_quaternion(self, yaw):
        # 创建一个只包含yaw旋转的Rotation对象
        rotation = Rotation.from_euler('z', yaw, degrees=True)
        
        # 获取四元数表示
        quaternion = rotation.as_quat()
        
        # 将scipy的四元数转换为geometry_msgs.msg.Quaternion
        quat_msg = Quaternion()
        quat_msg.x = quaternion[0]
        quat_msg.y = quaternion[1]
        quat_msg.z = quaternion[2]
        quat_msg.w = quaternion[3]
        return quat_msg

    #@ 计算当前航向的前进和倒退点位置 正distance为倒车，负为前进
    def calculate_reverse_point(self, point_goal, distance):
        x, y, z, w = point_goal[1]
        yaw = math.atan2(2 * (w * z), 1 - 2 * z * z)
        x_new = point_goal[0][0] - distance * math.cos(yaw)
        y_new = point_goal[0][1] - distance * math.sin(yaw)
        return ((x_new, y_new, 0.0), (0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2)))

    #@ 计算当前点的左右位置的点 direction如下定义
    def calculate_new_point(self, point_goal, direction, distance):
        x, y, z, w = point_goal[1]
        yaw = math.atan2(2 * (w * z), 1 - 2 * z * z)
        if direction == 1:
            # 右边 10.31
            new_yaw = yaw - math.pi/2
        elif direction == 2:
            # 左边 10.31
            new_yaw = yaw + math.pi/2
        else:
            raise ValueError("Invalid direction. Use 1 for left or 2 for right.")
        x_new = point_goal[0][0] + distance * math.cos(new_yaw)
        y_new = point_goal[0][1] + distance * math.sin(new_yaw)
        return ((x_new, y_new, 0.0), (0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2)))

    #@ 计算当前点的旋转过后的点 都是旋转90度 direction如下定义
    def calculate_new_point_rotation(self, point_goal, direction):
        x, y, z, w = point_goal[1]
        yaw = math.atan2(2 * (w * z), 1 - 2 * z * z)
        if direction == 1:
            # 1 顺时针
            new_yaw = yaw - math.pi / 2
        elif direction == 2:
            # 2 逆时针
            new_yaw = yaw + math.pi / 2
        else:
            raise ValueError("Invalid direction. Use 1 for left or 2 for right.")
        return ((point_goal[0][0], point_goal[0][1], 0.0), (0.0, 0.0, math.sin(new_yaw / 2), math.cos(new_yaw / 2)))


    #@ 下面函数用于计算AdjustHeading_Design_Angle AdjustHeading中的需要旋转的角度
    def normalize_angle(self, angle):
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle
    def calculate_optimal_rotation(self, current_yaw, goal_yaw):
        raw_angle = goal_yaw - current_yaw
        
        normalized_angle = self.normalize_angle(raw_angle)
        
        if normalized_angle > 0:
            clockwise_angle = 360 - normalized_angle
            counterclockwise_angle = normalized_angle
        else:
            clockwise_angle = -normalized_angle
            counterclockwise_angle = 360 + normalized_angle

        if clockwise_angle < counterclockwise_angle:
            direction = 1 # 顺时针
            rotation_angle = clockwise_angle
        else:
            direction = 2 # 逆时针
            rotation_angle = counterclockwise_angle
        
        return direction, rotation_angle/180*math.pi

    #@ 计算当前航向与目标航向的角度差
    def calculate_angle(self, current_yaw, target_yaw):
        diff = target_yaw - current_yaw
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

    #@ 将点（x,y,yaw）转换为四元数
    def process_point(self, intersection_x, intersection_y, intersection_yaw):
        quat = tf.quaternion_from_euler(0, 0, intersection_yaw)
        return ((intersection_x, intersection_y, 0.0), (quat[0], quat[1], quat[2], quat[3]))

    #@ 点到线距离
    def point_to_line_distance(self, current_x, current_y, x1, y1, goal_slope):
        A = goal_slope
        B = -1
        C = y1 - goal_slope * x1
        distance = abs(A * current_x + B * current_y + C) / math.sqrt(A ** 2 + B ** 2)

        return distance

    # @ 存储点位
    def save_point(self):
        global current_x,current_y, current_yaw

        with open(csv_file_path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([current_x, current_y, current_yaw])

    #@ 计算两个点之间的航向
    def calculate_heading(self, point_start, point_end):
        """计算从起点到终点的航向角"""
        dx = point_end[0] - point_start[0]
        dy = point_end[1] - point_start[1]
        return math.atan2(dy, dx)

    #@ 生成轨迹 list格式
    def generate_trajectory(self, pointA, pointB, step_size=0.1):
        """生成从pointA到pointB的离散轨迹，每个点都有坐标和航向"""
        start_pos = pointA[0][:2]  # 只取x,y坐标
        end_pos = pointB[0][:2]    # 只取x,y坐标
        
        yaw = self.calculate_heading(start_pos, end_pos)
        distance = math.sqrt((end_pos[0] - start_pos[0]) ** 2 + (end_pos[1] - start_pos[1]) ** 2)
        trajectory = []
        steps = int(distance / step_size) + 1
        for i in range(steps):
            t = i * step_size / distance
            x_new = start_pos[0] + t * (end_pos[0] - start_pos[0])
            y_new = start_pos[1] + t * (end_pos[1] - start_pos[1])
            
            # 将四元数设置为基于yaw角
            q_yaw = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
            
            trajectory.append(((x_new, y_new, 0.0), q_yaw))
        
        if distance % step_size != 0:
            trajectory.append((end_pos[0], end_pos[1], 0.0), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))
        
        return trajectory

    #@ 生成轨迹 标准path格式
    def create_path_segment(self, x1, y1, x2, y2, step=0.1):
        """Create a path segment between two points."""
        path = Path()
        path.header.frame_id = "map"  # 设置参考坐标系
        path.header.stamp = rospy.Time.now()

        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        if distance == 0:
            return path  # 如果两点重合，返回空路径

        num_steps = int(math.ceil(distance / step))
        dx = (x2 - x1) / num_steps if num_steps > 0 else 0
        dy = (y2 - y1) / num_steps if num_steps > 0 else 0

        for i in range(num_steps + 1):  # 包含终点
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = x1 + i * dx
            pose.pose.position.y = y1 + i * dy
            pose.pose.position.z = 0.0  # 假设为平面路径

            # 计算航向角并设置姿态（可选）
            heading = math.atan2(dy, dx)
            quaternion = quaternion_from_euler(0, 0, heading)
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]

            path.poses.append(pose)
        return path

    def merge_paths(self, path1, path2):
        """Merge two paths into one."""
        merged_path = Path()
        merged_path.header = path1.header  # 使用第一个路径的header信息

        # 添加第一条路径的所有点
        merged_path.poses.extend(path1.poses)

        # 添加第二条路径的所有点，确保没有重复添加起点
        if len(path2.poses) > 0 and len(path1.poses) > 0:
            if path2.poses[0] != path1.poses[-1]:
                merged_path.poses.extend(path2.poses)
            else:
                merged_path.poses.extend(path2.poses[1:])  # 跳过重复的起点

        return merged_path

    # 使用方法：
    # def publish_path():
    #     rospy.init_node('path_publisher', anonymous=True)
    #     pub = rospy.Publisher('/global_path', Path, queue_size=10)
    #     rate = rospy.Rate(10)  # 10hz

    #     # 定义两个路径的起始和结束坐标
    #     x1, y1 = 0.0, 0.0
    #     x2, y2 = 10.0, 10.0
    #     x3, y3 = 15.0, -15.0

    #     while not rospy.is_shutdown():
    #         path1 = create_path_segment(x1, y1, x2, y2)
    #         path2 = create_path_segment(x2, y2, x3, y3)
    #         full_path = merge_paths(path1, path2)
    #         pub.publish(full_path)
    #         rate.sleep()


    #---------------------------lib-------------------------------

    
test = SerManager()

test.create_state_machine(0)
