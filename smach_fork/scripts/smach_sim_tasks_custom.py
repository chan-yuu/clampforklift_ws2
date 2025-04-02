#!/usr/bin/env python3

import rospy
import smach
from std_msgs.msg import Float64,UInt8, Int8
from car_interfaces.msg import Decision
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import sys
import time
from car_interfaces.srv import GetInitPose, GetTargetPose, GetInitPoseRequest, GetTargetPoseRequest, BalesNumInCamera, BalesNumInCameraRequest,BalesNumInCameraResponse
from car_interfaces.srv import PlanTask, ControlTask,ControlTaskRequest, CameraTask, PositionTask, PositionTaskRequest, PositionTaskResponse, StopTask
from car_interfaces.srv import cloud_order, cloud_orderResponse
from car_interfaces.srv import TaskSts, TaskStsRequest
from car_interfaces.msg import pose
from car_interfaces.srv import FusionDetTask, FusionDetTaskResponse
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
import yaml

yaml_file='/home/nvidia/clamp_forklift_ws2/src/smach_fork/config/output.yaml'

rospack = rospkg.RosPack()
package_path = rospack.get_path('smach_fork')

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


    def send_task_sts_message(self, req):
        rospy.wait_for_service('task_sts_service')
        try:
            task_sts_service = rospy.ServiceProxy('task_sts_service', TaskSts)
            response = task_sts_service(req)
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def send_plan_task_service(self, request_value):
        rospy.loginfo("\033[31m 等待规划 server... \033[0m ")
        rospy.wait_for_service('plan_task')
        rospy.loginfo("\033[32m"+"规划server ready!"+"\033[0m")
        try:
            client = rospy.ServiceProxy("plan_task", PlanTask)
            resp = client(request_value)
            if resp.success:
                rospy.loginfo("规划反馈 successful")
            else:
                rospy.loginfo("Failed to plan_task: %s", resp.message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def send_control_task_service(self, request_value, limited_speed):
        rospy.loginfo("\033[31m 等待控制 server... \033[0m ")
        rospy.wait_for_service('control_task')
        rospy.loginfo("\033[32m"+"控制 server ready!"+"\033[0m")
        try:
            client = rospy.ServiceProxy("control_task", ControlTask)
            # 构建请求对象并填充数据
            req = ControlTaskRequest()
            req.data = request_value
            req.limited_speed = limited_speed
            # 发送请求并接收响应
            resp = client(req)
            if resp.success:
                rospy.loginfo("控制反馈 successful")
            else:
                rospy.loginfo("Failed to control_task: %s", resp.message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def send_camera_task_service(self, request_value):
        rospy.loginfo("\033[91m 等待相机 server... \033[0m ")
        rospy.wait_for_service('camera_task')
        rospy.loginfo("\033[32m"+"Sending camera_task server ready!"+"\033[0m")
        try:
            client = rospy.ServiceProxy("camera_task", CameraTask)
            resp = client(request_value)
            if resp.success:
                rospy.loginfo("相机反馈 successful")
            else:
                rospy.loginfo("Failed to camera_task: %s", resp.message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def send_stop_task_service(self, request_value):
        rospy.loginfo("\033[31m 等待停障 server... \033[0m ")
        rospy.wait_for_service('stop_task')
        rospy.loginfo("\033[32m"+"Sending stop_task server ready!"+"\033[0m")
        try:
            client = rospy.ServiceProxy("stop_task", StopTask)
            resp = client(request_value)
            if resp.success:
                rospy.loginfo("停障反馈 successful")
            else:
                rospy.loginfo("Failed to stop_task: %s", resp.message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def send_init_pose(self, point, quaternion):
        rospy.loginfo("\033[91m Waiting for Sending init pose server... \033[0m ")
        rospy.wait_for_service('get_init_pose')
        rospy.loginfo("\033[32m"+"Sending init pose server ready!"+"\033[0m")
        try:
            get_init_pose = rospy.ServiceProxy('get_init_pose', GetInitPose)

            request = GetInitPoseRequest()

            request.init_pose.pose.pose.position = Point(point.x, point.y, point.z)
            request.init_pose.pose.pose.orientation = Quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w)

            response = get_init_pose(request)
            if response.success:
                rospy.loginfo("Init pose sent successfully: %s", response.message)
            else:
                rospy.loginfo("Failed to send init pose: %s", response.message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def send_target_pose(self, point, quaternion):
        rospy.loginfo("\033[91m Waiting for Sending target pose server... \033[0m")
        rospy.wait_for_service('get_target_pose')
        rospy.loginfo("\033[32m"+"Sending target pose server ready!"+"\033[0m")
        try:
            get_target_pose = rospy.ServiceProxy('get_target_pose', GetTargetPose)
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = "map"
            
            pose_stamped.pose.position = Point(point.x, point.y, point.z)
            pose_stamped.pose.orientation = Quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w)
            
            response = get_target_pose(pose_stamped)

            if response.success:
                rospy.loginfo("Tar pose sent successfully: %s", response.message)
            else:
                rospy.loginfo("Failed to send tar pose: %s", response.message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)



    #=============================================================================
    #---------------------------lib-------------------------------
    #--------------------------------------------------------------------------
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




# 用于服务端的回调内容 接收来自客户端的请求并给出响应
class GlobalTaskInstance:
    def __init__(self):
        self.position_state = None
        self.bales_num_in_camera = None

    def handle_position_state(self, req):
        global position_
        rospy.loginfo("Request received for position update: x = %d", req.data)
        position_ = req.data
        
        self.position_state = req.data
        resp = PositionTaskResponse()
        resp.success = True
        resp.message = "Position state updated successfully."
        return resp

    def handle_bales_num_state(self, req):
        global bales_num_
        rospy.loginfo("Request received for bales num : x = %d", req.data)
        bales_num_ =  req.data
        self.bales_num_in_camera = req.data
        resp = BalesNumInCameraResponse()
        resp.success = True
        resp.message = "Bales number state updated successfully."
        return resp

    def odom_callback(self, msg):
        global current_x, current_y, current_yaw
        position = msg.pose.pose.position
        orientation_quat = msg.pose.pose.orientation
        _, _, current_yaw = tf.euler_from_quaternion([orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w])
        current_x = position.x
        current_y = position.y
        
    def handle_cloud_order(self,req):
        global delivery_point,task_id,shelf_name
        for task_info in req.task_info:
            # 目前云端只会有一个任务
            task_id = task_info.task_id
            # print(f'Received  task_id: {task_id}')
            x_ori = task_info.origin_shelf_job_position.position.x
            y_ori = task_info.origin_shelf_job_position.position.y
            z_ori = task_info.origin_shelf_job_position.position.z
            
            quaternion_ori = task_info.origin_shelf_job_position.orientation.z
            x_tar = task_info.destination_shelf_job_position.position.x
            y_tar = task_info.destination_shelf_job_position.position.y
            z_tar = task_info.destination_shelf_job_position.position.z
            yaw_tar = task_info.destination_shelf_job_position.orientation.z
            quaternion_tar = self.yaw_to_quaternion(yaw_tar)
            delivery_point = ((x_tar,y_tar,z_tar), (quaternion_tar.x, quaternion_tar.y, quaternion_tar.z,quaternion_tar.w))
            shelf_name = task_info.destination_shelf_area
            # rospy.loginfo(f"shelf_name: {shelf_name}")

        response = cloud_orderResponse()
        response.task_progress = 100
        return response  
        
    def camera_cotton_callback(self,req):
        global cotton_x, cotton_y, cotton_z, cotton_yaw, cotton_height, cotton_width
        global conveyer_belt_point
        
        rospy.loginfo("Request received for camera det: x = %f", req.x)
        rospy.loginfo("Request received for camera det: y = %f", req.y)
        rospy.loginfo("Request received for camera det: yaw = %f", req.yaw)
        cotton_x = req.x
        cotton_y = req.y
        cotton_z = req.z
        cotton_width = req.width
        cotton_height = req.height
        cotton_yaw = req.yaw
    
        quat = tf.quaternion_from_euler(0, 0, cotton_yaw)
        # print(quat)
        conveyer_belt_point = ((cotton_x, cotton_y, cotton_z), (quat[0], quat[1], quat[2], quat[3]))
        
        response = FusionDetTaskResponse()        
        response.success = True
        response.message = "Task completed successfully."
        return response


class ClampControlState(smach.State,SerManager):
    def __init__(self, target):
        smach.State.__init__(self, outcomes=['success'], io_keys=['conveyer_belt_point', 'observer_point', 'delivery_point'])
        SerManager.__init__(self)
        self.tar_width = target
        rospy.Subscriber('/clamp_state', Float64, self.clamp_state_callback)
        rospy.Subscriber('/ipc_state', Int8, self.ipc_state_callback)
        # 成员定义，然后再execute中初始化
        self.cur_width = 0
        self.start_time = None
        self.previous_changed_width = None
        self.last_check_time = None
        self.ipc_state = 0


    def clamp_state_callback(self, msg):
        self.cur_width = msg.data
        # self.changed_width = abs(self.tar_width - self.cur_width)

    def ipc_state_callback(self, msg):
        self.ipc_state = msg.data

    def execute(self, userdata):
        global num_work
        # self.send_control_task_service(0,0)
        # self.send_stop_task_service(0)
        # self.send_plan_task_service(0)
        start_time = time.time()
        specific_start_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(start_time))

        time.sleep(0.5)
        # 计算新的target:
        if (self.tar_width>0):
            self.tar_width = self.tar_width + self.cur_width
        else:
            self.tar_width = self.cur_width + self.tar_width


        rospy.loginfo_once("\033[32m"+'Executing state: ClampControlState'+"\033[0m")
        count = 0  # 初始化计数器，按照一定频率打印
        
        while True:
            # 先循环发布210才能进无人使能
            self.set_tar_width_pub.publish(self.tar_width)
            self.set_tar_height_pub_.publish(2) # 宽度控制的模式，2为高速升降模式 默认使用2 @@

            if (self.ipc_state == 1):
                # 循环发布1s->每隔1s检查宽度变化->变化范围小于10,认为已经到达目标
                # 100hz 每1s打印一次
                if self.start_time is None:
                    self.start_time = rospy.Time.now()
                self.set_tar_width_pub.publish(self.tar_width)
                self.set_tar_height_pub_.publish(2)

                if (rospy.Time.now() - self.start_time).to_sec() > 1.0:
                    current_time = rospy.Time.now()
                    if self.last_check_time is None or (current_time - self.last_check_time).to_sec() >= 1.0:
                        current_changed_width = abs(self.tar_width - self.cur_width)
                        if self.previous_changed_width is not None:
                            difference = abs(current_changed_width - self.previous_changed_width)
                            if difference < 10:
                                self.start_time = None
                                self.last_check_time = None
                                self.previous_changed_width = None
                                break
                            self.previous_changed_width = current_changed_width
                        else:
                            self.previous_changed_width = current_changed_width
                        self.last_check_time = current_time

                count += 1
                if count % 100 == 0:
                    rospy.loginfo(f" \033[33m cur_width:{abs(self.cur_width):.3f} \033[0m")
                    rospy.loginfo(f" \033[33m tar_width:{abs(self.tar_width):.3f} \033[0m")
            else:
                rospy.loginfo_once("\033[31m"+"Wait for auto_drive"+"\033[0m")
            rospy.Rate(100).sleep()

        end_time = time.time()
        elapsed_time = end_time - start_time
        specific_end_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(end_time))
        state_name = "SET_WIDTH"
        with open(csv_file_path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([state_name, start_time, end_time, specific_start_time, specific_end_time,
                             round(elapsed_time, 3)])
        rospy.loginfo_once("\033[32m"+"Width set successfully"+"\033[0m")
        self.start_time = None
        return 'success'



class UpdownControlState(smach.State,SerManager):
    def __init__(self, target):
        smach.State.__init__(self, outcomes=['success'], io_keys=['conveyer_belt_point', 'observer_point', 'delivery_point'])
        SerManager.__init__(self)
        self.tar_height = target

        rospy.Subscriber('/updown_state', Float64, self.updown_state_callback)
        rospy.Subscriber('/ipc_state', Int8, self.ipc_state_callback)

        self.updown_mode = 2

        self.cur_height = 0
        self.ipc_state = 0
        self.start_time = None
        self.previous_changed_height = None
        self.last_check_time = None

    def updown_state_callback(self, msg):
        self.cur_height = msg.data
        # self.changed_height = abs(self.tar_height - self.cur_height)

    def ipc_state_callback(self, msg):
        self.ipc_state = msg.data

    def execute(self, userdata):
        count = 0  # 初始化计数器
        start_time = time.time()
        specific_start_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(start_time))

        time.sleep(0.5)
        if (self.tar_height>0):
            self.tar_height = self.cur_height + self.tar_height
        else:
            self.tar_height = self.cur_height + self.tar_height

        rospy.loginfo_once("\033[32m"+'Executing state: UpdownControlState'+"\033[0m")
        while True:
            # 先循环发布210才能进无人使能
            self.set_tar_height_pub.publish(self.tar_height)
            self.set_tar_height_pub_.publish(self.updown_mode)

            if (self.ipc_state == 1):
                if self.start_time is None:
                    self.start_time = rospy.Time.now()

                self.set_tar_height_pub.publish(self.tar_height)
                self.set_tar_height_pub_.publish(self.updown_mode)

                if (rospy.Time.now() - self.start_time).to_sec() > 1.0:
                    current_time = rospy.Time.now()
                    if self.last_check_time is None or (current_time - self.last_check_time).to_sec() >= 1.0:
                        current_changed_height = abs(self.tar_height - self.cur_height)
                        if self.previous_changed_height is not None:
                            difference = abs(current_changed_height - self.previous_changed_height)
                            if difference < 10:
                                self.start_time = None
                                self.last_check_time = None
                                self.previous_changed_height = None
                                break
                            self.previous_changed_height = current_changed_height
                        else:
                            self.previous_changed_height = current_changed_height
                        self.last_check_time = current_time

                count += 1
                if count % 100 == 0: 
                    rospy.loginfo(f" \033[33m cur_height:{(self.cur_height):.3f} \033[0m")
                    rospy.loginfo(f" \033[33m tar_height:{(self.tar_height):.3f} \033[0m")
            else:
                rospy.loginfo_once("\033[31m"+"Wait For auto_drive"+"\033[0m")
            rospy.Rate(100).sleep()

        end_time = time.time()
        elapsed_time = end_time - start_time
        specific_end_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(end_time))
        state_name = "SET_HEIGHT"
        with open(csv_file_path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([state_name, start_time, end_time, specific_start_time, specific_end_time,
                             round(elapsed_time, 3)])

        rospy.loginfo_once("\033[32m"+"Height set successfully"+"\033[0m")
        return 'success'


# 完成状态
class Finish(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeed'], io_keys=['conveyer_belt_point', 'observer_point', 'delivery_point'])
    def execute(self, userdata):
        rospy.loginfo("\033[32m"+'Executing state: FINISH'+"\033[0m")
        return 'succeed'


class SteeringControlState(smach.State,SerManager):
    def __init__(self, angle=None, gear=None, speed=None):
        smach.State.__init__(self, outcomes=['success'], io_keys=['conveyer_belt_point', 'observer_point', 'delivery_point'])
        SerManager.__init__(self)
        # 初始化相关变量
        self.conveyer_belt_yaw = None
        self.current_yaw = None
        self.steering_state = None
        self.dest_gear=int(gear)
        self.adjust_steering_angle = 1.2

        rospy.Subscriber('/steering_state', Float64, self.steering_callback)

        self.throttle = speed

        self.conveyer_belt_point = 0
        self.delivery_point_mode = 0
        self.control_com = 5

        if self.throttle>0.8:
            self.control_com = 10
        
        self.angle_diff = 0
        self.init_yaw = None
        self.change_angle = True
        self.start_time = None
        
        self.goal_yaw = angle

    def steering_callback(self, msg):
        self.steering_state = msg.data


    def execute(self, userdata):
            global current_yaw, current_x, current_y
            start_time = time.time()
            specific_start_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(start_time))
            # 空的路径
            path_msg = Path()
            path_msg.header.frame_id = 'map'
            self.path_publisher.publish(path_msg)
            count = 0
            while(current_y==0):
                rospy.loginfo_once("\033[31m"+'Wait Odom'+"\033[0m")
            
            self.current_yaw = current_yaw
            if self.init_yaw is None:
                self.init_yaw = self.current_yaw

            goal_yaw = self.current_yaw + self.goal_yaw * math.pi / 180
            direction, self.need_diff = self.calculate_optimal_rotation(self.current_yaw * 180 / math.pi, goal_yaw * 180 / math.pi)
            print(direction, self.need_diff)
            # 1-顺时针 2-逆时针
            if(direction==1):
                if self.dest_gear==1: #倒车
                    adjust_steering_angle_ = -self.adjust_steering_angle
                elif self.dest_gear==3:
                    adjust_steering_angle_ = self.adjust_steering_angle
            if(direction==2):
                if self.dest_gear==1: #倒车
                    adjust_steering_angle_ = self.adjust_steering_angle
                elif self.dest_gear==3:
                    adjust_steering_angle_ = -self.adjust_steering_angle

            while True:
                self.current_yaw = current_yaw
                if self.steering_state is None:
                    rospy.loginfo_once("\033[31m"+'Wait steering'+"\033[0m")

                if self.steering_state is not None and self.current_yaw is not None:
                    # 1先把车轮转好
                    steering_diff = abs(adjust_steering_angle_ - self.steering_state)

                    if self.change_angle:
                        # 转完了只能执行一次self.change_angle
                        self.brake_pub.publish(0.0)
                        self.steering_pub.publish(adjust_steering_angle_)
                        throttle_cmd = Float64(0.0)
                        self.throttle_pub.publish(throttle_cmd)
                        gear_cmd = UInt8(self.dest_gear)
                        self.gear_pub.publish(gear_cmd)
                        if steering_diff < 0.1:
                            self.change_angle = False
                    else:
                        # 2累计转过的 yaw，如果大于等于 self.angle_diff 先把角度转回来，然后停止
                        if abs(self.calculate_angle(self.init_yaw, self.current_yaw)) <= (abs(self.need_diff)-self.control_com *math.pi/180): # 少转一点，补偿90度
                        # if abs(self.calculate_angle(self.init_yaw, self.current_yaw)) <= (abs(self.need_diff)): # 少转一点，补偿90度
                            steering_cmd = Float64(adjust_steering_angle_)
                            self.steering_pub.publish(steering_cmd)
                            self.brake_pub.publish(0.0)
                            throttle_cmd = Float64(self.throttle)
                            self.throttle_pub.publish(throttle_cmd)
                            gear_cmd = UInt8(self.dest_gear)
                            self.gear_pub.publish(gear_cmd)
                            
                        else:
                            steering_cmd = Float64(0.0)
                            self.steering_pub.publish(0.0)
                            throttle_cmd = Float64(0.0)
                            self.brake_pub.publish(4.0)
                            self.throttle_pub.publish(0)
                            gear_cmd = UInt8(2)
                            self.gear_pub.publish(gear_cmd)
                            if self.start_time is None:
                                self.start_time = rospy.Time.now()
                            if (rospy.Time.now() - self.start_time).to_sec() > 0.5:
                                rospy.loginfo("\033[32m"+"last diff: " + str(abs(self.calculate_angle(self.init_yaw, self.current_yaw))) + "\033[0m")
                                break

                    count += 1
                    if count % 100 == 0:
                        rospy.loginfo("\033[32m"+"NOW TURN: " + f"{abs(self.calculate_angle(self.init_yaw, self.current_yaw)):.3f}" + "\033[0m")
                        rospy.loginfo("\033[32m"+"NEED TURN: " + f"{abs(self.need_diff):.3f}" + "\033[0m")
                        rospy.loginfo("\033[32m"+"Wheel Angle pub: " + f"{adjust_steering_angle_:.3f}" + "\033[0m")
                rospy.Rate(100).sleep()

            end_time = time.time()
            elapsed_time = end_time - start_time
            specific_end_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(end_time))
            state_name = f"{self.__class__.__name__}"
            with open(csv_file_path, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(
                    [state_name, start_time, end_time, specific_start_time, specific_end_time, round(elapsed_time, 3)])

            if not self.brake_enable: 
                rospy.loginfo_once("\033[33m"+'Adjust yaw success!!!'+"\033[0m")
                self.change_angle = True
                self.init_yaw = None
                self.start_time = None
                self.steering_state = None
                return 'success'


class MotionControlState(smach.State, SerManager):
    def __init__(self, distance, speed):
        smach.State.__init__(self, outcomes=['success'], io_keys=['conveyer_belt_point', 'observer_point', 'delivery_point'])
        SerManager.__init__(self)
        self.target_distance = distance
        self.Throttle = speed
        
        rospy.Subscriber('/ipc_state', Int8, self.ipc_state_callback)

        self.ipc_state = 0
        self.new_point_reindex = []

    def ipc_state_callback(self, msg):
        self.ipc_state = msg.data


    def execute(self, userdata):
        global num_work
        global position_, bales_num_
        global current_x, current_y, current_yaw, delivery_point, work_direction
        global num_work_max, num_work_height,work_direction
        global conveyer_belt_point, conveyer_belt_point_camera

        start_time = time.time()
        specific_start_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(start_time))

        rospy.loginfo_once("\033[32m"+'Executing state: MotionControlState'+"\033[0m")
        rospy.loginfo("\033[32m"+"下发全局任务"+"\033[0m")
        self.task = 1
        
        self.send_plan_task_service(self.task)
        self.send_control_task_service(self.task,0)
        # self.send_stop_task_service(self.task) # 0-不响应 1-响应停障
        rospy.loginfo("\033[32m"+"任务下发成功"+"\033[0m")

        time.sleep(0.5)
        current_pos_point = self.process_point(current_x, current_y, current_yaw)
        
        # 前进
        if (self.target_distance>0):
            cut_distance1, cut_distance2 = 0, 0
            if self.target_distance <= 0.5:
                # front_point_observe = self.calculate_reverse_point(current_pos_point, -self.target_distance + cut_distance1)
                front_front_point_observe = self.calculate_reverse_point(current_pos_point, -self.target_distance)
                rospy.loginfo("\033[32m"+"下发目标点位"+"\033[0m")
                goal_point, goal_quaternion = front_front_point_observe
                goal_pose = Point(goal_point[0], goal_point[1], goal_point[2])
                goal_orientation = Quaternion(goal_quaternion[0], goal_quaternion[1], goal_quaternion[2], goal_quaternion[3])
                self.send_target_pose(goal_pose, goal_orientation)
                rospy.loginfo("\033[32m"+"目标点位发布成功"+"\033[0m")
            else:    
                if self.target_distance > 8:
                    cut_distance1 = 4
                    # cut_distance2 = 4
                if self.target_distance > 6:
                    cut_distance1 = 3
                    # cut_distance2 = 3
                elif self.target_distance > 4:
                    cut_distance1 = 2.2
                    # cut_distance2 = 2
                elif self.target_distance > 2:
                    cut_distance1 = 1.4
                    # cut_distance2 = 0.7
                elif self.target_distance > 1.5:
                    cut_distance1 = 0.7
                    # cut_distance2 = 0
                elif self.target_distance > 0.5:
                    cut_distance1 = 0.2
                    # cut_distance2 = 0

                front_point_observe = self.calculate_reverse_point(current_pos_point, -self.target_distance + cut_distance1)
                front_front_point_observe = self.calculate_reverse_point(current_pos_point, -self.target_distance)
                self.new_point_reindex.append(front_point_observe)
                for point_data, quaternion_data in self.new_point_reindex:
                    point = Point(*point_data)
                    quaternion = Quaternion(*quaternion_data)
                    self.send_init_pose(point, quaternion)
                    rospy.loginfo("\033[32m"+"途经点位发布成功"+"\033[0m")
                rospy.loginfo("\033[32m"+"下发目标点位"+"\033[0m")
                goal_point, goal_quaternion = front_front_point_observe
                goal_pose = Point(goal_point[0], goal_point[1], goal_point[2])
                goal_orientation = Quaternion(goal_quaternion[0], goal_quaternion[1], goal_quaternion[2], goal_quaternion[3])
                self.send_target_pose(goal_pose, goal_orientation)
                rospy.loginfo("\033[32m"+"目标点位发布成功"+"\033[0m")

        # 倒车
        elif (self.target_distance<0):
            cut_distance1, cut_distance2 = 0, 0
            if self.target_distance <= 0.5:
                # front_point_observe = self.calculate_reverse_point(current_pos_point, -self.target_distance + cut_distance1)
                front_front_point_observe = self.calculate_reverse_point(current_pos_point, self.target_distance)
                rospy.loginfo("\033[32m"+"下发目标点位"+"\033[0m")
                goal_point, goal_quaternion = front_front_point_observe
                goal_pose = Point(goal_point[0], goal_point[1], goal_point[2])
                goal_orientation = Quaternion(goal_quaternion[0], goal_quaternion[1], goal_quaternion[2], goal_quaternion[3])
                self.send_target_pose(goal_pose, goal_orientation)
                rospy.loginfo("\033[32m"+"目标点位发布成功"+"\033[0m")
            else:    
                if self.target_distance < -8:
                    cut_distance1 = 4
                    # cut_distance2 = 4
                if self.target_distance < -6:
                    cut_distance1 = 3
                    # cut_distance2 = 3
                elif self.target_distance < -4:
                    cut_distance1 = 2.2
                    # cut_distance2 = 2
                elif self.target_distance < -2:
                    cut_distance1 = 1.4
                    # cut_distance2 = 0.7
                elif self.target_distance < -1.5:
                    cut_distance1 = 0.7
                    # cut_distance2 = 0
                elif self.target_distance < -0.5:
                    cut_distance1 = 0.2
                    # cut_distance2 = 0

                front_point_observe = self.calculate_reverse_point(current_pos_point, self.target_distance + cut_distance1)
                front_front_point_observe = self.calculate_reverse_point(current_pos_point, self.target_distance)
                self.new_point_reindex.append(front_point_observe)
                for point_data, quaternion_data in self.new_point_reindex:
                    point = Point(*point_data)
                    quaternion = Quaternion(*quaternion_data)
                    self.send_init_pose(point, quaternion)
                    rospy.loginfo("\033[32m"+"途经点位发布成功"+"\033[0m")

                rospy.loginfo("\033[32m"+"下发目标点位"+"\033[0m")
                goal_point, goal_quaternion = front_front_point_observe
                goal_pose = Point(goal_point[0], goal_point[1], goal_point[2])
                goal_orientation = Quaternion(goal_quaternion[0], goal_quaternion[1], goal_quaternion[2], goal_quaternion[3])
                self.send_target_pose(goal_pose, goal_orientation)
                rospy.loginfo("\033[32m"+"目标点位发布成功"+"\033[0m")

        count = 0
        while position_ == 0 and not self.brake_enable:
            if (self.target_distance==0):
                break
            # 空的路径
            path_msg = Path()
            path_msg.header.frame_id = 'map'
            self.path_publisher.publish(path_msg)
            if(position_==1):
                break
            count += 1
            if count % 100 == 0:
                rospy.loginfo("\033[32m" + f"Go to global endpoint: {position_}"+ "\033[0m")
            rospy.Rate(100).sleep()

        self.send_plan_task_service(0)
        self.send_control_task_service(0,0)
        # self.send_stop_task_service(0) # 0-不响应 1-响应停障
        self.new_point_delivery = []
        self.new_point_reindex = []

        end_time = time.time()
        elapsed_time = end_time - start_time
        specific_end_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(end_time))
        state_name = f"{self.__class__.__name__}"
        with open(csv_file_path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(
                [state_name, start_time, end_time, specific_start_time, specific_end_time, round(elapsed_time, 3)])

        # 到达终点->开始夹取
        if not self.brake_enable: 
            position_ = 0
            rospy.loginfo_once("\033[32m"+'Position reached, switching to local_planning'+"\033[0m")
            return 'success'


class Init(smach.State, SerManager):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'], io_keys=['conveyer_belt_point', 'observer_point', 'delivery_point'] )
        SerManager.__init__(self)
    def execute(self, userdata):

        self.send_control_task_service(0, 0)
        # self.send_camera_task_service(0)
        self.send_plan_task_service(0)
        # self.send_stop_task_service(0)

        start_time = time.time()
        return 'success'

def create_state_machine():
# def create_state_machine():
    global delivery_point, conveyer_belt_point, observer_point
    global work_direction, num_work, num_work_max, num_work_height

    #===========================Param==================================
    #------------------------------------------------------------------
    test_high_speed_control = rospy.get_param('/real/test_high_speed_control', True) 

    # 从参数服务器中获取参数：
    use_sim = rospy.get_param('/use_sim', False)
    use_camera_det = rospy.get_param('/use_camera_det', True)

    single_cargo_height_sim = rospy.get_param('/sim/single_cargo_height', 0.1)
    height_between_two_cargos_sim = rospy.get_param('/sim/height_between_two_cargos', 0.5)
    two_cargos_height_sim = rospy.get_param('/sim/two_cargos_height', 0.8)
    above_two_cargos_height_sim = rospy.get_param('/sim/above_two_cargos_height', 0.8)
    clamping_width_sim = rospy.get_param('/sim/clamping_width', 0.05)
    open_width_sim = rospy.get_param('/sim/open_width', 0.6)

    # 原地转向的车轮转角
    adjust_steering_angle = rospy.get_param("/smach/adjust_steering_angle", 1.22)
    adjust_steering_angle_sim = rospy.get_param("/smach/adjust_steering_angle_sim", 1.1)

    # 一个棉包的长度：
    single_cargo_length = rospy.get_param('/real/single_cargo_length', 1.34)
    # 一个货物的高度
    single_cargo_height = rospy.get_param('/real/single_cargo_height', 40)
    # 一个半
    single_cargo_height_half = rospy.get_param('/real/single_cargo_height_half', 85)
    # 两个之间的货物高度
    height_between_two_cargos = rospy.get_param('/real/height_between_two_cargos', 234)
    # 两个货物高度
    two_cargos_height = rospy.get_param('/real/two_cargos_height', 514)
    # 高于两个货物高度
    above_two_cargos_height = rospy.get_param('/real/above_two_cargos_height', 736)
    # 放货的高度
    above_two_cargos_height_max = rospy.get_param('/real/above_two_cargos_height_max', 1125)
    # 夹取宽度
    clamping_width = rospy.get_param('/real/clamping_width', 40)
    # 张开宽度
    open_width = rospy.get_param('/real/open_width', 400)
    # 放货的张开宽度
    open_width_pickoff = rospy.get_param('/real/open_width_pickoff', 400)
    # 最终放货高度
    pickoff_height_ = rospy.get_param('/real/pickoff_height_', 35)
    open_width_ = rospy.get_param('/real/open_width_', 380)
    
    use_cloud = rospy.get_param('/real/use_cloud', True)
    
    if not use_camera_det:
        # 传送带位置
        conveyer_belt_point = rospy.get_param('conveyer_belt_point', ((24.3610,12.3270,0.5682),(0.0261,-0.0068,-0.9996,0.0003)))
    
    # 直接判断放货： 当前货架的放货方向
    if not use_cloud:
        delivery_point = rospy.get_param('delivery_point', ((24.9794, 0.9729, 0.0), (0.0, 0.0, 1.0000, 0.0063)))
        work_direction = rospy.get_param('/real/work_direction', 2)
        num_work = rospy.get_param('/real/num_work', 0)
        # num_work = 0
        num_work_max = rospy.get_param('/real/num_work_max', 36) 
        num_work_height = rospy.get_param('/real/num_work_height', 0)
    # num_work = 0
    # 观察点位置
    # observer_point = rospy.get_param('observer_point', ((27.3400,6.7202,0.0000),(0, 0, 0.7964, 0.6048)))
    ##############################
    #===========================Param==================================
    #------------------------------------------------------------------



    #------------------------------------------------------------------
    #===========================Serve==================================
    global_task_instance = GlobalTaskInstance()
    # # 创建服务 都是服务端
    # 位置信息，判断是否已经到达终点
    position_service = rospy.Service('position_service', PositionTask, global_task_instance.handle_position_state)
    bales_num_in_camera_service = rospy.Service('bales_num_in_camera_service', BalesNumInCamera, global_task_instance.handle_bales_num_state)
    odom_sub = rospy.Subscriber("/dis_odom",Odometry,  global_task_instance.odom_callback)
    
    handle_cloud_order_service = rospy.Service('cloud_order_service', cloud_order, global_task_instance.handle_cloud_order)
    camera_service = rospy.Service('fusion_det_task', FusionDetTask, global_task_instance.camera_cotton_callback)
    #===========================Serve==================================
    #------------------------------------------------------------------


    #===========================Smach==================================
    # 创建一个状态机
    sm = smach.StateMachine(outcomes=['succeed', 'aborted', 'preempted'])

    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)

    state_class_map = {
        'motion_control': MotionControlState,
        'steering_control': SteeringControlState,
        'clamp_control': ClampControlState,
        'updown_control': UpdownControlState,
    }

    state_names = []  # 用于存储所有状态的名字，以确保正确设置转移条件

    # 首先构建所有状态的名称列表
    for idx, item in enumerate(data, start=1):
        control_type = next(iter(item))
        unique_state_name = f"{control_type.upper()}_{idx}"
        state_names.append(unique_state_name)

    with sm:
        for idx, item in enumerate(data, start=1):
            control_type = next(iter(item))
            params = item[control_type]
            print(params)

            # 根据控制类型实例化对应的状态类，并传递必要的参数
            state_instance = state_class_map[control_type](**params)

            # 构建当前状态名称
            current_state_name = f"{control_type.upper()}_{idx}"

            # 定义转移规则：除了最后一个状态外，所有状态都转到下一个状态
            next_idx = idx + 1
            if next_idx <= len(state_names):
                next_state_name = state_names[next_idx - 1]
            else:
                next_state_name = 'succeed'

            transitions = {'success': next_state_name}

            # 添加状态到状态机
            smach.StateMachine.add(current_state_name, state_instance, transitions=transitions)

    return sm