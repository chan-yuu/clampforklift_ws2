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
        rospy.loginfo("\033[31m 等待规划server... \033[0m ")
        rospy.wait_for_service('plan_task')
        rospy.loginfo("\033[33m"+"规划server 已连接!"+"\033[0m")
        try:
            client = rospy.ServiceProxy("plan_task", PlanTask)
            resp = client(request_value)
            if resp.success:
                pass
                # rospy.loginfo("规划反馈 successful")
            else:
                rospy.loginfo("Failed to plan_task: %s", resp.message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def send_control_task_service(self, request_value, limited_speed):
        rospy.loginfo("\033[31m 等待控制server... \033[0m ")
        rospy.wait_for_service('control_task')
        rospy.loginfo("\033[33m"+"控制server 已连接!"+"\033[0m")
        try:
            client = rospy.ServiceProxy("control_task", ControlTask)
            # 构建请求对象并填充数据
            req = ControlTaskRequest()
            req.data = request_value
            req.limited_speed = limited_speed
            # 发送请求并接收响应
            resp = client(req)
            if resp.success:
                pass
                # rospy.loginfo("控制反馈 successful")
            else:
                rospy.loginfo("Failed to control_task: %s", resp.message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def send_camera_task_service(self, request_value):
        rospy.loginfo("\033[31m 等待相机server... \033[0m ")
        rospy.wait_for_service('camera_task')
        rospy.loginfo("\033[32m"+"Sending camera_task server ready!"+"\033[0m")
        try:
            client = rospy.ServiceProxy("camera_task", CameraTask)
            resp = client(request_value)
            if resp.success:
                pass
                # rospy.loginfo("相机反馈 successful")
            else:
                rospy.loginfo("Failed to camera_task: %s", resp.message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def send_stop_task_service(self, request_value):
        rospy.loginfo("\033[31m 等待停障server... \033[0m ")
        rospy.wait_for_service('stop_task')
        rospy.loginfo("\033[32m"+"Sending stop_task server ready!"+"\033[0m")
        try:
            client = rospy.ServiceProxy("stop_task", StopTask)
            resp = client(request_value)
            if resp.success:
                pass
                # rospy.loginfo("停障反馈 successful")
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


class SetWidth(smach.State,SerManager):
    def __init__(self, tar_width):
        smach.State.__init__(self, outcomes=['finish'])
        SerManager.__init__(self)
        self.tar_width = tar_width # 输入宽度
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
        self.changed_width = abs(self.tar_width - self.cur_width)

    def ipc_state_callback(self, msg):
        self.ipc_state = msg.data

    def execute(self, userdata):
        global num_work
        # self.send_control_task_service(0,0)
        # self.send_stop_task_service(0)
        # self.send_plan_task_service(0)
        start_time = time.time()
        specific_start_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(start_time))

        rospy.loginfo_once("\033[32m"+'Executing state: SET_WIDTH'+"\033[0m")
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
        return 'finish'



class SetHeight(smach.State,SerManager):
    def __init__(self, tar_height,updown_mode):
        smach.State.__init__(self, outcomes=['finish'])
        SerManager.__init__(self)
        self.tar_height = tar_height

        rospy.Subscriber('/updown_state', Float64, self.updown_state_callback)
        rospy.Subscriber('/ipc_state', Int8, self.ipc_state_callback)

        self.updown_mode = updown_mode

        self.cur_height = 0
        self.ipc_state = 0
        self.start_time = None
        self.previous_changed_height = None
        self.last_check_time = None

    def updown_state_callback(self, msg):
        self.cur_height = msg.data
        self.changed_height = abs(self.tar_height - self.cur_height)

    def ipc_state_callback(self, msg):
        self.ipc_state = msg.data

    def execute(self, userdata):
        count = 0  # 初始化计数器
        start_time = time.time()
        specific_start_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(start_time))


        rospy.loginfo_once("\033[32m"+'Executing state: SET_HEIGHT'+"\033[0m")
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
        return 'finish'



class SetHeight_Delay(smach.State,SerManager):
    
    def __init__(self, tar_height,updown_mode, set_min_distance):
        smach.State.__init__(self, outcomes=['finish'])
        SerManager.__init__(self)
        self.tar_height = tar_height
        self.updown_mode = 2

        rospy.Subscriber('/updown_state', Float64, self.updown_state_callback)
        rospy.Subscriber('/ipc_state', Int8, self.ipc_state_callback)
        self.updown_mode = updown_mode

        self.cur_height = 0
        self.ipc_state = 0
        self.start_time = None
        self.previous_changed_height = None
        self.last_check_time = None

        self.set_min_distance = set_min_distance

    def updown_state_callback(self, msg):
        self.cur_height = msg.data
        self.changed_height = abs(self.tar_height - self.cur_height)

    def ipc_state_callback(self, msg):
        self.ipc_state = msg.data

    def execute(self, userdata):
        global move_distance
        
        count = 0  # 初始化计数器
        start_time = time.time()
        specific_start_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(start_time))


        rospy.loginfo_once("\033[32m"+'Executing state: SET_HEIGHT_DELAY'+"\033[0m")
        while True:
            if move_distance==0:
                continue
            
            if (move_distance<self.set_min_distance):
                rospy.loginfo_once("\033[32m"+'未达到下降时机'+"\033[0m")
                continue
            # print(1111)
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
        return 'finish'



class SetHeight_Delay_Sim(smach.State):
    def __init__(self, tar_height, set_min_distance):
        smach.State.__init__(self, outcomes=['finish'])
        self.tar_height = tar_height
        self.pub_left_vertical = rospy.Publisher('/fork_left_horizontal_to_fork_left_vertical_controller/command', Float64, queue_size=10)
        self.pub_right_vertical = rospy.Publisher('/fork_right_horizontal_to_fork_right_vertical_controller/command', Float64, queue_size=10)
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        self.cur_height = 0.0  # 初始化当前高度
        self.start_time = None
        self.previous_changed_height = None
        self.last_check_time = None
        self.set_min_distance = set_min_distance

    def joint_state_callback(self, msg):
        self.cur_height = msg.position[msg.name.index('fork_left_horizontal_to_fork_left_vertical')]  # 替换为实际关节名称
        self.changed_height = abs(self.tar_height - self.cur_height)

    def execute(self, userdata):
        global move_distance
        start_time = time.time()
        specific_start_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(start_time))
        count = 0
        rospy.loginfo_once("\033[32m"+'Executing state: SET_HEIGHT_SIM'+"\033[0m")
        
        while True:
            if move_distance==0:
                continue
            
            if (move_distance<self.set_min_distance):
                rospy.loginfo_once("\033[32m"+'未达到下降时机'+"\033[0m")
                continue

            if self.start_time is None:
                self.start_time = rospy.Time.now()
            self.pub_left_vertical.publish(Float64(self.tar_height))
            self.pub_right_vertical.publish(Float64(self.tar_height))

            if (rospy.Time.now() - self.start_time).to_sec() > 1:
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
            
            rospy.Rate(100).sleep()

        rospy.loginfo_once("\033[32m"+"Height set successfully"+"\033[0m")

        end_time = time.time()
        elapsed_time = end_time - start_time
        specific_end_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(end_time))
        state_name = f"{self.__class__.__name__}"
        with open(csv_file_path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(
                [state_name, start_time, end_time, specific_start_time, specific_end_time, round(elapsed_time, 3)])

        return 'finish'


class SetFY(smach.State, SerManager):
    def __init__(self, tar_fy):
        smach.State.__init__(self, outcomes=['finish'])
        SerManager.__init__(self)

        rospy.Subscriber('/updown_state', Float64, self.updown_state_callback)
        rospy.Subscriber('/ipc_state', Int8, self.ipc_state_callback)

        self.cur_fy = 0
        self.tar_fy=tar_fy
        self.ipc_state = 0
        self.start_time = None
        self.previous_changed_fy = None
        self.last_check_time = None

    def updown_state_callback(self, msg):
        self.cur_fy = msg.data
        self.changed_fy = abs(self.tar_fy - self.cur_fy)

    def ipc_state_callback(self, msg):
        self.ipc_state = msg.data

    def execute(self, userdata):
        start_time = time.time()
        specific_start_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(start_time))
        count = 0
        rospy.loginfo_once("\033[32m"+'Executing state: SET_FY'+"\033[0m")
        while True:
            self.set_tar_fy_pub.publish(self.tar_fy)

            if (self.ipc_state == 1):
                if self.start_time is None:
                    self.start_time = rospy.Time.now()
                self.set_tar_fy_pub.publish(self.tar_fy)

                if (rospy.Time.now() - self.start_time).to_sec() > 1.0:
                    current_time = rospy.Time.now()
                    if self.last_check_time is None or (current_time - self.last_check_time).to_sec() >= 1.0:
                        current_changed_fy = abs(self.tar_fy - self.cur_fy)
                        if self.previous_changed_fy is not None:
                            difference = abs(current_changed_fy - self.previous_changed_fy)
                            if difference < 10:
                                self.start_time = None
                                self.last_check_time = None
                                self.previous_changed_fy = None
                                break
                            self.previous_changed_fy = current_changed_fy
                        else:
                            self.previous_changed_fy = current_changed_fy
                        self.last_check_time = current_time

                count += 1
                if count % 100 == 0: 
                    rospy.loginfo(f" \033[33m cur_fy:{(self.cur_fy):.3f} \033[0m")
                    rospy.loginfo(f" \033[33m tar_fy:{(self.tar_fy):.3f} \033[0m")
            else:
                rospy.loginfo_once("\033[31m"+"Wait For auto_drive"+"\033[0m")
            rospy.Rate(100).sleep()

        end_time = time.time()
        elapsed_time = end_time - start_time
        specific_end_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(end_time))
        state_name = "SetFY"
        with open(csv_file_path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([state_name, start_time, end_time, specific_start_time, specific_end_time,
                             round(elapsed_time, 3)])

        rospy.loginfo_once("\033[32m"+"FY set successfully"+"\033[0m")
        return 'finish'


class HandleCloudOrder(smach.State, SerManager):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finish'],io_keys=['delivery_point'])
        SerManager.__init__(self)

        self.ori_point = None
        self.start_time = None
        self.ori_quaternion = None
        self.tar_point = None
        self.tar_quaternion = None
        self.delivery_point = delivery_point

        
    def execute(self, userdata):
        global delivery_point, work_direction
        global num_work,num_work_max,num_work_height

        start_time = time.time()
        specific_start_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(start_time))

        rospy.loginfo("\033[32m"+'Executing state: HANDLE_CLOUD_ORDER'+"\033[0m")
        self.task = 0
        self.send_plan_task_service(self.task)
        self.send_control_task_service(self.task,0)
        # self.send_stop_task_service(self.task) # 0-不响应 1-响应停障
        
        # 测试使用
        delivery_point = ((-8.5285,-26.7638,0.0000),(0.0000,0.0000,-0.6863,0.7273))

        if self.start_time is None:
            self.start_time = rospy.Time.now()
        while True:
            if (rospy.Time.now() - self.start_time).to_sec() < 0.5:
                steering_cmd = Float64(0.0)
                self.steering_pub.publish(steering_cmd)
                throttle_cmd = Float64(0.0)
                self.throttle_pub.publish(throttle_cmd)
                gear_cmd = UInt8(2)
                self.gear_pub.publish(gear_cmd)
                brake_cmd = Float64(0.0)
                self.brake_pub.publish(brake_cmd)
                rospy.loginfo_once("\033[32m Gear Init Success \033[0m")
            else:
                break
            rospy.Rate(100).sleep()

        while delivery_point == ():
            rospy.loginfo_once("\033[91m 等待云端任务... \033[0m")
            rospy.sleep(1)

        rospy.loginfo("\033[32m"+'收到云端任务'+"\033[0m")
        rospy.loginfo("\033[33m"+f'放货点:{delivery_point}'+"\033[0m")

        self.delivery_point = delivery_point
        userdata.delivery_point = delivery_point
        if shelf_name:
            num_work = int(str(shelf_name)[-1]) - 1
            work_direction = ord(shelf_name[0]) % 2 + 1
            num_work_height = int(str(shelf_name)[2]) -1
            rospy.loginfo(f'num_work:{num_work}  work_direction:{work_direction}  num_work_height:{num_work_height}   ')
        self.start_time = None
        
        end_time = time.time()
        elapsed_time = end_time - start_time
        specific_end_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(end_time))
        state_name = f"{self.__class__.__name__}"
        with open(csv_file_path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([state_name, start_time, end_time, specific_start_time, specific_end_time,
                             round(elapsed_time, 3)])
        return 'finish'


# 高速循迹测试
class LidarObstacle(smach.State, SerManager):
    def __init__(self, lidar_obs=None):
        smach.State.__init__(self, outcomes=['finish'], io_keys=['conveyer_belt_point', 'observer_point', 'delivery_point'])
        SerManager.__init__(self)
        # self.points = points if points is not None else []
                
        self.start_time = None
        self.new_point_reindex = []
        self.lidar_obs = lidar_obs # 1-前进 2-倒车


    def execute(self, userdata):
        global position_
        global cotton_4_num
 
        userdata.conveyer_belt_point = None
        userdata.observer_point = None
        userdata.delivery_point = None

        start_time = time.time()
        specific_start_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(start_time))

        rospy.loginfo_once("\033[32m"+'Executing state: HighSpeedState'+"\033[0m")
        rospy.loginfo("\033[32m"+"下发全局任务"+"\033[0m")
        self.task = 1
        
        self.send_plan_task_service(self.task)
        self.send_control_task_service(self.task,0)
        self.send_stop_task_service(self.lidar_obs) # 0-不响应 1-响应停障
        rospy.loginfo("\033[32m"+"任务下发成功"+"\033[0m")

        # 目标点，采集一下 也是要途径的点，所以起点可以稍微往后！

        point_pose = ((-7.0469,-4.1680,0.0000),(0.0000,0.0000,0.0156,0.9999))
        point_pose2 = self.calculate_reverse_point(point_pose, 6)
        point_pose3 = self.calculate_reverse_point(point_pose, -21)

        point_pose4=((15.2225,-5.0968,0.0000),(0.0000,0.0000,-0.7132,0.7009))
        point_pose5 = self.calculate_reverse_point(point_pose4, -16.5)

        point_pose6=((13.8742,-23.5500,0.0000),(0.0000,0.0000,1.0000,0.0061))
        point_pose7 = self.calculate_reverse_point(point_pose6, -18-cotton_4_num*1.5)


        # point_pose = ((-9.0137,-2.1283,0.0000),(0.0000,0.0000,0.0020,1.0000))
        # point_pose2 = ((12.0966,-1.4156,0.0000),(0.0000,0.0000,-0.0185,0.9998))
        # point_pose3 = ((15.3279,-3.8193,0.0000),(0.0000,0.0000,-0.7182,0.6959))
        # point_pose4 = ((15.1449,-19.7544,0.0000),(0.0000,0.0000,-0.7139,0.7003))

        # # init2:((10.7145,-4.3899,0.0000),(0.0000,0.0000,0.0014,1.0000))
        # # init3:((15.2467,-7.5660,0.0000),(0.0000,0.0000,-0.6996,0.7145))
        # # init4:((15.7374,-18.7177,0.0000),(0.0000,0.0000,-0.6860,0.7276))

        # # 放货
        # point_pose5 = ((-10.5496,-26.8690,0.0000),(0.0000,0.0000,-0.7012,0.7130))
        # point_pose6 = self.calculate_reverse_point(point_pose5, 3)
        # point_pose7 = self.calculate_new_point(point_pose6, 2, 1.5)
        # point_pose8 = self.calculate_new_point_rotation(point_pose7, 1)
        # point_pose9 = self.calculate_reverse_point(point_pose8, 8)
        # point_pose10 = self.calculate_reverse_point(point_pose8, 18)

        self.new_point_reindex.append(point_pose2)
        self.new_point_reindex.append(point_pose)
        self.new_point_reindex.append(point_pose3)
        self.new_point_reindex.append(point_pose4)
        self.new_point_reindex.append(point_pose5)
        self.new_point_reindex.append(point_pose6)
        # self.new_point_reindex.append(point_pose8)


        # # 前进15m
        # self.new_point_reindex.append(point_pose)
        # point_pose = self.calculate_reverse_point(point_pose, -24)
        # self.new_point_reindex.append(point_pose)

        # point_pose = self.calculate_reverse_point(point_pose, -0.6)
        # point_pose = self.calculate_new_point(point_pose, 1, 0.6)
        # # 旋转90°
        # point_pose = self.calculate_new_point_rotation(point_pose, 1)
        # self.new_point_reindex.append(point_pose)
        # point_pose = self.calculate_reverse_point(point_pose, -15)


        for point_data, quaternion_data in self.new_point_reindex:
            point = Point(*point_data)
            quaternion = Quaternion(*quaternion_data)
            self.send_init_pose(point, quaternion)
            rospy.loginfo("\033[32m"+"途经点位发布成功"+"\033[0m")

        rospy.loginfo("\033[32m"+"下发目标点位"+"\033[0m")
        goal_point, goal_quaternion = point_pose7
        goal_pose = Point(goal_point[0], goal_point[1], goal_point[2])
        goal_orientation = Quaternion(goal_quaternion[0], goal_quaternion[1], goal_quaternion[2], goal_quaternion[3])
        self.send_target_pose(goal_pose, goal_orientation)
        rospy.loginfo("\033[32m"+"目标点位发布成功"+"\033[0m")

        count = 0
        while position_ == 0 and not self.brake_enable:
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
        state_name = "HighSpeedState"
        with open(csv_file_path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([state_name, start_time, end_time, specific_start_time, specific_end_time,
                             round(elapsed_time, 3)])
            
        # 到达终点->开始夹取
        if not self.brake_enable: 
            position_ = 0
            self.start_time = None
            conveyer_belt_point = None
            rospy.loginfo_once("\033[32m"+'Position reached, switching to local_planning'+"\033[0m")
            return 'finish'
        # 如果有刹车
        elif self.brake_enable:
            position_ = 0
            conveyer_belt_point = None
            self.start_time = None
            rospy.loginfo_once('AEB')
            userdata.last_state = 'MOVING_TO_PICKUP_LOCATION'
            return 'braking'


# 高速循迹测试
class HighSpeedState(smach.State, SerManager):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finish'], io_keys=['conveyer_belt_point', 'observer_point', 'delivery_point'])
        SerManager.__init__(self)
        # self.points = points if points is not None else []
                
        self.start_time = None
        self.new_point_reindex = []


    def execute(self, userdata):
        global position_
 
        userdata.conveyer_belt_point = None
        userdata.observer_point = None
        userdata.delivery_point = None

        start_time = time.time()
        specific_start_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(start_time))

        rospy.loginfo_once("\033[32m"+'Executing state: HighSpeedState'+"\033[0m")
        rospy.loginfo("\033[32m"+"下发全局任务"+"\033[0m")
        self.task = 1
        
        self.send_plan_task_service(self.task)
        self.send_control_task_service(self.task,0)
        # self.send_stop_task_service(self.task) # 0-不响应 1-响应停障
        rospy.loginfo("\033[32m"+"任务下发成功"+"\033[0m")


        # 目标点，采集一下 也是要途径的点，所以起点可以稍微往后！
        point_pose = ((2.1218, -4.2735, 0), (0.0005,-0.0118,0.9999,-0.0023))
        # 前进15m
        self.new_point_reindex.append(point_pose)
        point_pose = self.calculate_reverse_point(point_pose, -24)
        self.new_point_reindex.append(point_pose)

        point_pose = self.calculate_reverse_point(point_pose, -0.6)
        point_pose = self.calculate_new_point(point_pose, 1, 0.6)
        # 旋转90°
        point_pose = self.calculate_new_point_rotation(point_pose, 1)
        self.new_point_reindex.append(point_pose)
        point_pose = self.calculate_reverse_point(point_pose, -15)


        for point_data, quaternion_data in self.new_point_reindex:
            point = Point(*point_data)
            quaternion = Quaternion(*quaternion_data)
            self.send_init_pose(point, quaternion)
            rospy.loginfo("\033[32m"+"途经点位发布成功"+"\033[0m")

        rospy.loginfo("\033[32m"+"下发目标点位"+"\033[0m")
        goal_point, goal_quaternion = point_pose
        goal_pose = Point(goal_point[0], goal_point[1], goal_point[2])
        goal_orientation = Quaternion(goal_quaternion[0], goal_quaternion[1], goal_quaternion[2], goal_quaternion[3])
        self.send_target_pose(goal_pose, goal_orientation)
        rospy.loginfo("\033[32m"+"目标点位发布成功"+"\033[0m")

        count = 0
        while position_ == 0 and not self.brake_enable:
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
        state_name = "HighSpeedState"
        with open(csv_file_path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([state_name, start_time, end_time, specific_start_time, specific_end_time,
                             round(elapsed_time, 3)])
            
        # 到达终点->开始夹取
        if not self.brake_enable: 
            position_ = 0
            self.start_time = None
            conveyer_belt_point = None
            rospy.loginfo_once("\033[32m"+'Position reached, switching to local_planning'+"\033[0m")
            return 'finish'
        # 如果有刹车
        elif self.brake_enable:
            position_ = 0
            conveyer_belt_point = None
            self.start_time = None
            rospy.loginfo_once('AEB')
            userdata.last_state = 'MOVING_TO_PICKUP_LOCATION'
            return 'braking'


class DistanceCheckObserve(smach.State, SerManager):
    def __init__(self, Throttle=None,distance=None, first_time_observer_flag=None):
        smach.State.__init__(self, outcomes=['finish'], io_keys=['conveyer_belt_point', 'observer_point', 'delivery_point'])
        SerManager.__init__(self)
        # 初始化相关变量
        self.conveyer_belt_yaw = None
        self.current_yaw = None
        self.steering_state = None
        self.distance_check = distance
        self.re_first_time_observer_flag = first_time_observer_flag

        
        self.Throttle = Throttle
        self.distance = 0
        self.start_position = None
        self.start_time = None


    def get_path_msg(self, target_distance, GEAR):
        global current_x, current_y, current_yaw

        path_msg = Path()
        path_msg.header.frame_id = 'map'

        # 根据目标距离和步长计算路径点数量
        num_points = int(abs(target_distance) / 0.1)
        # print("num_points:", num_points)
        step_distance = 0.1
        current_point = Point(current_x, current_y, 0.0)
        for _ in range(num_points):
            if GEAR == 1:  # 倒车
                # 根据当前航向和步长计算新的位置
                new_x = current_point.x - step_distance * math.cos(current_yaw)
                new_y = current_point.y - step_distance * math.sin(current_yaw)
            else:  # 前进
                new_x = current_point.x + step_distance * math.cos(current_yaw)
                new_y = current_point.y + step_distance * math.sin(current_yaw)
            new_point = Point(new_x, new_y, 0.0)

            pose_stamped = PoseStamped()
            pose_stamped.pose.position = new_point
            # 设置朝向与当前航向一致
            quaternion = tf.quaternion_from_euler(0, 0, current_yaw)
            pose_stamped.pose.orientation.x = quaternion[0]
            pose_stamped.pose.orientation.y = quaternion[1]
            pose_stamped.pose.orientation.z = quaternion[2]
            pose_stamped.pose.orientation.w = quaternion[3]
            pose_stamped.header.frame_id = 'map'
            path_msg.poses.append(pose_stamped)
            current_point = new_point


    def execute(self, userdata):
        global current_x, current_y, current_yaw
        global first_time_observer
        global observer_point
        start_time = time.time()
        specific_start_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(start_time))
        
        # 是否重新保存观察点
        if (self.re_first_time_observer_flag):
            first_time_observer = True

        goal_point1 = userdata.conveyer_belt_point[0]
        goal_point2 = userdata.conveyer_belt_point[1]

        goal_quaternion = goal_point2
        _, _, goal_yaw = tf.euler_from_quaternion([goal_quaternion[0], goal_quaternion[1], goal_quaternion[2], goal_quaternion[3]])
        x1, y1, _ = goal_point1
        if math.isclose(math.cos(goal_yaw), 0):
            # 直线垂直于 x 轴，此时用 y 坐标差值算距离
            self.distance_result = abs(current_y - y1)
        elif math.isclose(math.sin(goal_yaw), 0):
            self.distance_result = abs(current_x - x1)
            
        else:
            goal_slope = math.tan(goal_yaw)
            # # 目标点的垂直方向的直线，然后计算这个垂直距离来判断距离，而不是直接计算点之间距离
            self.distance_result = self.point_to_line_distance(current_x, current_y, x1, y1, -1/goal_slope)
        # print("distance:", self.distance_result)
        rospy.loginfo("\033[33m" + f"Need Distance: {(self.distance_result):.3f}" "\033[0m")

        # 如果距离不够那么先倒车不够的距离：
        if (self.distance_result < self.distance_check):
            error_dis = self.distance_check - self.distance_result
        else:
            error_dis = 0
        self.target_distance = -error_dis

        if self.start_position is None:
            self.start_position = Point(current_x, current_y, 0.0)

        if self.target_distance < 0:
            GEAR = 1
        else:
            GEAR = 3

        count = 0  # 初始化计数器

        # 获取路径
        # path_msg = self.get_path_msg(self.target_distance, GEAR)
    
        rospy.loginfo("\033[32m" + f"Target:{self.target_distance:.3f}" "\033[0m")
        rospy.loginfo("\033[33m" + f"Current:{self.distance:.3f}" "\033[0m")

        while True:
            count += 1
            if count % 100 == 0:  
                rospy.loginfo("\033[32m" + f"Target:{self.target_distance:.3f}" "\033[0m")
                rospy.loginfo("\033[33m" + f"Current:{self.distance:.3f}" "\033[0m")

            current_position = Point(current_x, current_y, 0.0)
            dx = current_position.x - self.start_position.x
            dy = current_position.y - self.start_position.y
            self.distance = ((dx ** 2) + (dy ** 2)) ** 0.5


            if self.distance >= abs(self.target_distance):

                # 到位小刹车
                self.brake_pub.publish(4.0)
                self.steering_pub.publish(0.0)
                throttle_cmd = Float64(0.0)
                self.throttle_pub.publish(0)
                gear_cmd = UInt8(2)
                self.gear_pub.publish(gear_cmd)
                if self.start_time is None:
                    self.start_time = rospy.Time.now()
                # print(55555)

                if (rospy.Time.now() - self.start_time).to_sec() > 0.5:
                    rospy.loginfo_once("\033[33m" + f"last Current:{self.distance:.3f}" "\033[0m")
                    break

            else:
                steering_cmd = Float64(0.0)
                self.steering_pub.publish(steering_cmd)
                throttle_cmd = Float64(self.Throttle)
                self.throttle_pub.publish(throttle_cmd)
                gear_cmd = UInt8(GEAR)
                self.gear_pub.publish(gear_cmd)
                brake_cmd = Float64(0.0)
                self.brake_pub.publish(brake_cmd)
                # 发布路径消息
                path_msg = Path()
                path_msg.header.frame_id = 'map'
                self.path_publisher.publish(path_msg)
            
            rospy.Rate(100).sleep()

        # 记录此时的观察点作为以后的观察点：
        if (first_time_observer):
            userdata.observer_point = self.process_point(current_x, current_y, current_yaw)
            observer_point = self.process_point(current_x, current_y, current_yaw)
            # 只记录一次观察点
            first_time_observer = False

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
            self.start_position = None
            self.distance = 0
            self.start_time = None
            return 'finish'


# 完成状态
class Finish(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeed'], io_keys=['conveyer_belt_point', 'observer_point', 'delivery_point'])
    def execute(self, userdata):
        rospy.loginfo("\033[32m"+'Executing state: FINISH'+"\033[0m")
        return 'succeed'


class GlobalPickupTask_1(smach.State, SerManager):
    def __init__(self,control_task=None, limited_speed=None):
        smach.State.__init__(self, outcomes=['finish'], io_keys=['conveyer_belt_point', 'observer_point', 'delivery_point'])
        SerManager.__init__(self)
        
        self.limited_speed = limited_speed
        
        self.new_point_delivery = []
        self.new_point_reindex = []
        self.control_task = control_task


    def execute(self, userdata):
        global position_
        global conveyer_belt_point_camera
        global conveyer_belt_point

        start_time = time.time()
        specific_start_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(start_time))


        print("conveyer_belt_point", userdata.conveyer_belt_point)
        rospy.loginfo_once("\033[32m"+'Executing state: GlobalPickupTask_1'+"\033[0m")
        rospy.loginfo("\033[32m"+"下发全局任务"+"\033[0m")
        self.task = 1
        
        self.send_plan_task_service(self.task)
        self.send_control_task_service(self.control_task, self.limited_speed)
        # self.send_stop_task_service(self.task) # 0-不响应 1-响应停障
        rospy.loginfo("\033[32m"+"任务下发成功"+"\033[0m")

        conveyer_belt_point_camera = userdata.conveyer_belt_point
        right_conveyer_belt_point = self.calculate_new_point(conveyer_belt_point_camera, 1, 0.0) # 目标点的左右移动（在camera中使用了）
        reversed_goal_yaw_point = self.calculate_reverse_point(right_conveyer_belt_point, 2.0) # 后移 要求检测距离为2.5m时。
        self.new_point_reindex.append(reversed_goal_yaw_point)

        for point_data, quaternion_data in self.new_point_reindex:
            point = Point(*point_data)
            quaternion = Quaternion(*quaternion_data)
            self.send_init_pose(point, quaternion)
            rospy.loginfo("\033[32m"+"途经点位发布成功"+"\033[0m")

        rospy.loginfo("\033[32m"+"下发目标点位"+"\033[0m")
        goal_point, goal_quaternion = right_conveyer_belt_point
        goal_pose = Point(goal_point[0], goal_point[1], goal_point[2])
        goal_orientation = Quaternion(goal_quaternion[0], goal_quaternion[1], goal_quaternion[2], goal_quaternion[3])
        self.send_target_pose(goal_pose, goal_orientation)
        rospy.loginfo("\033[32m"+"目标点位发布成功"+"\033[0m")

        count = 0
        while position_ == 0 and not self.brake_enable:
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
        # self.send_stop_task_service(0)
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
            self.start_time = None
            conveyer_belt_point = None
            userdata.conveyer_belt_point = None
            rospy.loginfo_once("\033[32m"+'Position reached, switching to local_planning'+"\033[0m")
            return 'finish'



class GlobalPickupTask_3(smach.State, SerManager):
    def __init__(self,control_task=None,limited_speed=None):
        smach.State.__init__(self, outcomes=['finish'], io_keys=['conveyer_belt_point', 'observer_point', 'delivery_point'])
        SerManager.__init__(self)
        
        self.limited_speed = limited_speed
        # rospy.Subscriber('/steering_state', Float64, self.steering_callback)
        self.new_point_delivery = []
        self.new_point_reindex = []
        self.control_task = control_task

    def steering_callback(self, msg):
        self.steering_state = msg.data

    def execute(self, userdata):
        global position_
        global conveyer_belt_point_camera

        start_time = time.time()
        specific_start_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(start_time))


        rospy.loginfo_once("\033[32m"+'Executing state: GlobalPickupTask_3'+"\033[0m")
        rospy.loginfo("\033[32m"+"下发全局任务"+"\033[0m")
        self.task = 1
        
        self.send_plan_task_service(self.task)
        # TODO 给规划传递限速
        self.send_control_task_service(self.control_task, self.limited_speed)
        self.send_stop_task_service(0)
        rospy.loginfo("\033[32m"+"任务下发成功"+"\033[0m")

        # observer_point = userdata.observer_point
        # print("observer_point", userdata.observer_point)
        
        point_observe = self.calculate_new_point(observer_point, 1, 0.0) # 观察点不动
        front_point_observe = self.calculate_reverse_point(point_observe, -1)
        self.new_point_reindex.append(front_point_observe)
        self.new_point_reindex.append(point_observe)
        reversed_goal_yaw_point = self.calculate_reverse_point(point_observe, -2.2)

        for point_data, quaternion_data in self.new_point_reindex:
            point = Point(*point_data)
            quaternion = Quaternion(*quaternion_data)
            self.send_init_pose(point, quaternion)
            rospy.loginfo("\033[32m"+"途经点位发布成功"+"\033[0m")

        rospy.loginfo("\033[32m"+"下发目标点位"+"\033[0m")
        goal_point, goal_quaternion = reversed_goal_yaw_point
        goal_pose = Point(goal_point[0], goal_point[1], goal_point[2])
        goal_orientation = Quaternion(goal_quaternion[0], goal_quaternion[1], goal_quaternion[2], goal_quaternion[3])
        self.send_target_pose(goal_pose, goal_orientation)
        rospy.loginfo("\033[32m"+"目标点位发布成功"+"\033[0m")

        count = 0
        while position_ == 0 and not self.brake_enable:
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
        self.send_stop_task_service(0) # 0-不响应 1-响应停障
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
            self.start_time = None
            userdata.conveyer_belt_point = None
            rospy.loginfo_once("\033[32m"+'Position reached, switching to local_planning'+"\033[0m")
            return 'finish'
        


class GlobalPickupTask_2(smach.State, SerManager):
    def __init__(self,control_task=None,limited_speed=None):
        smach.State.__init__(self, outcomes=['finish'], io_keys=['conveyer_belt_point', 'observer_point', 'delivery_point'])
        SerManager.__init__(self)
        
        self.limited_speed = limited_speed
        # rospy.Subscriber('/steering_state', Float64, self.steering_callback)
        
        self.new_point_delivery = []
        self.new_point_reindex = []
        self.control_task = control_task

    def steering_callback(self, msg):
        self.steering_state = msg.data

    def execute(self, userdata):
        global position_
        global conveyer_belt_point_camera
        global cotton_4_num

        start_time = time.time()
        specific_start_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(start_time))

        print("conveyer_belt_point", userdata.conveyer_belt_point)
        rospy.loginfo_once("\033[32m"+'Executing state: GlobalPickupTask_2'+"\033[0m")
        rospy.loginfo("\033[32m"+"下发全局任务"+"\033[0m")
        self.task = 1
        
        self.send_plan_task_service(self.task)
        # TODO 给规划传递限速
        self.send_control_task_service(self.control_task, self.limited_speed)
        # self.send_stop_task_service(self.task)
        rospy.loginfo("\033[32m"+"任务下发成功"+"\033[0m")

        observer_point = userdata.observer_point
        point_observe = self.calculate_new_point(observer_point, 1, 0.0) # 观察点不动
        front_point_observe = self.calculate_reverse_point(point_observe, -1)
        self.new_point_reindex.append(front_point_observe)
        self.new_point_reindex.append(point_observe)
        reversed_goal_yaw_point = self.calculate_reverse_point(point_observe, 1.5+cotton_4_num*2.2) # 后移 要求检测距离为2.5m时。

        for point_data, quaternion_data in self.new_point_reindex:
            point = Point(*point_data)
            quaternion = Quaternion(*quaternion_data)
            self.send_init_pose(point, quaternion)
            rospy.loginfo("\033[32m"+"途经点位发布成功"+"\033[0m")

        rospy.loginfo("\033[32m"+"下发目标点位"+"\033[0m")
        goal_point, goal_quaternion = reversed_goal_yaw_point
        goal_pose = Point(goal_point[0], goal_point[1], goal_point[2])
        goal_orientation = Quaternion(goal_quaternion[0], goal_quaternion[1], goal_quaternion[2], goal_quaternion[3])
        self.send_target_pose(goal_pose, goal_orientation)
        rospy.loginfo("\033[32m"+"目标点位发布成功"+"\033[0m")

        count = 0
        while position_ == 0 and not self.brake_enable:
            # 空的路径
            path_msg = Path()
            path_msg.header.frame_id = 'map'
            self.path_publisher.publish(path_msg)
            if(position_==1):
                cotton_4_num += 1
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
            self.start_time = None
            userdata.conveyer_belt_point = None
            rospy.loginfo_once("\033[32m"+'Position reached, switching to local_planning'+"\033[0m")
            return 'finish'



class AdjustHeading_Design_Angle(smach.State,SerManager):
    def __init__(self, angle=None,steering_angle=None, gear=None, throttle=None, control_com=None):
        smach.State.__init__(self, outcomes=['finish'], io_keys=['conveyer_belt_point', 'observer_point', 'delivery_point'])
        SerManager.__init__(self)
        # 初始化相关变量
        self.conveyer_belt_yaw = None
        self.current_yaw = None
        self.steering_state = None
        self.dest_gear=gear
        self.adjust_steering_angle = steering_angle

        rospy.Subscriber('/steering_state', Float64, self.steering_callback)

        self.throttle = throttle

        self.conveyer_belt_point = 0
        self.delivery_point_mode = 0
        self.control_com = control_com
        
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
                return 'finish'


class AdjustHeading(smach.State, SerManager):
    def __init__(self, goal=None,steering_angle=None, changeYawangle=None,gear=None, throttle=None,control_com=None):
        smach.State.__init__(self, outcomes=['finish'], io_keys=['delivery_point', 'conveyer_belt_point', 'observer_point'])
        SerManager.__init__(self)
        # 初始化相关变量
        self.conveyer_belt_yaw = None
        self.current_yaw = None
        self.steering_state = None
        self.dest_gear=gear

        rospy.Subscriber('/steering_state', Float64, self.steering_callback)
        self.throttle = throttle

        self.conveyer_belt_point = 0
        self.delivery_point_mode = 0
        self.control_com = control_com
        self.changeYawangle = changeYawangle # 正为逆时针，减为顺时针
        self.goal = goal
        self.adjust_steering_angle = steering_angle

        self.angle_diff = 0
        self.init_yaw = None
        self.change_angle = True
        self.start_time = None
        self.goal_yaw = 0

    def steering_callback(self, msg):
        self.steering_state = msg.data


    def execute(self, userdata):
        global current_yaw, current_x, current_y
        global delivery_point, conveyer_belt_point
        global cotton_cross_product,observer_point

        start_time = time.time()
        specific_start_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(start_time))

        print("userdata.conveyer_belt_point: ", userdata.conveyer_belt_point)
        print("userdata.observer_point: ", userdata.observer_point)
        # if self.goal == 'delivery_point':
        #     self.delivery_point_mode = userdata.delivery_point
        if self.goal=="conveyer_belt_point":
            self.delivery_point_mode = userdata.conveyer_belt_point
        elif self.goal=="observer_point":
            self.delivery_point_mode = userdata.observer_point
        # elif self.goal=="delivery_point":
        #     self.delivery_point_mode = userdata.delivery_point

        else:
            rospy.logwarn("Invalid parameter for conveyer_belt_point.")

        if len(self.goal) == 2 and isinstance(self.goal[1], (tuple, list)) and len(self.goal[1]) == 4:
            quaternion = Quaternion(*self.goal[1])
            self.goal_yaw = self.get_yaw_from_quaternion(quaternion)
        else:
            rospy.logwarn("Invalid parameter format for conveyer_belt_point.")

        rospy.loginfo_once("\033[32m"+'Executing state: Adjust Heading Task'+"\033[0m")

        count = 0
        while(current_y==0):
            rospy.loginfo_once("\033[31m"+'Wait Odom'+"\033[0m")
        
        rospy.loginfo_once("\033[33m"+f' 任务目标: {self.goal} '+ "\033[0m")

        self.current_yaw = current_yaw
        if self.init_yaw is None:
            self.init_yaw = self.current_yaw
        print("self.goal_yaw: ", self.goal_yaw*180/math.pi)
        print("self.current_yaw: ", self.init_yaw)

        goal_yaw = self.goal_yaw + self.changeYawangle * math.pi / 180
        direction, self.need_diff = self.calculate_optimal_rotation(self.current_yaw*180/math.pi, goal_yaw*180/math.pi)
        print("direction: ", direction)
        print("self.need_diff: ", self.need_diff*180/math.pi)
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
            # 空的路径
            path_msg = Path()
            path_msg.header.frame_id = 'map'
            self.path_publisher.publish(path_msg)
            
            self.current_yaw = current_yaw

            if self.steering_state is not None and self.current_yaw is not None :
                # 1先把车轮转好
                steering_diff = abs(adjust_steering_angle_ - self.steering_state)
                if self.change_angle:
                    # 只能执行一次self.change_angle
                    self.steering_pub.publish(adjust_steering_angle_)
                    self.brake_pub.publish(0.0)
                    self.throttle_pub.publish(0.0)
                    gear_cmd = UInt8(self.dest_gear)
                    self.gear_pub.publish(gear_cmd)
                    if steering_diff < 0.1:
                        self.change_angle = False
                else:
                    # 2累计转过的 yaw，如果大于等于 self.angle_diff 先把角度转回来，然后停止
                    if abs(self.calculate_angle(self.init_yaw, self.current_yaw)) <= (abs(self.need_diff)-self.control_com*math.pi/180): # 少转一点，补偿90度
                        steering_cmd = Float64(adjust_steering_angle_)
                        self.steering_pub.publish(steering_cmd)
                        self.brake_pub.publish(0.0)
                        throttle_cmd = Float64(self.throttle)
                        self.throttle_pub.publish(throttle_cmd)
                        gear_cmd = UInt8(self.dest_gear)
                        self.gear_pub.publish(gear_cmd)
                    
                    else:
                        # 到位小刹车
                        self.brake_pub.publish(4.0)
                        self.steering_pub.publish(0.0)
                        throttle_cmd = Float64(0.0)
                        self.throttle_pub.publish(0)
                        gear_cmd = UInt8(2)
                        self.gear_pub.publish(gear_cmd)
                        if self.start_time is None:
                            self.start_time = rospy.Time.now()
                        if (rospy.Time.now() - self.start_time).to_sec() > 0.5:
                            rospy.loginfo("\033[32m"+"last diff: " + str(abs(self.calculate_angle(self.init_yaw, self.current_yaw))) + "\033[0m")
                            break

                rospy.loginfo("\033[32m"+"NOW TURN: " + f"{abs(self.calculate_angle(self.init_yaw, self.current_yaw)):.3f}" + "\033[0m")
                rospy.loginfo("\033[32m"+"NEED TURN: " + f"{abs(self.need_diff):.3f}" + "\033[0m")
                rospy.loginfo("\033[32m"+"Wheel Angle pub: " + f"{adjust_steering_angle_:.3f}" + "\033[0m")

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
            return 'finish'



class CAMERA_TASK(smach.State, SerManager):
    def __init__(self , goal=None, use_camera_det=None):
        smach.State.__init__(self, outcomes=['finish'], io_keys=['conveyer_belt_point', 'observer_point', 'delivery_point'])
        SerManager.__init__(self)
        self.goal = goal
        self.use_camera_det = use_camera_det

    def execute(self, userdata):
        global conveyer_belt_point
        global conveyer_belt_point_sim

        start_time = time.time()
        specific_start_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(start_time))

        time.sleep(0.5)
        if self.use_camera_det:
            if (self.goal==1):
                self.task = 2
            elif (self.goal==4):
                self.task = 3
            # 相机使能
            self.send_camera_task_service(self.task)
            # print("1111111111111111")
            # 相机位姿
            while conveyer_belt_point is None:
                # print("cotton_x:", cotton_x)
                if(conveyer_belt_point is not None):
                    userdata.conveyer_belt_point = conveyer_belt_point
                    if (userdata.conveyer_belt_point is not None):
                        break
                rospy.loginfo("\033[32m"+"等待识别最近的棉包"+"\033[0m")
                rospy.Rate(1).sleep()
            rospy.loginfo_once("\033[32m"+"识别到棉包位姿，关闭相机"+"\033[0m")
            userdata.conveyer_belt_point = conveyer_belt_point
            
            self.task = 0
            # 关闭
            self.send_camera_task_service(0)

            end_time = time.time()
            elapsed_time = end_time - start_time
            specific_end_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(end_time))
            state_name = "CAMERA_TASK"
            with open(csv_file_path, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([state_name, start_time, end_time, specific_start_time, specific_end_time,
                                round(elapsed_time, 3)])
        
        else:
            rospy.loginfo("\033[32m"+"yaml参数给出识别目标"+"\033[0m")
            # userdata.conveyer_belt_point = conveyer_belt_point
            userdata.conveyer_belt_point = conveyer_belt_point_sim
            print("userdata.conveyer_belt_point:", userdata.conveyer_belt_point)
        return 'finish'


class GlobalMoveTask_CloseLoop(smach.State, SerManager):
    def __init__(self, target_distance=None, limited_speed=None):
        smach.State.__init__(self, outcomes=['finish'], io_keys=['conveyer_belt_point', 'observer_point', 'delivery_point'])
        SerManager.__init__(self)
        self.target_distance = target_distance
        self.Throttle = limited_speed
        
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

        rospy.loginfo_once("\033[32m"+'Executing state: GlobalMoveTask_CloseLoop'+"\033[0m")
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
            return 'finish'


class GlobalMoveTask_OpenLoop(smach.State, SerManager):
    def __init__(self, target_distance=None, Throttle=None):
        smach.State.__init__(self, outcomes=['finish'], io_keys=['conveyer_belt_point', 'observer_point', 'delivery_point'])
        SerManager.__init__(self)
        self.target_distance = target_distance
        self.start_position = None
        self.Throttle = Throttle
        
        rospy.Subscriber('/ipc_state', Int8, self.ipc_state_callback)
        self.ipc_state = 0
        self.distance = 0
        self.start_time = None

    def ipc_state_callback(self, msg):
        self.ipc_state = msg.data


    def get_path_msg(self, target_distance, GEAR):
        global current_x, current_y, current_yaw

        path_msg = Path()
        path_msg.header.frame_id = 'map'

        # 根据目标距离和步长计算路径点数量
        num_points = int(abs(target_distance) / 0.1)
        # print("num_points:", num_points)
        step_distance = 0.1
        current_point = Point(current_x, current_y, 0.0)
        for _ in range(num_points):
            if GEAR == 1:  # 倒车
                # 根据当前航向和步长计算新的位置
                new_x = current_point.x - step_distance * math.cos(current_yaw)
                new_y = current_point.y - step_distance * math.sin(current_yaw)
            else:  # 前进
                new_x = current_point.x + step_distance * math.cos(current_yaw)
                new_y = current_point.y + step_distance * math.sin(current_yaw)
            new_point = Point(new_x, new_y, 0.0)

            pose_stamped = PoseStamped()
            pose_stamped.pose.position = new_point
            # 设置朝向与当前航向一致
            quaternion = tf.quaternion_from_euler(0, 0, current_yaw)
            pose_stamped.pose.orientation.x = quaternion[0]
            pose_stamped.pose.orientation.y = quaternion[1]
            pose_stamped.pose.orientation.z = quaternion[2]
            pose_stamped.pose.orientation.w = quaternion[3]
            pose_stamped.header.frame_id = 'map'
            path_msg.poses.append(pose_stamped)
            current_point = new_point

    def execute(self, userdata):
        global position_
        global current_x, current_y, current_yaw
        global move_distance

        start_time = time.time()
        specific_start_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(start_time))

        time.sleep(0.5)
        if self.start_position is None:
            self.start_position = Point(current_x, current_y, 0.0)

        if self.target_distance < 0:
            GEAR = 1
            rospy.loginfo("\033[33m"+"倒车"+"\033[0m")
        else:
            GEAR = 3
            rospy.loginfo("\033[33m"+"前进"+"\033[0m")

        # path_msg = self.get_path_msg(self.target_distance, GEAR)

        count = 0
        while True:
            count += 1
            
            if count % 50 == 0:
                rospy.loginfo("\033[34m" + f"Target:{self.target_distance:.3f}" "\033[0m")
                rospy.loginfo("\033[34m" + f"Current:{self.distance:.3f}" "\033[0m")

            current_position = Point(current_x, current_y, 0.0)
            dx = current_position.x - self.start_position.x
            dy = current_position.y - self.start_position.y
            self.distance = ((dx ** 2) + (dy ** 2)) ** 0.5  
            move_distance = self.distance

            self.steering_pub.publish(0.0)
            throttle_cmd = Float64(self.Throttle)
            self.throttle_pub.publish(throttle_cmd)
            gear_cmd = UInt8(GEAR)
            self.gear_pub.publish(gear_cmd)
            brake_cmd = Float64(0.0)
            self.brake_pub.publish(brake_cmd)
            # 发布路径消息
            # self.path_publisher.publish(path_msg)
            
            if self.distance >= abs(self.target_distance):
                self.steering_pub.publish(0.0)
                self.brake_pub.publish(4.0)
                self.throttle_pub.publish(0.0)
                self.gear_pub.publish(2)
                if self.start_time is None:
                    self.start_time = rospy.Time.now()
                if (rospy.Time.now() - self.start_time).to_sec() > 0.3:
                    rospy.loginfo("\033[33m" + f"last Current:{self.distance:.3f}" "\033[0m")
                    break
            rospy.Rate(100).sleep()

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
            self.start_position = None
            self.distance = 0
            rospy.loginfo_once("\033[32m"+'Position reached, switching to local_planning'+"\033[0m")
            return 'finish'


class CloudTaskFeedback(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'],input_keys=['task_id'])

    def execute(self, userdata):
        global delivery_point

        start_time = time.time()
        specific_start_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(start_time))

        rospy.loginfo("\033[32mExecuting state: SEND_TASK_STS_MESSAGE\033[0m")

        # Create a TaskStsRequest object
        req = TaskStsRequest()
        req.order_id = "0"
        try:
            req.task_id = task_id
            req.business_type = "inbound"
            req.origin_shelf_area = "A"
            req.destination_shelf_area = "E1"
            req.bale_num_to_handle = 2
            req.task_sts = 2
            req.final_pose = pose()
            req.final_pose.x = 1.0
            req.final_pose.y = 2.0
            req.final_pose.yaw = 0.5

            # Send the request
            success = self.send_task_sts_message(req)

            end_time = time.time()
            elapsed_time = end_time - start_time
            specific_end_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(end_time))
            state_name = f"{self.__class__.__name__}"
            with open(csv_file_path, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(
                    [state_name, start_time, end_time, specific_start_time, specific_end_time, round(elapsed_time, 3)])

            if success:
                rospy.loginfo("Task status message sent successfully.")
                delivery_point = ()
                return 'success'
        except :
            rospy.logerr("No task_id found in userdata. Skipping task status update.")
            return 'success'
        

class Init(smach.State, SerManager):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'], io_keys=['conveyer_belt_point', 'observer_point', 'delivery_point'])
        SerManager.__init__(self)
    def execute(self, userdata):

        userdata.conveyer_belt_point = None
        userdata.observer_point = None
        userdata.delivery_point = None

        self.send_control_task_service(0, 0)
        # self.send_camera_task_service(0)
        self.send_plan_task_service(0)
        # self.send_stop_task_service(0)

        start_time = time.time()
        return 'success'


def create_state_machine():
    global delivery_point, conveyer_belt_point, observer_point
    global work_direction, num_work, num_work_max, num_work_height
    global conveyer_belt_point_sim


    #===========================Param==================================
    #------------------------------------------------------------------
    # 从参数服务器中获取参数：
    test_high_speed_control = rospy.get_param('/real/test_high_speed_control', False) 
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
    test_obstacle = rospy.get_param('/real/test_obstacle', False)

    if not use_camera_det:
        # 传送带位置
        conveyer_belt_point_sim = rospy.get_param('conveyer_belt_point_sim', ((24.3610,12.3270,0.5682),(0.0261,-0.0068,-0.9996,0.0003)))
    
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
    # 创建数据库
    sm = smach.StateMachine(outcomes=['succeed', 'aborted', 'preempted'])
    userdata = smach.UserData()
    userdata.conveyer_belt_point = None  # 初始化 conveyer_belt_point
    userdata.observer_point = None  # 初始化 observer_point
    userdata.delivery_point = None  # 初始化 delivery_point
    if (not test_obstacle and not test_high_speed_control):
        sm.set_initial_state(['SUB_STATE_MACHINE'], userdata)  # 设置初始状态并传递用户数据
    with sm:
        # 高速循迹+停障功能
        if test_obstacle:
            smach.StateMachine.add('LidarObstacle', LidarObstacle(lidar_obs=1), transitions={'finish': 'FINISH'})

        # 高速才能测试循迹，至少3m/s
        if test_high_speed_control:
            smach.StateMachine.add('HIGH_SPEED_TEST_STATE', HighSpeedState(), transitions={'finish': 'FINISH'})

        sub_sm = smach.StateMachine(outcomes=['finish'])
        with sub_sm:
            smach.StateMachine.add('INIT_STATE', Init(), transitions={'success': 'SELF_INSPECTION'})
            # 并行状态机 初始化夹抱 俯仰和高度
            if not use_sim:
                parallel_container = smach.Concurrence(outcomes=['all_finished'],
                                                default_outcome='all_finished',
                                                outcome_map={'all_finished': {
                                                    'SET_WIDTH_INIT': 'finish',
                                                    'SET_FY_INIT': 'finish',
                                                    'SET_HEIGHT_INIT': 'finish'}},
                                                input_keys=['conveyer_belt_point', 'observer_point', 'delivery_point'], 
                                                output_keys=['conveyer_belt_point', 'observer_point', 'delivery_point'])
                with parallel_container:
                        smach.Concurrence.add('SET_WIDTH_INIT', SetWidth(tar_width=10))
                        smach.Concurrence.add('SET_FY_INIT', SetFY(tar_fy=1111))
                        smach.Concurrence.add('SET_HEIGHT_INIT', SetHeight(tar_height=80, updown_mode=2))
                smach.StateMachine.add('SELF_INSPECTION', parallel_container,transitions={'all_finished': 'HW_SETUP'})
            else:
                parallel_container = smach.Concurrence(outcomes=['all_finished'],
                                                default_outcome='all_finished',
                                                outcome_map={'all_finished': {
                                                    'SET_WIDTH_INIT': 'finish',
                                                    'SET_HEIGHT_INIT': 'finish'}},
                                                input_keys=['conveyer_belt_point', 'observer_point', 'delivery_point'], 
                                                output_keys=['conveyer_belt_point', 'observer_point', 'delivery_point']
                                                )
                with parallel_container:
                        smach.Concurrence.add('SET_WIDTH_INIT', SetWidthSim(tar_width=clamping_width_sim))
                        smach.Concurrence.add('SET_HEIGHT_INIT', SetHeightSim(tar_height=single_cargo_height_sim))
                smach.StateMachine.add('SELF_INSPECTION', parallel_container,transitions={'all_finished': 'HW_SETUP'})


            # 接收云平台的调度指令  从点，到按钮，等等
            # smach.StateMachine.add('HANDLE_CLOUD_ORDER', HandleCloudOrder(), transitions={'finish': 'HW_SETUP'})

            # 并行状态机 设置初始夹抱参数
            parallel_container2 = smach.Concurrence(outcomes=['all_finished'],
                                            default_outcome='all_finished',
                                            outcome_map={'all_finished': {
                                                'SET_WIDTH_START': 'finish',
                                                'SET_HEIGHT_START': 'finish'}},
                                            input_keys=['conveyer_belt_point', 'observer_point', 'delivery_point'], 
                                            output_keys=['conveyer_belt_point', 'observer_point', 'delivery_point']
                                            )
            with parallel_container2:
                if not use_sim:
                    smach.Concurrence.add('SET_WIDTH_START', SetWidth(tar_width=open_width))
                    smach.Concurrence.add('SET_HEIGHT_START', SetHeight(tar_height=80, updown_mode=2))
                else:
                    smach.Concurrence.add('SET_WIDTH_START', SetWidthSim(tar_width=open_width_sim))
                    smach.Concurrence.add('SET_HEIGHT_START', SetHeightSim(tar_height=height_between_two_cargos_sim))
            smach.StateMachine.add('HW_SETUP', parallel_container2,
                                transitions={'all_finished': 'PICKUP_SUB_SECTION'})
            
            # 嵌套
            pickup_sub_sm = smach.StateMachine(outcomes=['pickup_sub_finish'], input_keys=['conveyer_belt_point', 'observer_point', 'delivery_point'],output_keys=['conveyer_belt_point', 'observer_point', 'delivery_point'])
            with pickup_sub_sm:
                # use_camera_det 是否进行闭环检测
                smach.StateMachine.add('CAMERA_TASK_1', CAMERA_TASK(goal=1, use_camera_det=use_camera_det), transitions={'finish': 'DISTANCE_CHECK'})
                smach.StateMachine.add('DISTANCE_CHECK', DistanceCheckObserve(Throttle=0.5, distance=2.5, first_time_observer_flag=False),transitions={'finish': 'GLOBAL_PICKUP_1'})
                smach.StateMachine.add('GLOBAL_PICKUP_1', GlobalPickupTask_1(control_task=2, limited_speed=0.5), transitions={'finish': 'SET_WIDTH_PICKUP'})
                if not use_sim:
                    # 取一个包
                    smach.StateMachine.add('SET_WIDTH_PICKUP', SetWidth(clamping_width), transitions={'finish': 'HEIGHT_RAISE_FIRST'})
                    # 升高，准备后退
                    smach.StateMachine.add('HEIGHT_RAISE_FIRST', SetHeight(height_between_two_cargos+5, updown_mode=2),transitions={'finish': 'pickup_sub_finish'})
                else:
                    # 取一个包
                    smach.StateMachine.add('SET_WIDTH_PICKUP', SetWidthSim(clamping_width_sim), transitions={'finish': 'HEIGHT_RAISE_FIRST'})
                    # 升高，准备后退
                    smach.StateMachine.add('HEIGHT_RAISE_FIRST', SetHeightSim(height_between_two_cargos),transitions={'finish': 'pickup_sub_finish'})
            smach.StateMachine.add('PICKUP_SUB_SECTION', pickup_sub_sm, transitions={'pickup_sub_finish': 'HW_BACKUP'})
            # 嵌套

            # 倒车继续识别目标
            # 并列状态机 倒车 下降 但是下降需要延时
            parallel_container3 = smach.Concurrence(outcomes=['all_finished'],
                                            default_outcome='all_finished',
                                            outcome_map={'all_finished': {
                                                'GLOBAL_MOVE_BACK': 'finish',
                                                'HEIGHT_LOWER_WITH_DELAY': 'finish'}},
                                            input_keys=['conveyer_belt_point', 'observer_point', 'delivery_point'],
                                            output_keys=['conveyer_belt_point', 'observer_point', 'delivery_point']
                                            )
            with parallel_container3:
                smach.Concurrence.add('GLOBAL_MOVE_BACK', GlobalMoveTask_OpenLoop(target_distance=(-2.3), Throttle=1.0))
                if not use_sim:
                    smach.Concurrence.add('HEIGHT_LOWER_WITH_DELAY', SetHeight_Delay(tar_height=170+5, updown_mode=2, set_min_distance=0.5))
                else:
                    smach.Concurrence.add('HEIGHT_LOWER_WITH_DELAY', SetHeight_Delay_Sim(tar_height=single_cargo_height_sim, set_min_distance=1.7))
            smach.StateMachine.add('HW_BACKUP', parallel_container3,transitions={'all_finished': 'CAMERA_TASK_2'})


            smach.StateMachine.add('CAMERA_TASK_2', CAMERA_TASK(goal=1, use_camera_det=use_camera_det), transitions={'finish': 'HW_PLAN_SECOND_PICKUP'},)

            # 规划继续取第二个包
            parallel_container4 = smach.Concurrence(outcomes=['all_finished'],
                                            default_outcome='all_finished',
                                            outcome_map={'all_finished': {
                                                'GLOBAL_PICKUP_2': 'finish',
                                                'HEIGHT_ADJUST_SECOND': 'finish'}},
                                            input_keys=['conveyer_belt_point', 'observer_point'],
                                            output_keys=['conveyer_belt_point', 'observer_point']
                                            )
            with parallel_container4:
                smach.Concurrence.add('GLOBAL_PICKUP_2', GlobalPickupTask_1(control_task=2, limited_speed=0.5))
                if not use_sim:
                    smach.Concurrence.add('HEIGHT_ADJUST_SECOND', SetHeight(tar_height=above_two_cargos_height, updown_mode=2),)
                else:
                    smach.Concurrence.add('HEIGHT_ADJUST_SECOND', SetHeightSim(tar_height=height_between_two_cargos_sim),)
            smach.StateMachine.add('HW_PLAN_SECOND_PICKUP', parallel_container4,transitions={'all_finished': 'STACK_AND_MOVE_SECTION'})

            # 嵌套
            stack_and_move_sub_sm = smach.StateMachine(outcomes=['stack_and_move_finish'], input_keys=['conveyer_belt_point', 'observer_point'], output_keys=['conveyer_belt_point', 'observer_point'])
            with stack_and_move_sub_sm:
                stack_sub = smach.StateMachine(outcomes=['stack_finish'])
                with stack_sub:
                    if not use_sim:
                        smach.StateMachine.add('HEIGHT_STACK', SetHeight(two_cargos_height, updown_mode=2),transitions={'finish': 'WIDTH_OPEN_FIRST'} )
                        smach.StateMachine.add('WIDTH_OPEN_FIRST', SetWidth(open_width_-40), transitions={'finish': 'stack_finish'})
                    else:
                        smach.StateMachine.add('HEIGHT_STACK', SetHeightSim(single_cargo_height_sim),transitions={'finish': 'WIDTH_OPEN_FIRST'})
                        smach.StateMachine.add('WIDTH_OPEN_FIRST', SetWidthSim(open_width_sim), transitions={'finish': 'stack_finish'})
                smach.StateMachine.add('STACK_SECTION', stack_sub,
                                    transitions={'stack_finish': 'MOVE_BACK_DOWN'})

                # 倒车 边倒车边下降
                smach.StateMachine.add('MOVE_BACK_DOWN', GlobalMoveTask_OpenLoop(target_distance=(-0.1), Throttle=0.2),
                                        transitions={'finish': 'HEIGHT_LOWER_FINAL'}, )
                if not use_sim:
                    # 下降
                    smach.StateMachine.add('HEIGHT_LOWER_FINAL', SetHeight(single_cargo_height_half, updown_mode=1),transitions={'finish': 'GLOBAL_MOVE_FORWARD'})
                else:
                    smach.StateMachine.add('HEIGHT_LOWER_FINAL', SetHeightSim(height_between_two_cargos_sim),transitions={'finish': 'GLOBAL_MOVE_FORWARD'})

                # 前进，顶齐
                smach.StateMachine.add('GLOBAL_MOVE_FORWARD', GlobalMoveTask_OpenLoop(target_distance=(0.2 + 0.15), Throttle=0.3),
                                        transitions={'finish': 'CLOSE_SECTION'})

                # 嵌套
                close_sub = smach.StateMachine(outcomes=['close_finish'], input_keys=['conveyer_belt_point', 'observer_point', 'delivery_point'], output_keys=['conveyer_belt_point', 'observer_point', 'delivery_point'])
                with close_sub:
                    if not use_sim:
                        smach.StateMachine.add('WIDTH_CLOSE', SetWidth(clamping_width), transitions={'finish': 'HEIGHT_RAISE_FINAL'})
                        smach.StateMachine.add('HEIGHT_RAISE_FINAL', SetHeight(height_between_two_cargos + 1, updown_mode=2), transitions={'finish': 'close_finish'})
                    else:
                        smach.StateMachine.add('WIDTH_CLOSE', SetWidthSim(clamping_width), transitions={'finish': 'HEIGHT_RAISE_FINAL'})
                        smach.StateMachine.add('HEIGHT_RAISE_FINAL', SetHeightSim(height_between_two_cargos_sim), transitions={'finish': 'close_finish'})
                smach.StateMachine.add('CLOSE_SECTION', close_sub,transitions={'close_finish': 'stack_and_move_finish'})
                # 嵌套
            smach.StateMachine.add('STACK_AND_MOVE_SECTION', stack_and_move_sub_sm, transitions={'stack_and_move_finish': 'GLOBAL_PICKUP_3'})
            # 嵌套

            # 倒车 每次要多往后移动
            smach.StateMachine.add('GLOBAL_PICKUP_3', GlobalPickupTask_2(control_task=2, limited_speed=0.8), transitions={'finish': 'ADJUST_HEADING'})

            if not use_sim:
                # 调整航向
                smach.StateMachine.add('ADJUST_HEADING', AdjustHeading(goal='observer_point', steering_angle=adjust_steering_angle, changeYawangle=-0, gear=3, throttle=0.5, control_com=12),
                                        # 都是角度制
                                        transitions={'finish': 'finish'})
                smach.StateMachine.add('ADJUST_DESIGN_HEADING', AdjustHeading_Design_Angle(angle=90,steering_angle=adjust_steering_angle, gear=3, throttle=0.4, control_com=6),transitions={'finish': 'finish'})
            else:
                # 调整航向
                smach.StateMachine.add('ADJUST_HEADING', AdjustHeading(goal='observer_point', steering_angle=adjust_steering_angle_sim, changeYawangle=-0, gear=3, throttle=0.4, control_com=0),transitions={'finish': 'finish'})
                smach.StateMachine.add('ADJUST_DESIGN_HEADING', AdjustHeading_Design_Angle(angle=90,steering_angle=adjust_steering_angle_sim, gear=3, throttle=0.4, control_com=0),transitions={'finish': 'finish'})

        smach.StateMachine.add('SUB_STATE_MACHINE', sub_sm, transitions={'finish': 'LidarObstacleState'})
        # 标定点位
        smach.StateMachine.add('LidarObstacleState', LidarObstacle(lidar_obs=1), transitions={'finish': 'AdjustHeading1'})
        smach.StateMachine.add('AdjustHeading1', AdjustHeading_Design_Angle(angle=90,steering_angle=adjust_steering_angle, gear=3, throttle=0.3, control_com=6),transitions={'finish': 'HeightRaiseFinal1'})
        # smach.StateMachine.add('AdjustHeading1', AdjustHeading(goal='delivery_point', steering_angle=adjust_steering_angle, changeYawangle=0, gear=3, throttle=0.3, control_com=8),transitions={'finish': 'WidthClose1'})
        if not use_sim:
            smach.StateMachine.add('HeightRaiseFinal1', SetHeight(single_cargo_height_half, updown_mode=2), transitions={'finish': 'WidthClose1'})
            smach.StateMachine.add('WidthClose1', SetWidth(open_width_-30), transitions={'finish': 'HeightRaiseFinal2'})
            smach.StateMachine.add('HeightRaiseFinal2', SetHeight(above_two_cargos_height_max, updown_mode=2), transitions={'finish': 'ADJUST_DESIGN_HEADING1'})
        else:
            smach.StateMachine.add('HeightRaiseFinal1', SetHeightSim(open_width_sim), transitions={'finish': 'WidthClose1'})
            smach.StateMachine.add('WidthClose1', SetWidthSim(clamping_width_sim), transitions={'finish': 'HeightRaiseFinal2'})
            smach.StateMachine.add('HeightRaiseFinal2', SetHeightSim(single_cargo_height_sim), transitions={'finish': 'ADJUST_DESIGN_HEADING1'})

        smach.StateMachine.add('ADJUST_DESIGN_HEADING1', AdjustHeading_Design_Angle(angle=-90,steering_angle=adjust_steering_angle, gear=1, throttle=0.3, control_com=6),transitions={'finish': 'GLOBAL_PICKUP_4'})
        # 继续夹两个放置  回到起点
        smach.StateMachine.add('GLOBAL_PICKUP_4', GlobalPickupTask_3(control_task=1, limited_speed=0.8), transitions={'finish': 'SUB_STATE_MACHINE1'})
        
        smach.StateMachine.add('SUB_STATE_MACHINE1', sub_sm, transitions={'finish': 'LidarObstacleState1'})
        smach.StateMachine.add('LidarObstacleState1', LidarObstacle(lidar_obs=1), transitions={'finish': 'AdjustHeading2'})
        smach.StateMachine.add('AdjustHeading2', AdjustHeading_Design_Angle(angle=90,steering_angle=adjust_steering_angle, gear=3, throttle=0.3, control_com=6),transitions={'finish': 'HeightRaiseFinal3'})
        
        if not use_sim:
            smach.StateMachine.add('HeightRaiseFinal3', SetHeight(single_cargo_height_half, updown_mode=2), transitions={'finish': 'WidthClose2'})
            smach.StateMachine.add('WidthClose2', SetWidth(open_width_-30), transitions={'finish': 'HeightRaiseFinal4'})
            smach.StateMachine.add('HeightRaiseFinal4', SetHeight(above_two_cargos_height_max, updown_mode=2), transitions={'finish': 'ADJUST_DESIGN_HEADING2'})
        else:
            smach.StateMachine.add('HeightRaiseFinal3', SetHeightSim(single_cargo_height_sim), transitions={'finish': 'WidthClose2'})
            smach.StateMachine.add('WidthClose2', SetWidthSim(clamping_width_sim), transitions={'finish': 'HeightRaiseFinal4'})
            smach.StateMachine.add('HeightRaiseFinal4', SetHeightSim(open_width_sim), transitions={'finish': 'ADJUST_DESIGN_HEADING2'})

        smach.StateMachine.add('ADJUST_DESIGN_HEADING2', AdjustHeading_Design_Angle(angle=90,steering_angle=adjust_steering_angle, gear=1, throttle=0.3, control_com=6),transitions={'finish': 'FINISH'})
        smach.StateMachine.add('FINISH', Finish(), transitions={'succeed':'succeed'})
    #===========================Smach==================================
    return sm