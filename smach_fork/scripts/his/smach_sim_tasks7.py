#!/usr/bin/env python3

import rospy
import smach
from std_msgs.msg import Float64,UInt8, Int8
from car_interfaces.msg import Decision
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import sys
import time
from car_interfaces.srv import GetInitPose, GetTargetPose, GetInitPoseRequest, GetTargetPoseRequest, BalesNumInCamera, BalesNumInCameraRequest,BalesNumInCameraResponse
from car_interfaces.srv import PlanTask, ControlTask, CameraTask, PositionTask, PositionTaskRequest, PositionTaskResponse
from v2n.srv import cloud_order, cloud_orderResponse
from car_interfaces.srv import TaskSts, TaskStsRequest
from car_interfaces.msg import pose
from car_interfaces.srv import FusionDetTask, FusionDetTaskResponse

from sensor_msgs.msg import JointState
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

# 父类,初始化服务端和客户端
class SerManager:
    def __init__(self):
        self.position_state = 0  # 初始化 position_state 属性
        self.brake_enable = 0
        self.decision_sub_ = rospy.Subscriber('/emergency_brake_from_decision', Decision, self.decision_callback)
        self.position_state_service = None
        self.bales_num_in_camera_server = None
        self.bales_num_in_camera = 0
        self.camera_state = 0
        self.control_state = 0
        self.plan_state = 0
    def decision_callback(self, msg):
        self.brake_enable = msg.brake_enable

    # 接收识别的棉包数
    def handle_bales_num_state(self, req):
        rospy.loginfo_once("request camera: %d", req.data)
        self.bales_num_in_camera = req.data
        resp = BalesNumInCameraResponse()
        resp.success = True
        resp.message = "Position state updated successfully."
        return resp
    # 服务端接收终点的位置信息：
    def handle_position_state(self, req):
        rospy.loginfo_once("request: x = %d", req.data)
        self.position_state = req.data
        resp = PositionTaskResponse()
        resp.success = True
        resp.message = "Position state updated successfully."
        return resp

    def send_plan_task_service(self, request_value):
        rospy.loginfo("\033[31m 等待规划 server... \033[0m ")
        rospy.wait_for_service('plan_task')
        rospy.loginfo("\033[32m"+"Sending 规划 server ready!"+"\033[0m")
        try:
            client = rospy.ServiceProxy("plan_task", PlanTask)
            resp = client(request_value)
            if resp.success:
                rospy.loginfo("规划反馈 successful")
            else:
                rospy.loginfo("Failed to plan_task: %s", resp.message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def send_control_task_service(self, request_value):
        rospy.loginfo("\033[31m 等待控制 server... \033[0m ")
        rospy.wait_for_service('control_task')
        rospy.loginfo("\033[32m"+"Sending control_task server ready!"+"\033[0m")
        try:
            client = rospy.ServiceProxy("control_task", ControlTask)
            resp = client(request_value)
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

class AdjustHeading_design_AngleDirect(smach.State,SerManager):
    def __init__(self, angle=None,gear=None, throttle=None, control_com=None):
        smach.State.__init__(self, outcomes=['braking', 'finish'])
        SerManager.__init__(self)
        # 初始化相关变量
        self.conveyer_belt_yaw = None
        self.current_yaw = None
        self.steering_state = None
        self.dest_gear=gear
        # rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.steering_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=1)
        rospy.Subscriber('/steering_state', Float64, self.steering_callback)
        self.throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=1)
        self.gear_pub = rospy.Publisher('/gear_cmd', UInt8, queue_size=1)
        self.brake_pub = rospy.Publisher('/brake_cmd', Float64, queue_size=1)

        self.adjust_steering_angle = rospy.get_param("/smach/adjust_steering_angle", 1.22)
        self.adjust_throttle = rospy.get_param("/smach/adjust_throttle", 0.05)

        self.path_publisher = rospy.Publisher('/smach_path', Path, queue_size=1)

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

    def get_yaw_from_quaternion(self, quaternion):
        _, _, yaw = tf.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return yaw
    
    def calculate_angle(self, current_yaw, target_yaw):
        diff = target_yaw - current_yaw
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff


    def execute(self, userdata):
            global current_yaw, current_x, current_y
            
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

            goal_yaw = self.current_yaw+self.goal_yaw * math.pi / 180
            direction, self.need_diff = calculate_optimal_rotation(self.current_yaw * 180 / math.pi, goal_yaw * 180 / math.pi)
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
                        if abs(self.calculate_angle(self.init_yaw, self.current_yaw)) <= (abs(self.need_diff)-self.control_com): # 少转一点，补偿90度
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

            if not self.brake_enable: 
                rospy.loginfo_once("\033[33m"+'Adjust yaw success!!!'+"\033[0m")
                self.change_angle = True
                self.init_yaw = None
                self.start_time = None
                self.steering_state = None
                return 'finish'
            # 如果有刹车
            elif self.brake_enable:
                rospy.loginfo_once('AEB')
                self.change_angle = True
                self.init_yaw = None
                self.start_time = None
                self.steering_state = None
                userdata.last_state = 'MOVING_TO_PICKUP_LOCATION'
                return 'braking'

class AdjustHeading_Conveyer(smach.State, SerManager):
    def __init__(self, goal=None, changeYawangle=None,gear=None, throttle=None,control_com=None):
        smach.State.__init__(self, outcomes=['braking', 'finish'])
        SerManager.__init__(self)
        # 初始化相关变量
        self.conveyer_belt_yaw = None
        self.current_yaw = None
        self.steering_state = None
        self.dest_gear=gear
        # rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.steering_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=1)
        rospy.Subscriber('/steering_state', Float64, self.steering_callback)
        self.throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=1)
        self.gear_pub = rospy.Publisher('/gear_cmd', UInt8, queue_size=1)
        self.brake_pub = rospy.Publisher('/brake_cmd', Float64, queue_size=1)
        self.path_publisher = rospy.Publisher('/smach_path', Path, queue_size=1)

        self.adjust_steering_angle = rospy.get_param("/smach/adjust_steering_angle", 1.22)
        self.adjust_throttle = rospy.get_param("/smach/adjust_throttle", 0.05)
        self.throttle = throttle

        self.conveyer_belt_point = 0
        self.delivery_point_mode = 0
        self.control_com = control_com
        self.changeYawangle = changeYawangle # 加为逆时针，减为顺时针
        self.goal = None
        if goal == 'delivery_point':
            self.delivery_point_mode = 1
        elif goal=="conveyer_belt_point":
            self.delivery_point_mode = 2
        self.angle_diff = 0
        self.init_yaw = None
        self.change_angle = True
        self.start_time = None
        self.goal_yaw = 0

    def steering_callback(self, msg):
        self.steering_state = msg.data

    def get_yaw_from_quaternion(self, quaternion):
        _, _, yaw = tf.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return yaw
    
    def calculate_angle(self, current_yaw, target_yaw):
        diff = target_yaw - current_yaw
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff


    def execute(self, userdata):
        global current_yaw, current_x, current_y
        global delivery_point, conveyer_belt_point
        global cotton_cross_product

        # 获取目标的位置和航向
        if self.delivery_point_mode == 1:
            self.goal = delivery_point
        elif self.delivery_point_mode == 2:
            self.goal = conveyer_belt_point

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
        
        # cotton_cross_product = 1
        # 按照叉乘来修改转向值 根据在出包口的左边和右边来修改转向的方向
        if(cotton_cross_product<0):
            # 右边
            self.changeYawangle = self.changeYawangle
        else:
            # 左边
            self.changeYawangle = -self.changeYawangle

        goal_yaw = self.goal_yaw + self.changeYawangle * math.pi / 180
        direction, self.need_diff = calculate_optimal_rotation(self.current_yaw*180/math.pi, goal_yaw*180/math.pi)

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
                    if abs(self.calculate_angle(self.init_yaw, self.current_yaw)) <= (abs(self.need_diff)-self.control_com): # 少转一点，补偿90度
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

                count += 1
                if count % 100 == 0:
                    rospy.loginfo("\033[32m"+"NOW TURN: " + f"{abs(self.calculate_angle(self.init_yaw, self.current_yaw)):.3f}" + "\033[0m")
                    rospy.loginfo("\033[32m"+"NEED TURN: " + f"{abs(self.need_diff):.3f}" + "\033[0m")
                    rospy.loginfo("\033[32m"+"Wheel Angle pub: " + f"{adjust_steering_angle_:.3f}" + "\033[0m")
            
            rospy.Rate(100).sleep()

        if not self.brake_enable: 
            rospy.loginfo_once("\033[33m"+'Adjust yaw success!!!'+"\033[0m")
            self.change_angle = True
            self.init_yaw = None
            self.start_time = None
            return 'finish'
        # 如果有刹车
        elif self.brake_enable:
            rospy.loginfo_once('AEB')
            self.change_angle = True
            self.init_yaw = None
            self.start_time = None
            userdata.last_state = 'MOVING_TO_PICKUP_LOCATION'
            return 'braking'


class AdjustHeading(smach.State, SerManager):
    def __init__(self, goal=None, changeYawangle=None,gear=None, throttle=None,control_com=None):
        smach.State.__init__(self, outcomes=['braking', 'finish'])
        SerManager.__init__(self)
        # 初始化相关变量
        self.conveyer_belt_yaw = None
        self.current_yaw = None
        self.steering_state = None
        self.dest_gear=gear
        # rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.steering_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=1)
        rospy.Subscriber('/steering_state', Float64, self.steering_callback)
        self.throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=1)
        self.gear_pub = rospy.Publisher('/gear_cmd', UInt8, queue_size=1)
        self.brake_pub = rospy.Publisher('/brake_cmd', Float64, queue_size=1)
        self.path_publisher = rospy.Publisher('/smach_path', Path, queue_size=1)

        self.adjust_steering_angle = rospy.get_param("/smach/adjust_steering_angle", 1.22)
        self.adjust_throttle = rospy.get_param("/smach/adjust_throttle", 0.05)
        self.throttle = throttle

        self.conveyer_belt_point = 0
        self.delivery_point_mode = 0
        self.control_com = control_com
        self.changeYawangle = changeYawangle # 加为逆时针，减为顺时针
        self.goal = None
        if goal == 'delivery_point':
            self.delivery_point_mode = 1
        elif goal=="conveyer_belt_point":
            self.delivery_point_mode = 2
        elif goal=="observer_point":
            self.delivery_point_mode = 3

        
        self.angle_diff = 0
        self.init_yaw = None
        self.change_angle = True
        self.start_time = None
        self.goal_yaw = 0

    def steering_callback(self, msg):
        self.steering_state = msg.data

    def get_yaw_from_quaternion(self, quaternion):
        _, _, yaw = tf.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return yaw
    
    def calculate_angle(self, current_yaw, target_yaw):
        diff = target_yaw - current_yaw
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff


    def execute(self, userdata):
        global current_yaw, current_x, current_y
        global delivery_point, conveyer_belt_point
        global cotton_cross_product,observer_point

        if self.delivery_point_mode == 1:
            self.goal = delivery_point
        elif self.delivery_point_mode == 2:
            self.goal = conveyer_belt_point
        elif self.delivery_point_mode == 3:
            self.goal = observer_point

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
        

        goal_yaw = self.goal_yaw + self.changeYawangle * math.pi / 180
        direction, self.need_diff = calculate_optimal_rotation(self.current_yaw*180/math.pi, goal_yaw*180/math.pi)

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
                    if abs(self.calculate_angle(self.init_yaw, self.current_yaw)) <= (abs(self.need_diff)-self.control_com): # 少转一点，补偿90度
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

                count += 1
                if count % 100 == 0:
                    rospy.loginfo("\033[32m"+"NOW TURN: " + f"{abs(self.calculate_angle(self.init_yaw, self.current_yaw)):.3f}" + "\033[0m")
                    rospy.loginfo("\033[32m"+"NEED TURN: " + f"{abs(self.need_diff):.3f}" + "\033[0m")
                    rospy.loginfo("\033[32m"+"Wheel Angle pub: " + f"{adjust_steering_angle_:.3f}" + "\033[0m")
            rospy.Rate(100).sleep()

        if not self.brake_enable: 
            rospy.loginfo_once("\033[33m"+'Adjust yaw success!!!'+"\033[0m")
            self.change_angle = True
            self.init_yaw = None
            self.start_time = None
            return 'finish'
        # 如果有刹车
        elif self.brake_enable:
            rospy.loginfo_once('AEB')
            self.change_angle = True
            self.init_yaw = None
            self.start_time = None
            userdata.last_state = 'MOVING_TO_PICKUP_LOCATION'
            return 'braking'


class GlobalBackTask0(smach.State, SerManager):
    def __init__(self, goal=None, changeYawangle=None,gear=None, throttle=None,control_com=None):
        smach.State.__init__(self, outcomes=['braking', 'finish'])
        SerManager.__init__(self)
        # 初始化相关变量
        self.conveyer_belt_yaw = None
        self.current_yaw = None
        self.steering_state = None
        self.dest_gear=gear
        # rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.steering_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=1)
        rospy.Subscriber('/steering_state', Float64, self.steering_callback)
        self.throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=1)
        self.gear_pub = rospy.Publisher('/gear_cmd', UInt8, queue_size=1)
        self.brake_pub = rospy.Publisher('/brake_cmd', Float64, queue_size=1)
        self.path_publisher = rospy.Publisher('/smach_path', Path, queue_size=1)

        self.adjust_steering_angle = rospy.get_param("/smach/adjust_steering_angle", 1.22)
        self.adjust_throttle = rospy.get_param("/smach/adjust_throttle", 0.05)
        self.throttle = throttle

        self.conveyer_belt_point = 0
        self.delivery_point_mode = 0
        self.control_com = control_com
        self.changeYawangle = changeYawangle # 加为逆时针，减为顺时针
        self.goal = None
        if goal == 'delivery_point':
            self.delivery_point_mode = 1
        elif goal=="conveyer_belt_point":
            self.delivery_point_mode = 2
        self.angle_diff = 0
        self.init_yaw = None
        self.change_angle = True
        self.start_time = None
        self.goal_yaw = 0

    def steering_callback(self, msg):
        self.steering_state = msg.data

    def get_yaw_from_quaternion(self, quaternion):
        _, _, yaw = tf.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return yaw
    
    def calculate_angle(self, current_yaw, target_yaw):
        diff = target_yaw - current_yaw
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff


    def execute(self, userdata):
        global current_yaw, current_x, current_y
        global delivery_point, conveyer_belt_point
        global cotton_cross_product

        # 获取目标的位置和航向
        if self.delivery_point_mode == 1:
            self.goal = delivery_point
        elif self.delivery_point_mode == 2:
            self.goal = conveyer_belt_point

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

        goal_yaw = self.goal_yaw + self.changeYawangle * math.pi / 180
        direction, self.need_diff = calculate_optimal_rotation(self.current_yaw*180/math.pi, goal_yaw*180/math.pi)

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
                    throttle_cmd = Float64(0.0)
                    self.throttle_pub.publish(throttle_cmd)
                    gear_cmd = UInt8(self.dest_gear)
                    self.gear_pub.publish(gear_cmd)
                    if steering_diff < 0.1:
                        self.change_angle = False
                else:
                    # 2累计转过的 yaw，如果大于等于 self.angle_diff 先把角度转回来，然后停止
                    if abs(self.calculate_angle(self.init_yaw, self.current_yaw)) <= (abs(self.need_diff)-self.control_com): # 少转一点，补偿90度
                        steering_cmd = Float64(adjust_steering_angle_)
                        self.steering_pub.publish(steering_cmd)
                        self.brake_pub.publish(0.0)
                        throttle_cmd = Float64(self.throttle)
                        self.throttle_pub.publish(throttle_cmd)
                        gear_cmd = UInt8(self.dest_gear)
                        self.gear_pub.publish(gear_cmd)
                    else:
                        # 到位了之后再等待1s
                        steering_cmd = Float64(0.0)
                        self.brake_pub.publish(0.0)
                        self.steering_pub.publish(0.0)
                        throttle_cmd = Float64(0.0)
                        self.throttle_pub.publish(0)
                        gear_cmd = UInt8(2)
                        self.gear_pub.publish(gear_cmd)
                        if self.start_time is None:
                            self.start_time = rospy.Time.now()
                        if (rospy.Time.now() - self.start_time).to_sec() > 1.0:
                            rospy.loginfo("\033[32m"+"last diff: " + str(abs(self.calculate_angle(self.init_yaw, self.current_yaw))) + "\033[0m")
                            break

                count += 1
                if count % 100 == 0:
                    rospy.loginfo("\033[32m"+"NOW TURN: " + f"{abs(self.calculate_angle(self.init_yaw, self.current_yaw)):.3f}" + "\033[0m")
                    rospy.loginfo("\033[32m"+"NEED TURN: " + f"{abs(self.need_diff):.3f}" + "\033[0m")
                    rospy.loginfo("\033[32m"+"Wheel Angle pub: " + f"{adjust_steering_angle_:.3f}" + "\033[0m")
            rospy.Rate(100).sleep()

        if not self.brake_enable: 
            rospy.loginfo_once("\033[33m"+'Adjust yaw success!!!'+"\033[0m")
            self.change_angle = True
            self.init_yaw = None
            self.start_time = None
            return 'finish'
        # 如果有刹车
        elif self.brake_enable:
            rospy.loginfo_once('AEB')
            self.change_angle = True
            self.init_yaw = None
            self.start_time = None
            userdata.last_state = 'MOVING_TO_PICKUP_LOCATION'
            return 'braking'



class GlobalPickupTask(smach.State, SerManager):
    def __init__(self , goal=None):
        smach.State.__init__(self, outcomes=['braking', 'set_width_height'], input_keys=['points'])
        SerManager.__init__(self)
        # self.goal = goal  # 保存目标点数据
        # 标定
        self.current_yaw_reverse_distance = rospy.get_param("/smach/current_yaw_reverse_distance", 0.8)
        self.goal_yaw_reverse_distance = rospy.get_param("/smach/goal_yaw_reverse_distance", -0.8)
        self.new_point_reindex = []

        if goal == 'delivery_point':
            self.delivery_point_mode = 1
        elif goal=="conveyer_belt_point":
            self.delivery_point_mode = 2


    def execute(self, userdata):
        global num_work
        global position_, bales_num_
        global current_x, current_y, current_yaw
        global delivery_point, conveyer_belt_point
        global cotton_cross_product

        # 获取目标的位置和航向
        if self.delivery_point_mode == 1:
            self.goal = delivery_point
        elif self.delivery_point_mode == 2:
            self.goal = conveyer_belt_point
        else:
            rospy.logwarn("Invalid parameter for conveyer_belt_point.")
        
        rospy.loginfo_once("\033[33m"+f' 任务目标: {self.goal} '+ "\033[0m")

        self.task = 1
        self.send_plan_task_service(self.task)
        self.send_control_task_service(3)

        # 等待最新的定位点
        time.sleep(1)
        print(self.delivery_point_mode)
        goal_point1 = self.goal[0]
        goal_point2 = self.goal[1]
        # 从四元数获取目标点的航向
        goal_quaternion = goal_point2
        _, _, goal_yaw = tf.euler_from_quaternion([goal_quaternion[0], goal_quaternion[1], goal_quaternion[2], goal_quaternion[3]])

        goal_slope = math.tan(goal_yaw)
        current_slope = math.tan(current_yaw)
        # 计算交点
        if goal_slope == current_slope:
            rospy.logerr_once("slope error")
        else:
            x1, y1, _ = goal_point1
            x2, y2 = current_x, current_y
            intersection_x = (y2 - y1 + x1 * goal_slope - x2 * current_slope) / (goal_slope - current_slope)
            intersection_y = y1 + goal_slope * (intersection_x - x1)
        # 交点
        current_yaw_point = process_point(intersection_x, intersection_y, current_yaw)
        goal_yaw_point = process_point(intersection_x, intersection_y, goal_yaw)


        # 途经点 标定的转向参数
        reversed_current_yaw_point = calculate_reverse_point(current_yaw_point, self.current_yaw_reverse_distance)
        reversed_goal_yaw_point = calculate_reverse_point(goal_yaw_point, self.goal_yaw_reverse_distance)
        self.new_point_reindex.append(reversed_current_yaw_point)
        self.new_point_reindex.append(reversed_goal_yaw_point)

        # print("reversed_current_yaw_point", reversed_current_yaw_point)
        # print("reversed_goal_yaw_point", reversed_goal_yaw_point)
        # print("self.goal", self.goal)

        for point_data, quaternion_data in self.new_point_reindex:
            point = Point(*point_data)
            quaternion = Quaternion(*quaternion_data)
            self.send_init_pose(point, quaternion)
            rospy.loginfo("\033[32m"+"途经点位发布成功"+"\033[0m")

        # new_point_right = calculate_new_point(self.goal, 1, distance*num_work)
        rospy.loginfo("\033[32m"+"下发目标点位"+"\033[0m")
        goal_point, goal_quaternion = self.goal # new_point_right
        goal_pose = Point(goal_point[0], goal_point[1], goal_point[2])
        goal_orientation = Quaternion(goal_quaternion[0], goal_quaternion[1], goal_quaternion[2], goal_quaternion[3])
        self.send_target_pose(goal_pose, goal_orientation)
        rospy.loginfo("\033[32m"+"目标点位发布成功"+"\033[0m")

        count = 0 
        rospy.loginfo("position_: %d", position_)
        while position_ != 1 and not self.brake_enable:
            count += 1
            if count % 100 == 0:
                rospy.loginfo_once("\033[32m"+"Go to global endpoint" "\033[0m")
            rospy.Rate(100).sleep()

        # 到达终点->开始夹取
        if position_ == 1 and not self.brake_enable: 
            rospy.loginfo_once("\033[32m"+'Position reached, switching to local_planning'+"\033[0m")
            self.task = 0
            self.send_plan_task_service(self.task)
            self.send_control_task_service(self.task)
            position_ = 0
            bales_num_ = 0
            self.new_point_pickup = []
            self.new_point_reindex = []
            return 'set_width_height'
        # 如果有刹车
        elif self.brake_enable:
            rospy.loginfo_once('AEB')
            userdata.last_state = 'MOVING_TO_PICKUP_LOCATION'
            return 'braking'


class GlobalMoveTask(smach.State, SerManager):
    def __init__(self, target_distance=None, Throttle=None, tar_height=None, updown_mode=None,tar_width=None):
        smach.State.__init__(self, outcomes=['braking', 'set_width_height'], input_keys=['points'])
        SerManager.__init__(self)
        self.target_distance = target_distance
        self.start_position = None
        self.path_publisher = rospy.Publisher('/smach_path', Path, queue_size=1)
        self.Throttle = Throttle
        self.steering_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=1)
        self.throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=1)
        self.gear_pub = rospy.Publisher('/gear_cmd', UInt8, queue_size=1)
        self.brake_pub = rospy.Publisher('/brake_cmd', Float64, queue_size=1)
        
        self.tar_height = tar_height
        self.tar_width = tar_width
        # self.timer = timer
        self.set_tar_height_pub = rospy.Publisher('/updown_cmd', Float64, queue_size=1)
        self.set_tar_height_pub_ = rospy.Publisher('/updown_mode', Int8, queue_size=1)
        
        self.updown_mode = updown_mode
        rospy.Subscriber('/updown_state', Float64, self.updown_state_callback)
        rospy.Subscriber('/clamp_state', Float64, self.clamp_state_callback)

        self.set_tar_width_pub = rospy.Publisher('/clamp_cmd', Float64, queue_size=1)
        self.set_tar_fy_pub = rospy.Publisher('/fy_cmd', Float64, queue_size=1)
        self.set_tar_lateral_pub = rospy.Publisher('/lateral_cmd', Float64, queue_size=1)
        rospy.Subscriber('/ipc_state', Int8, self.ipc_state_callback)
        self.path_publisher = rospy.Publisher('/smach_path', Path, queue_size=1)

        self.cur_height = 0
        self.ipc_state = 0
        self.start_time = None
        self.previous_changed_height = None
        self.last_check_time = None

        self.start_time_2 = None
        self.last_check_time_2 = None

        self.cur_width = 0
        self.previous_changed_width = None

    def ipc_state_callback(self, msg):
        self.ipc_state = msg.data

    def clamp_state_callback(self, msg):
        self.cur_width = msg.data

    def updown_state_callback(self, msg):
        self.cur_height = msg.data
        self.changed_height = abs(self.tar_height - self.cur_height)


    def execute(self, userdata):
        global num_work
        global position_, bales_num_
        global current_x, current_y, current_yaw

        time.sleep(1)
        if self.start_position is None:
            self.start_position = Point(current_x, current_y, 0.0)

        if self.target_distance < 0:
            GEAR = 1
            rospy.loginfo("\033[33m"+"倒车"+"\033[0m")
        else:
            GEAR = 3
            rospy.loginfo("\033[33m"+"前进"+"\033[0m")


        count = 0  # 初始化计数器
        path_msg = Path()
        path_msg.header.frame_id = 'map'

        # 根据目标距离和步长计算路径点数量
        num_points = int(abs(self.target_distance) / 0.1)
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
            # print("new_x: ", new_x)
            # print("new_y: ", new_y)

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


        def first_loop():
            count = 0
            while True:
                if (self.tar_width==0):
                    break
                # 先循环发布210才能进无人使能
                self.set_tar_width_pub.publish(self.tar_width)
                if (self.ipc_state == 1):
                    if self.start_time is None:
                        self.start_time = rospy.Time.now()
                    # 先循环发布一段时间
                    self.set_tar_width_pub.publish(self.tar_width)
                    # 0.5s 过后，每 0.1s 检查一次宽度变化
                    if (rospy.Time.now() - self.start_time).to_sec() > 1.0:
                        current_time = rospy.Time.now()
                        if self.last_check_time is None or (current_time - self.last_check_time).to_sec() >= 1:
                            current_changed_width = abs(self.tar_width - self.cur_width)
                            if self.previous_changed_width is not None:
                                difference = abs(current_changed_width - self.previous_changed_width)
                                # print("difference: ", difference)
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

        def second_loop():
            count2 = 0
            while True:
                if (self.tar_height==0):
                    break
                # 先循环发布210才能进无人使能
                self.set_tar_height_pub.publish(self.tar_height)
                self.set_tar_height_pub_.publish(self.updown_mode)
                
                if (self.ipc_state == 1):
                    if self.start_time_2 is None:
                        self.start_time_2 = rospy.Time.now()
                    # 先循环发布一段时间
                    self.set_tar_height_pub.publish(self.tar_height)
                    self.set_tar_height_pub_.publish(self.updown_mode)
                    if (rospy.Time.now() - self.start_time_2).to_sec() > 1.0:
                        current_time = rospy.Time.now()
                        if self.last_check_time_2 is None or (current_time - self.last_check_time_2).to_sec() >= 1.0:
                            current_changed_height = abs(self.tar_height - self.cur_height)
                            if self.previous_changed_height is not None:
                                difference = abs(current_changed_height - self.previous_changed_height)
                                if difference < 10:
                                    self.start_time_2 = None
                                    self.last_check_time_2 = None
                                    self.previous_changed_height = None
                                    break
                                self.previous_changed_height = current_changed_height
                            else:
                                self.previous_changed_height = current_changed_height
                            self.last_check_time_2 = current_time
                    count2 += 1
                    if count2 % 100 == 0:
                        rospy.loginfo(f" \033[32m cur_height:{(self.cur_height):.3f} \033[0m")
                        rospy.loginfo(f" \033[32m tar_height:{(self.tar_height):.3f} \033[0m")
                else:
                    rospy.loginfo_once("\033[31m"+"Wait For auto_drive"+"\033[0m")
                rospy.Rate(100).sleep()

        def third_loop():
            count = 0
            while True:
                count += 1  # 每次循环递增计数器
                
                if count % 50 == 0:  # 每隔100次打印一次
                    rospy.loginfo("\033[34m" + f"Target:{self.target_distance:.3f}" "\033[0m")
                    rospy.loginfo("\033[34m" + f"Current:{self.distance:.3f}" "\033[0m")

                current_position = Point(current_x, current_y, 0.0)
                dx = current_position.x - self.start_position.x
                dy = current_position.y - self.start_position.y
                self.distance = ((dx ** 2) + (dy ** 2)) ** 0.5  

                self.steering_pub.publish(0.0)
                throttle_cmd = Float64(self.Throttle)
                self.throttle_pub.publish(throttle_cmd)
                gear_cmd = UInt8(GEAR)
                self.gear_pub.publish(gear_cmd)
                brake_cmd = Float64(0.0)
                self.brake_pub.publish(brake_cmd)
                # 发布路径消息
                self.path_publisher.publish(path_msg)
                
                if self.distance >= abs(self.target_distance):
                    self.steering_pub.publish(0.0)
                    self.brake_pub.publish(4.0)
                    self.throttle_pub.publish(0.0)
                    self.gear_pub.publish(2)
                    # 要注意，如果动作没有到位还是要继续发的，所以会继续监测
                    if self.start_time is None:
                        self.start_time = rospy.Time.now()
                    if (rospy.Time.now() - self.start_time).to_sec() > 0.5:
                        rospy.loginfo("\033[33m" + f"last Current:{self.distance:.3f}" "\033[0m")
                        break
                rospy.Rate(100).sleep()

        thread1 = threading.Thread(target=first_loop)
        thread2 = threading.Thread(target=second_loop)
        thread3 = threading.Thread(target=third_loop)
        thread1.start()
        thread2.start()
        thread3.start()
        thread1.join()
        thread2.join()
        thread3.join()


        # 到达终点->开始夹取
        if not self.brake_enable: 
            self.start_position = None
            rospy.loginfo_once("\033[32m"+'Position reached, switching to local_planning'+"\033[0m")
            return 'set_width_height'
        # 如果有刹车
        elif self.brake_enable:
            self.start_position = None
            rospy.loginfo_once('AEB')
            userdata.last_state = 'MOVING_TO_PICKUP_LOCATION'
            return 'braking'

class GlobalPickupTask_4(smach.State, SerManager):
    def __init__(self, goal=None, Throttle=None, tar_height=None, updown_mode=None,tar_width=None):
        smach.State.__init__(self, outcomes=['braking', 'set_width_height'], input_keys=['points'])
        SerManager.__init__(self)
        self.target_distance = 0
        self.start_position = None
        self.path_publisher = rospy.Publisher('/smach_path', Path, queue_size=1)
        self.Throttle = Throttle
        self.steering_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=1)
        self.throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=1)
        self.gear_pub = rospy.Publisher('/gear_cmd', UInt8, queue_size=1)
        self.brake_pub = rospy.Publisher('/brake_cmd', Float64, queue_size=1)
        
        self.tar_height = tar_height
        self.tar_width = tar_width
        # self.timer = timer
        self.set_tar_height_pub = rospy.Publisher('/updown_cmd', Float64, queue_size=1)
        self.set_tar_height_pub_ = rospy.Publisher('/updown_mode', Int8, queue_size=1)
        
        self.updown_mode = updown_mode
        rospy.Subscriber('/updown_state', Float64, self.updown_state_callback)
        rospy.Subscriber('/clamp_state', Float64, self.clamp_state_callback)

        self.set_tar_width_pub = rospy.Publisher('/clamp_cmd', Float64, queue_size=1)
        self.set_tar_fy_pub = rospy.Publisher('/fy_cmd', Float64, queue_size=1)
        self.set_tar_lateral_pub = rospy.Publisher('/lateral_cmd', Float64, queue_size=1)
        rospy.Subscriber('/ipc_state', Int8, self.ipc_state_callback)
        self.path_publisher = rospy.Publisher('/smach_path', Path, queue_size=1)

        self.cur_height = 0
        self.ipc_state = 0
        self.start_time = None
        self.previous_changed_height = None
        self.last_check_time = None

        self.start_time_2 = None
        self.last_check_time_2 = None

        self.cur_width = 0
        self.previous_changed_width = None
        self.goal = goal

        if goal == 'delivery_point':
            self.delivery_point_mode = 1
        elif goal=="conveyer_belt_point":
            self.delivery_point_mode = 2
        elif goal=="observer_point":
            self.delivery_point_mode = 3

    def ipc_state_callback(self, msg):
        self.ipc_state = msg.data

    def clamp_state_callback(self, msg):
        self.cur_width = msg.data

    def updown_state_callback(self, msg):
        self.cur_height = msg.data
        self.changed_height = abs(self.tar_height - self.cur_height)


    def execute(self, userdata):
        global num_work
        global position_, bales_num_
        global current_x, current_y, current_yaw,observer_point

        if self.delivery_point_mode == 1:
            self.goal = delivery_point
        elif self.delivery_point_mode == 2:
            self.goal = conveyer_belt_point
        elif self.delivery_point_mode == 3:
            self.goal = observer_point
        
        # print(self.delivery_point_mode)
        goal_point1 = self.goal[0]
        goal_point2 = self.goal[1]
        # 从四元数获取目标点的航向
        goal_quaternion = goal_point2
        _, _, goal_yaw = tf.euler_from_quaternion([goal_quaternion[0], goal_quaternion[1], goal_quaternion[2], goal_quaternion[3]])
        x1, y1, _ = goal_point1
        goal_slope = math.tan(goal_yaw)

        self.distance_result = point_to_line_distance(current_x, current_y, x1, y1, -1/goal_slope)
        rospy.loginfo("\033[33m" + f"Need Distance: {(self.distance_result):.3f}" "\033[0m")
        # self.target_distance = -self.distance_result
        self.target_distance = -self.distance_result - 4

        if self.start_position is None:
            self.start_position = Point(current_x, current_y, 0.0)

        if self.target_distance < 0:
            GEAR = 1
            rospy.loginfo("\033[33m"+"倒车"+"\033[0m")
        else:
            GEAR = 3
            rospy.loginfo("\033[33m"+"前进"+"\033[0m")

        count = 0  # 初始化计数器
        path_msg = Path()
        path_msg.header.frame_id = 'map'

        # 根据目标距离和步长计算路径点数量
        num_points = int(abs(self.target_distance) / 0.1)
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
            # print("new_x: ", new_x)
            # print("new_y: ", new_y)

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


        def first_loop():
            count = 0
            while True:
                if (self.tar_width==0):
                    break
                # 先循环发布210才能进无人使能
                self.set_tar_width_pub.publish(self.tar_width)
                if (self.ipc_state == 1):
                    if self.start_time is None:
                        self.start_time = rospy.Time.now()
                    # 先循环发布一段时间
                    self.set_tar_width_pub.publish(self.tar_width)
                    # 0.5s 过后，每 0.1s 检查一次宽度变化
                    if (rospy.Time.now() - self.start_time).to_sec() > 1.0:
                        current_time = rospy.Time.now()
                        if self.last_check_time is None or (current_time - self.last_check_time).to_sec() >= 1:
                            current_changed_width = abs(self.tar_width - self.cur_width)
                            if self.previous_changed_width is not None:
                                difference = abs(current_changed_width - self.previous_changed_width)
                                # print("difference: ", difference)
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

        def second_loop():
            count2 = 0
            while True:
                if (self.tar_height==0):
                    break
                # 先循环发布210才能进无人使能
                self.set_tar_height_pub.publish(self.tar_height)
                self.set_tar_height_pub_.publish(self.updown_mode)
                
                if (self.ipc_state == 1):
                    if self.start_time_2 is None:
                        self.start_time_2 = rospy.Time.now()
                    # 先循环发布一段时间
                    self.set_tar_height_pub.publish(self.tar_height)
                    self.set_tar_height_pub_.publish(self.updown_mode)
                    if (rospy.Time.now() - self.start_time_2).to_sec() > 1.0:
                        current_time = rospy.Time.now()
                        if self.last_check_time_2 is None or (current_time - self.last_check_time_2).to_sec() >= 1.0:
                            current_changed_height = abs(self.tar_height - self.cur_height)
                            if self.previous_changed_height is not None:
                                difference = abs(current_changed_height - self.previous_changed_height)
                                if difference < 10:
                                    self.start_time_2 = None
                                    self.last_check_time_2 = None
                                    self.previous_changed_height = None
                                    break
                                self.previous_changed_height = current_changed_height
                            else:
                                self.previous_changed_height = current_changed_height
                            self.last_check_time_2 = current_time
                    count2 += 1
                    if count2 % 100 == 0:
                        rospy.loginfo(f" \033[32m cur_height:{(self.cur_height):.3f} \033[0m")
                        rospy.loginfo(f" \033[32m tar_height:{(self.tar_height):.3f} \033[0m")
                else:
                    rospy.loginfo_once("\033[31m"+"Wait For auto_drive"+"\033[0m")
                rospy.Rate(100).sleep()

        def third_loop():
            count = 0
            while True:
                count += 1  # 每次循环递增计数器
                
                if count % 50 == 0:  # 每隔100次打印一次
                    rospy.loginfo("\033[34m" + f"Target:{self.target_distance:.3f}" "\033[0m")
                    rospy.loginfo("\033[34m" + f"Current:{self.distance:.3f}" "\033[0m")

                current_position = Point(current_x, current_y, 0.0)
                dx = current_position.x - self.start_position.x
                dy = current_position.y - self.start_position.y
                self.distance = ((dx ** 2) + (dy ** 2)) ** 0.5  

                self.steering_pub.publish(0.0)
                throttle_cmd = Float64(self.Throttle)
                self.throttle_pub.publish(throttle_cmd)
                gear_cmd = UInt8(GEAR)
                self.gear_pub.publish(gear_cmd)
                brake_cmd = Float64(0.0)
                self.brake_pub.publish(brake_cmd)
                # 发布路径消息
                self.path_publisher.publish(path_msg)
                
                if self.distance >= abs(self.target_distance):
                    self.steering_pub.publish(0.0)
                    self.brake_pub.publish(4.0)
                    self.throttle_pub.publish(0.0)
                    self.gear_pub.publish(2)
                    # 要注意，如果动作没有到位还是要继续发的，所以会继续监测
                    if self.start_time is None:
                        self.start_time = rospy.Time.now()
                    if (rospy.Time.now() - self.start_time).to_sec() > 0.5:
                        rospy.loginfo("\033[33m" + f"last Current:{self.distance:.3f}" "\033[0m")
                        break
                rospy.Rate(100).sleep()

        thread1 = threading.Thread(target=first_loop)
        thread2 = threading.Thread(target=second_loop)
        thread3 = threading.Thread(target=third_loop)
        thread1.start()
        thread2.start()
        thread3.start()
        thread1.join()
        thread2.join()
        thread3.join()


        # 到达终点->开始夹取
        if not self.brake_enable: 
            self.start_position = None
            rospy.loginfo_once("\033[32m"+'Position reached, switching to local_planning'+"\033[0m")
            return 'set_width_height'
        # 如果有刹车
        elif self.brake_enable:
            self.start_position = None
            rospy.loginfo_once('AEB')
            userdata.last_state = 'MOVING_TO_PICKUP_LOCATION'
            return 'braking'
        
class TemporaryParking1(smach.State, SerManager):
    def __init__(self, goal=None,throttle=None):
        smach.State.__init__(self, outcomes=['braking', 'set_width_height'], input_keys=['points'])
        SerManager.__init__(self)
        # self.points = points if points is not None else []
        self.goal = goal
        
        if goal == 'delivery_point':
            self.delivery_point_mode = 1
        elif goal=="conveyer_belt_point":
            self.delivery_point_mode = 2
        elif self.goal == "observer_point":
            self.delivery_point_mode = 3

        self.new_point_delivery = []
        self.new_point_reindex = []
        
        self.current_yaw_reverse_distance = rospy.get_param("/smach/current_yaw_reverse_distance", 0.8)
        self.goal_yaw_reverse_distance = rospy.get_param("/smach/goal_yaw_reverse_distance", -0.8)
        self.lateral_distance = rospy.get_param("/smach/lateral_distance", 1.0)
        self.path_publisher = rospy.Publisher("/smach_path", Path, queue_size=1)
        rospy.Subscriber('/steering_state', Float64, self.steering_callback)

        self.steering_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=1)
        self.throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=1)
        self.gear_pub = rospy.Publisher('/gear_cmd', UInt8, queue_size=1)
        self.brake_pub = rospy.Publisher('/brake_cmd', Float64, queue_size=1)
        self.start_position = None
        self.start_time = None
        self.new_point_back = []
        self.new_point_reindex = []

    def steering_callback(self, msg):
        self.steering_state = msg.data

    def execute(self, userdata):
        global position_, bales_num_
        global num_work,num_work_height
        global current_x, current_y, current_yaw, delivery_point, work_direction
        global num_work, num_work_max, num_work_height,work_direction
        global conveyer_belt_point,observer_point

        rospy.logwarn(f"num_work: {num_work}, num_work_height: {num_work_height}.")

        if self.delivery_point_mode == 1:
            self.goal = delivery_point
        elif self.delivery_point_mode == 2:
            self.goal = conveyer_belt_point
        if self.delivery_point_mode == 1:
            self.goal = delivery_point
        elif self.delivery_point_mode == 3:
            self.goal = observer_point

        else:
            rospy.logwarn("Invalid parameter for conveyer_belt_point.")

        rospy.loginfo_once("\033[32m"+'Executing state: TemporaryParking1 '+"\033[0m")
        rospy.loginfo("\033[32m"+"下发全局任务"+"\033[0m")
        self.task = 1
        
        self.send_plan_task_service(self.task)
        self.send_control_task_service(self.task)
        rospy.loginfo("\033[32m"+"任务下发成功"+"\033[0m")


        reversed_goal_point = calculate_reverse_point(observer_point, -5)
        reversed_reversed_goal_point = calculate_reverse_point(observer_point, -1)
        self.new_point_reindex.append(reversed_goal_point)

        for point_data, quaternion_data in self.new_point_reindex:
            point = Point(*point_data)
            quaternion = Quaternion(*quaternion_data)
            self.send_init_pose(point, quaternion)
            rospy.loginfo("\033[32m"+"途经点位发布成功"+"\033[0m")

        rospy.loginfo("\033[32m"+"下发目标点位"+"\033[0m")
        goal_point, goal_quaternion = observer_point#right_conveyer_belt_point
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
        self.send_control_task_service(0)
        self.new_point_delivery = []
        self.new_point_reindex = []

        # 到达终点->开始夹取
        if not self.brake_enable: 
            position_ = 0
            self.start_position = None
            self.start_time = None
            conveyer_belt_point = None
            rospy.loginfo_once("\033[32m"+'Position reached, switching to local_planning'+"\033[0m")
            return 'set_width_height'
        # 如果有刹车
        elif self.brake_enable:
            position_ = 0
            conveyer_belt_point = None
            self.start_position = None
            self.start_time = None
            rospy.loginfo_once('AEB')
            userdata.last_state = 'MOVING_TO_PICKUP_LOCATION'
            return 'braking'


class TemporaryParking2(smach.State, SerManager):
    def __init__(self, goal=None,throttle=None):
        smach.State.__init__(self, outcomes=['braking', 'set_width_height'], input_keys=['points'])
        SerManager.__init__(self)
        # self.points = points if points is not None else []
        self.goal = goal
        
        if goal == 'delivery_point':
            self.delivery_point_mode = 1
        elif goal=="conveyer_belt_point":
            self.delivery_point_mode = 2
        elif goal=="observer_point":
            self.delivery_point_mode = 3

        self.new_point_delivery = []
        self.new_point_reindex = []
        
        self.current_yaw_reverse_distance = rospy.get_param("/smach/current_yaw_reverse_distance", 0.8)
        self.goal_yaw_reverse_distance = rospy.get_param("/smach/goal_yaw_reverse_distance", -0.8)
        self.lateral_distance = rospy.get_param("/smach/lateral_distance", 1.0)
        self.path_publisher = rospy.Publisher("/smach_path", Path, queue_size=1)
        rospy.Subscriber('/steering_state', Float64, self.steering_callback)

        self.steering_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=1)
        self.throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=1)
        self.gear_pub = rospy.Publisher('/gear_cmd', UInt8, queue_size=1)
        self.brake_pub = rospy.Publisher('/brake_cmd', Float64, queue_size=1)
        self.start_position = None
        self.start_time = None
        self.new_point_back = []
        self.new_point_reindex = []

    def steering_callback(self, msg):
        self.steering_state = msg.data

    def execute(self, userdata):
        global position_, bales_num_
        global num_work,num_work_height
        global current_x, current_y, current_yaw, delivery_point, work_direction
        global num_work, num_work_max, num_work_height,work_direction
        global conveyer_belt_point, observer_point

        rospy.logwarn(f"num_work: {num_work}, num_work_height: {num_work_height}.")

        if self.delivery_point_mode == 1:
            self.goal = delivery_point
        elif self.delivery_point_mode == 2:
            self.goal = conveyer_belt_point
        elif self.delivery_point_mode == 3:
            self.goal = observer_point

        else:
            rospy.logwarn("Invalid parameter for conveyer_belt_point.")

        rospy.loginfo_once("\033[32m"+'Executing state: TemporaryParking2'+"\033[0m")
        rospy.loginfo("\033[32m"+"下发全局任务"+"\033[0m")
        self.task = 1
        
        self.send_plan_task_service(self.task)
        self.send_control_task_service(self.task)
        rospy.loginfo("\033[32m"+"任务下发成功"+"\033[0m")


        reversed_goal_point = calculate_reverse_point(observer_point, -5)
        reversed_reversed_goal_point = calculate_reverse_point(observer_point, 0.9)
        # right_conveyer_belt_point = calculate_new_point(conveyer_belt_point, 1, 0.0)
        # reversed_goal_yaw_point = calculate_reverse_point(right_conveyer_belt_point, 1.8)
        self.new_point_reindex.append(reversed_goal_point)

        for point_data, quaternion_data in self.new_point_reindex:
            point = Point(*point_data)
            quaternion = Quaternion(*quaternion_data)
            self.send_init_pose(point, quaternion)
            rospy.loginfo("\033[32m"+"途经点位发布成功"+"\033[0m")

        rospy.loginfo("\033[32m"+"下发目标点位"+"\033[0m")
        goal_point, goal_quaternion = reversed_reversed_goal_point#right_conveyer_belt_point
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
        self.send_control_task_service(0)
        self.new_point_delivery = []
        self.new_point_reindex = []

        # 到达终点->开始夹取
        if not self.brake_enable: 
            position_ = 0
            self.start_position = None
            self.start_time = None
            conveyer_belt_point = None
            rospy.loginfo_once("\033[32m"+'Position reached, switching to local_planning'+"\033[0m")
            return 'set_width_height'
        # 如果有刹车
        elif self.brake_enable:
            position_ = 0
            conveyer_belt_point = None
            self.start_position = None
            self.start_time = None
            rospy.loginfo_once('AEB')
            userdata.last_state = 'MOVING_TO_PICKUP_LOCATION'
            return 'braking'


class TemporaryParking1(smach.State, SerManager):
    def __init__(self, goal=None,throttle=None):
        smach.State.__init__(self, outcomes=['braking', 'set_width_height'], input_keys=['points'])
        SerManager.__init__(self)
        # self.points = points if points is not None else []
        self.goal = goal
        
        if goal == 'delivery_point':
            self.delivery_point_mode = 1
        elif goal=="conveyer_belt_point":
            self.delivery_point_mode = 2
        elif self.goal == "observer_point":
            self.delivery_point_mode = 3

        self.new_point_delivery = []
        self.new_point_reindex = []
        
        self.current_yaw_reverse_distance = rospy.get_param("/smach/current_yaw_reverse_distance", 0.8)
        self.goal_yaw_reverse_distance = rospy.get_param("/smach/goal_yaw_reverse_distance", -0.8)
        self.lateral_distance = rospy.get_param("/smach/lateral_distance", 1.0)
        self.path_publisher = rospy.Publisher("/smach_path", Path, queue_size=1)
        rospy.Subscriber('/steering_state', Float64, self.steering_callback)

        self.steering_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=1)
        self.throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=1)
        self.gear_pub = rospy.Publisher('/gear_cmd', UInt8, queue_size=1)
        self.brake_pub = rospy.Publisher('/brake_cmd', Float64, queue_size=1)
        self.start_position = None
        self.start_time = None
        self.new_point_back = []
        self.new_point_reindex = []

    def steering_callback(self, msg):
        self.steering_state = msg.data

    def execute(self, userdata):
        global position_, bales_num_
        global num_work,num_work_height
        global current_x, current_y, current_yaw, delivery_point, work_direction
        global num_work, num_work_max, num_work_height,work_direction
        global conveyer_belt_point,observer_point

        rospy.logwarn(f"num_work: {num_work}, num_work_height: {num_work_height}.")

        if self.delivery_point_mode == 1:
            self.goal = delivery_point
        elif self.delivery_point_mode == 2:
            self.goal = conveyer_belt_point
        if self.delivery_point_mode == 1:
            self.goal = delivery_point
        elif self.delivery_point_mode == 3:
            self.goal = observer_point

        else:
            rospy.logwarn("Invalid parameter for conveyer_belt_point.")

        rospy.loginfo_once("\033[32m"+'Executing state: TemporaryParking1 '+"\033[0m")
        rospy.loginfo("\033[32m"+"下发全局任务"+"\033[0m")
        self.task = 1
        
        self.send_plan_task_service(self.task)
        self.send_control_task_service(self.task)
        rospy.loginfo("\033[32m"+"任务下发成功"+"\033[0m")


        reversed_goal_point = calculate_reverse_point(observer_point, -5)
        reversed_reversed_goal_point = calculate_reverse_point(observer_point, -1)
        self.new_point_reindex.append(reversed_goal_point)

        for point_data, quaternion_data in self.new_point_reindex:
            point = Point(*point_data)
            quaternion = Quaternion(*quaternion_data)
            self.send_init_pose(point, quaternion)
            rospy.loginfo("\033[32m"+"途经点位发布成功"+"\033[0m")

        rospy.loginfo("\033[32m"+"下发目标点位"+"\033[0m")
        goal_point, goal_quaternion = observer_point#right_conveyer_belt_point
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
        self.send_control_task_service(0)
        self.new_point_delivery = []
        self.new_point_reindex = []

        # 到达终点->开始夹取
        if not self.brake_enable: 
            position_ = 0
            self.start_position = None
            self.start_time = None
            conveyer_belt_point = None
            rospy.loginfo_once("\033[32m"+'Position reached, switching to local_planning'+"\033[0m")
            return 'set_width_height'
        # 如果有刹车
        elif self.brake_enable:
            position_ = 0
            conveyer_belt_point = None
            self.start_position = None
            self.start_time = None
            rospy.loginfo_once('AEB')
            userdata.last_state = 'MOVING_TO_PICKUP_LOCATION'
            return 'braking'


class TemporaryParking3(smach.State, SerManager):
    def __init__(self, goal=None,throttle=None):
        smach.State.__init__(self, outcomes=['braking', 'set_width_height'], input_keys=['points'])
        SerManager.__init__(self)
        # self.points = points if points is not None else []
        self.goal = goal
        
        if goal == 'delivery_point':
            self.delivery_point_mode = 1
        elif goal=="conveyer_belt_point":
            self.delivery_point_mode = 2
        elif goal=="observer_point":
            self.delivery_point_mode = 3

        self.new_point_delivery = []
        self.new_point_reindex = []
        
        self.current_yaw_reverse_distance = rospy.get_param("/smach/current_yaw_reverse_distance", 0.8)
        self.goal_yaw_reverse_distance = rospy.get_param("/smach/goal_yaw_reverse_distance", -0.8)
        self.lateral_distance = rospy.get_param("/smach/lateral_distance", 1.0)
        self.path_publisher = rospy.Publisher("/smach_path", Path, queue_size=1)
        rospy.Subscriber('/steering_state', Float64, self.steering_callback)

        self.steering_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=1)
        self.throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=1)
        self.gear_pub = rospy.Publisher('/gear_cmd', UInt8, queue_size=1)
        self.brake_pub = rospy.Publisher('/brake_cmd', Float64, queue_size=1)
        self.start_position = None
        self.start_time = None
        self.new_point_back = []
        self.new_point_reindex = []

    def steering_callback(self, msg):
        self.steering_state = msg.data

    def execute(self, userdata):
        global position_, bales_num_
        global num_work,num_work_height
        global current_x, current_y, current_yaw, delivery_point, work_direction
        global num_work, num_work_max, num_work_height,work_direction
        global conveyer_belt_point, observer_point

        rospy.logwarn(f"num_work: {num_work}, num_work_height: {num_work_height}.")

        if self.delivery_point_mode == 1:
            self.goal = delivery_point
        elif self.delivery_point_mode == 2:
            self.goal = conveyer_belt_point
        elif self.delivery_point_mode == 3:
            self.goal = observer_point

        else:
            rospy.logwarn("Invalid parameter for conveyer_belt_point.")

        rospy.loginfo_once("\033[32m"+'Executing state: TemporaryParking2'+"\033[0m")
        rospy.loginfo("\033[32m"+"下发全局任务"+"\033[0m")
        self.task = 1
        
        self.send_plan_task_service(self.task)
        self.send_control_task_service(self.task)
        rospy.loginfo("\033[32m"+"任务下发成功"+"\033[0m")

        reversed_goal_point = calculate_reverse_point(observer_point, -5)
        reversed_reversed_goal_point = calculate_reverse_point(observer_point, 1.5)
        # reversed_goal_yaw_point = calculate_reverse_point(right_conveyer_belt_point, 1.8)
        self.new_point_reindex.append(reversed_goal_point)

        for point_data, quaternion_data in self.new_point_reindex:
            point = Point(*point_data)
            quaternion = Quaternion(*quaternion_data)
            self.send_init_pose(point, quaternion)
            rospy.loginfo("\033[32m"+"途经点位发布成功"+"\033[0m")

        rospy.loginfo("\033[32m"+"下发目标点位"+"\033[0m")
        goal_point, goal_quaternion = reversed_reversed_goal_point#right_conveyer_belt_point
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
        self.send_control_task_service(0)
        self.new_point_delivery = []
        self.new_point_reindex = []

        # 到达终点->开始夹取
        if not self.brake_enable: 
            position_ = 0
            self.start_position = None
            self.start_time = None
            conveyer_belt_point = None
            rospy.loginfo_once("\033[32m"+'Position reached, switching to local_planning'+"\033[0m")
            return 'set_width_height'
        # 如果有刹车
        elif self.brake_enable:
            position_ = 0
            conveyer_belt_point = None
            self.start_position = None
            self.start_time = None
            rospy.loginfo_once('AEB')
            userdata.last_state = 'MOVING_TO_PICKUP_LOCATION'
            return 'braking'



class TemporaryParking4(smach.State, SerManager):
    def __init__(self, goal=None,throttle=None):
        smach.State.__init__(self, outcomes=['braking', 'set_width_height'], input_keys=['points'])
        SerManager.__init__(self)
        # self.points = points if points is not None else []
        self.goal = goal
        
        if goal == 'delivery_point':
            self.delivery_point_mode = 1
        elif goal=="conveyer_belt_point":
            self.delivery_point_mode = 2
        elif goal=="observer_point":
            self.delivery_point_mode = 3

        self.new_point_delivery = []
        self.new_point_reindex = []
        
        self.current_yaw_reverse_distance = rospy.get_param("/smach/current_yaw_reverse_distance", 0.8)
        self.goal_yaw_reverse_distance = rospy.get_param("/smach/goal_yaw_reverse_distance", -0.8)
        self.lateral_distance = rospy.get_param("/smach/lateral_distance", 1.0)
        self.path_publisher = rospy.Publisher("/smach_path", Path, queue_size=1)
        rospy.Subscriber('/steering_state', Float64, self.steering_callback)

        self.steering_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=1)
        self.throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=1)
        self.gear_pub = rospy.Publisher('/gear_cmd', UInt8, queue_size=1)
        self.brake_pub = rospy.Publisher('/brake_cmd', Float64, queue_size=1)
        self.start_position = None
        self.start_time = None
        self.new_point_back = []
        self.new_point_reindex = []

    def steering_callback(self, msg):
        self.steering_state = msg.data

    def execute(self, userdata):
        global position_, bales_num_
        global num_work,num_work_height
        global current_x, current_y, current_yaw, delivery_point, work_direction
        global num_work, num_work_max, num_work_height,work_direction
        global conveyer_belt_point, observer_point

        rospy.logwarn(f"num_work: {num_work}, num_work_height: {num_work_height}.")

        if self.delivery_point_mode == 1:
            self.goal = delivery_point
        elif self.delivery_point_mode == 2:
            self.goal = conveyer_belt_point
        elif self.delivery_point_mode == 3:
            self.goal = observer_point

        else:
            rospy.logwarn("Invalid parameter for conveyer_belt_point.")

        rospy.loginfo_once("\033[32m"+'Executing state: TemporaryParking2'+"\033[0m")
        rospy.loginfo("\033[32m"+"下发全局任务"+"\033[0m")
        self.task = 1
        
        self.send_plan_task_service(self.task)
        self.send_control_task_service(self.task)
        rospy.loginfo("\033[32m"+"任务下发成功"+"\033[0m")

        reversed_goal_point = calculate_reverse_point(observer_point, -5)
        reversed_reversed_goal_point = calculate_reverse_point(observer_point, 1.5+0.9)
        # reversed_goal_yaw_point = calculate_reverse_point(right_conveyer_belt_point, 1.8)
        self.new_point_reindex.append(reversed_goal_point)

        for point_data, quaternion_data in self.new_point_reindex:
            point = Point(*point_data)
            quaternion = Quaternion(*quaternion_data)
            self.send_init_pose(point, quaternion)
            rospy.loginfo("\033[32m"+"途经点位发布成功"+"\033[0m")

        rospy.loginfo("\033[32m"+"下发目标点位"+"\033[0m")
        goal_point, goal_quaternion = reversed_reversed_goal_point#right_conveyer_belt_point
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
        self.send_control_task_service(0)
        self.new_point_delivery = []
        self.new_point_reindex = []

        # 到达终点->开始夹取
        if not self.brake_enable: 
            position_ = 0
            self.start_position = None
            self.start_time = None
            conveyer_belt_point = None
            rospy.loginfo_once("\033[32m"+'Position reached, switching to local_planning'+"\033[0m")
            return 'set_width_height'
        # 如果有刹车
        elif self.brake_enable:
            position_ = 0
            conveyer_belt_point = None
            self.start_position = None
            self.start_time = None
            rospy.loginfo_once('AEB')
            userdata.last_state = 'MOVING_TO_PICKUP_LOCATION'
            return 'braking'


class GlobalPickupTask_3(smach.State, SerManager):
    def __init__(self, goal=None, Throttle=None, tar_height=None, updown_mode=None,tar_width=None):
        smach.State.__init__(self, outcomes=['braking', 'set_width_height'], input_keys=['points'])
        SerManager.__init__(self)

        self.start_position = None
        self.path_publisher = rospy.Publisher('/smach_path', Path, queue_size=1)
        self.Throttle = Throttle
        self.steering_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=1)
        self.throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=1)
        self.gear_pub = rospy.Publisher('/gear_cmd', UInt8, queue_size=1)
        self.brake_pub = rospy.Publisher('/brake_cmd', Float64, queue_size=1)
        

        self.goal = goal
        if goal == 'delivery_point':
            self.delivery_point_mode = 1
        elif goal=="conveyer_belt_point":
            self.delivery_point_mode = 2

        self.new_point_delivery = []
        self.new_point_reindex = []

        self.tar_height = tar_height
        self.tar_width = tar_width
        # self.timer = timer
        self.set_tar_height_pub = rospy.Publisher('/updown_cmd', Float64, queue_size=1)
        self.set_tar_height_pub_ = rospy.Publisher('/updown_mode', Int8, queue_size=1)
        
        self.updown_mode = updown_mode
        rospy.Subscriber('/updown_state', Float64, self.updown_state_callback)
        rospy.Subscriber('/clamp_state', Float64, self.clamp_state_callback)

        self.set_tar_width_pub = rospy.Publisher('/clamp_cmd', Float64, queue_size=1)
        self.set_tar_fy_pub = rospy.Publisher('/fy_cmd', Float64, queue_size=1)
        self.set_tar_lateral_pub = rospy.Publisher('/lateral_cmd', Float64, queue_size=1)
        rospy.Subscriber('/ipc_state', Int8, self.ipc_state_callback)
        self.path_publisher = rospy.Publisher('/smach_path', Path, queue_size=1)

        self.cur_height = 0
        self.ipc_state = 0
        self.start_time = None
        self.previous_changed_height = None
        self.last_check_time = None
        self.start_time_2 = None
        self.last_check_time_2 = None
        self.cur_width = 0
        self.previous_changed_width = None

    def ipc_state_callback(self, msg):
        self.ipc_state = msg.data

    def clamp_state_callback(self, msg):
        self.cur_width = msg.data

    def updown_state_callback(self, msg):
        self.cur_height = msg.data
        self.changed_height = abs(self.tar_height - self.cur_height)


    def execute(self, userdata):
        global num_work
        global position_, bales_num_
        global num_work
        global current_x, current_y, current_yaw, delivery_point
        global num_work, num_work_max, num_work_height,work_direction
        global conveyer_belt_point

        if self.delivery_point_mode == 1:
            self.goal = delivery_point
        elif self.delivery_point_mode == 2:
            self.goal = conveyer_belt_point
        else:
            rospy.logwarn("Invalid parameter for conveyer_belt_point.")

        rospy.loginfo_once("\033[32m"+'Executing state: GlobalPickupTask_3'+"\033[0m")
        rospy.loginfo("\033[32m"+"下发全局任务"+"\033[0m")
        self.task = 1
        
        self.send_plan_task_service(self.task)
        self.send_control_task_service(self.task)
        rospy.loginfo("\033[32m"+"任务下发成功"+"\033[0m")

        right_conveyer_belt_point = calculate_new_point(conveyer_belt_point, 1, 0.0)
        reversed_reversed_goal_yaw_point = calculate_reverse_point(right_conveyer_belt_point, 0.1)
        reversed_goal_yaw_point = calculate_reverse_point(right_conveyer_belt_point, 1.8)
        self.new_point_reindex.append(reversed_goal_yaw_point)

        for point_data, quaternion_data in self.new_point_reindex:
            point = Point(*point_data)
            quaternion = Quaternion(*quaternion_data)
            self.send_init_pose(point, quaternion)
            rospy.loginfo("\033[32m"+"途经点位发布成功"+"\033[0m")

        rospy.loginfo("\033[32m"+"下发目标点位"+"\033[0m")
        goal_point, goal_quaternion = reversed_reversed_goal_yaw_point
        goal_pose = Point(goal_point[0], goal_point[1], goal_point[2])
        goal_orientation = Quaternion(goal_quaternion[0], goal_quaternion[1], goal_quaternion[2], goal_quaternion[3])
        self.send_target_pose(goal_pose, goal_orientation)
        rospy.loginfo("\033[32m"+"目标点位发布成功"+"\033[0m")


        def first_loop():
            count = 0
            while True:
                if (self.tar_width==0):
                    break
                # 先循环发布210才能进无人使能
                self.set_tar_width_pub.publish(self.tar_width)
                if (self.ipc_state == 1):
                    if self.start_time is None:
                        self.start_time = rospy.Time.now()
                    # 先循环发布一段时间
                    self.set_tar_width_pub.publish(self.tar_width)
                    # 0.5s 过后，每 0.1s 检查一次宽度变化
                    if (rospy.Time.now() - self.start_time).to_sec() > 1.0:
                        current_time = rospy.Time.now()
                        if self.last_check_time is None or (current_time - self.last_check_time).to_sec() >= 1:
                            current_changed_width = abs(self.tar_width - self.cur_width)
                            if self.previous_changed_width is not None:
                                difference = abs(current_changed_width - self.previous_changed_width)
                                # print("difference: ", difference)
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

        def second_loop():
            count2 = 0
            while True:
                if (self.tar_height==0):
                    break
                # 先循环发布210才能进无人使能
                self.set_tar_height_pub.publish(self.tar_height)
                self.set_tar_height_pub_.publish(self.updown_mode)
                
                if (self.ipc_state == 1):
                    if self.start_time_2 is None:
                        self.start_time_2 = rospy.Time.now()
                    # 先循环发布一段时间
                    self.set_tar_height_pub.publish(self.tar_height)
                    self.set_tar_height_pub_.publish(self.updown_mode)
                    if (rospy.Time.now() - self.start_time_2).to_sec() > 1.0:
                        current_time = rospy.Time.now()
                        if self.last_check_time_2 is None or (current_time - self.last_check_time_2).to_sec() >= 1.0:
                            current_changed_height = abs(self.tar_height - self.cur_height)
                            if self.previous_changed_height is not None:
                                difference = abs(current_changed_height - self.previous_changed_height)
                                if difference < 10:
                                    self.start_time_2 = None
                                    self.last_check_time_2 = None
                                    self.previous_changed_height = None
                                    break
                                self.previous_changed_height = current_changed_height
                            else:
                                self.previous_changed_height = current_changed_height
                            self.last_check_time_2 = current_time
                    count2 += 1
                    if count2 % 100 == 0:
                        rospy.loginfo(f" \033[32m cur_height:{(self.cur_height):.3f} \033[0m")
                        rospy.loginfo(f" \033[32m tar_height:{(self.tar_height):.3f} \033[0m")
                else:
                    rospy.loginfo_once("\033[31m"+"Wait For auto_drive"+"\033[0m")
                rospy.Rate(100).sleep()

        def third_loop():
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

        thread1 = threading.Thread(target=first_loop)
        thread2 = threading.Thread(target=second_loop)
        thread3 = threading.Thread(target=third_loop)
        thread1.start()
        thread2.start()
        thread3.start()
        thread1.join()
        thread2.join()
        thread3.join()

        self.send_plan_task_service(0)
        self.send_control_task_service(0)
        self.new_point_delivery = []
        self.new_point_reindex = []
        # 到达终点->开始夹取
        if not self.brake_enable:
            position_ = 0 
            conveyer_belt_point = None
            self.start_position = None
            rospy.loginfo_once("\033[32m"+'Position reached, switching to local_planning'+"\033[0m")
            return 'set_width_height'
        # 如果有刹车
        elif self.brake_enable:
            position_ = 0
            conveyer_belt_point = None
            self.start_position = None
            rospy.loginfo_once('AEB')
            userdata.last_state = 'MOVING_TO_PICKUP_LOCATION'
            return 'braking'

class GlobalPickupTask_2(smach.State, SerManager):
    def __init__(self, goal=None,throttle=None):

        smach.State.__init__(self, outcomes=['braking', 'set_width_height'], input_keys=['points'])
        SerManager.__init__(self)
        # self.points = points if points is not None else []
        self.goal = goal
        
        if goal == 'delivery_point':
            self.delivery_point_mode = 1
        elif goal=="conveyer_belt_point":
            self.delivery_point_mode = 2

        self.new_point_delivery = []
        self.new_point_reindex = []
        
        self.current_yaw_reverse_distance = rospy.get_param("/smach/current_yaw_reverse_distance", 0.8)
        self.goal_yaw_reverse_distance = rospy.get_param("/smach/goal_yaw_reverse_distance", -0.8)
        self.lateral_distance = rospy.get_param("/smach/lateral_distance", 1.0)
        self.path_publisher = rospy.Publisher("/smach_path", Path, queue_size=1)
        rospy.Subscriber('/steering_state', Float64, self.steering_callback)


        self.steering_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=1)
        self.throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=1)
        self.gear_pub = rospy.Publisher('/gear_cmd', UInt8, queue_size=1)
        self.brake_pub = rospy.Publisher('/brake_cmd', Float64, queue_size=1)
        self.start_position = None
        self.start_time = None
        
    def steering_callback(self, msg):
        self.steering_state = msg.data

    def execute(self, userdata):
        global position_, bales_num_
        global num_work
        global current_x, current_y, current_yaw, delivery_point, work_direction
        global num_work, num_work_max, num_work_height,work_direction
        global conveyer_belt_point,observer_point,conveyer_belt_point_camera

        if self.delivery_point_mode == 1:
            self.goal = delivery_point
        elif self.delivery_point_mode == 2:
            self.goal = conveyer_belt_point
        else:
            rospy.logwarn("Invalid parameter for conveyer_belt_point.")

        rospy.loginfo_once("\033[32m"+'Executing state: GlobalPickupTask_2'+"\033[0m")
        rospy.loginfo("\033[32m"+"下发全局任务"+"\033[0m")
        self.task = 1
        
        self.send_plan_task_service(self.task)
        self.send_control_task_service(self.task)
        rospy.loginfo("\033[32m"+"任务下发成功"+"\033[0m")
        # print("conveyer_belt_point", conveyer_belt_point)
        # print("self.goal", self.goal)
        # reversed_goal_yaw_point = calculate_reverse_point(observer_point, -1.8)
        reversed_goal_yaw_point = calculate_reverse_point(conveyer_belt_point_camera, 2.5)
        # print("reversed_goal_yaw_point", reversed_goal_yaw_point)
        # self.new_point_reindex.append(reversed_goal_yaw_point)

        # for point_data, quaternion_data in self.new_point_reindex:
        #     point = Point(*point_data)
        #     quaternion = Quaternion(*quaternion_data)
        #     self.send_init_pose(point, quaternion)
        #     rospy.loginfo("\033[32m"+"途经点位发布成功"+"\033[0m")

        rospy.loginfo("\033[32m"+"下发目标点位"+"\033[0m")
        goal_point, goal_quaternion = reversed_goal_yaw_point # observer_point
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
        self.send_control_task_service(0)
        self.new_point_delivery = []
        self.new_point_reindex = []
        while True:
            self.brake_pub.publish(0.0)
            self.steering_pub.publish(0.0)
            throttle_cmd = Float64(0.0)
            self.throttle_pub.publish(0)
            gear_cmd = UInt8(2)
            self.gear_pub.publish(gear_cmd)
            if self.start_time is None:
                self.start_time = rospy.Time.now()
            if (rospy.Time.now() - self.start_time).to_sec() > 0.5:
                break
        
        # 到达终点->开始夹取
        if not self.brake_enable: 
            position_ = 0
            self.start_position = None
            self.start_time = None
            rospy.loginfo_once("\033[32m"+'Position reached, switching to local_planning'+"\033[0m")
            return 'set_width_height'
        # 如果有刹车
        elif self.brake_enable:
            position_ = 0
            self.start_position = None
            self.start_time = None
            rospy.loginfo_once('AEB')
            userdata.last_state = 'MOVING_TO_PICKUP_LOCATION'
            return 'braking'
    

class GlobalBackTask1(smach.State, SerManager):
    def __init__(self, gear=None,throttle=None,tar_height=None):
        smach.State.__init__(self, outcomes=['braking', 'set_width_height'], input_keys=['points'])
        SerManager.__init__(self)

        self.path_publisher = rospy.Publisher('/smach_path', Path, queue_size=1)

        self.set_tar_height_pub = rospy.Publisher('/updown_cmd', Float64, queue_size=1)
        self.set_tar_height_pub_ = rospy.Publisher('/updown_mode', Int8, queue_size=1)
        self.height_between_two_cargos = rospy.get_param('/real/height_between_two_cargos', 234)


        self.new_point_back = []
        self.new_point_reindex = []

        self.start_position = None

        self.current_yaw_reverse_distance = rospy.get_param("/smach/current_yaw_reverse_distance", 0.8)
        self.goal_yaw_reverse_distance = rospy.get_param("/smach/goal_yaw_reverse_distance", -0.8)
        self.lateral_distance = rospy.get_param("/smach/lateral_distance", 1.0)
        rospy.Subscriber('/steering_state', Float64, self.steering_callback)

        self.steering_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=1)
        self.throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=1)
        self.gear_pub = rospy.Publisher('/gear_cmd', UInt8, queue_size=1)
        self.brake_pub = rospy.Publisher('/brake_cmd', Float64, queue_size=1)

        self.tar_height = tar_height

        rospy.Subscriber('/updown_state', Float64, self.updown_state_callback)
        rospy.Subscriber('/clamp_state', Float64, self.clamp_state_callback)

        self.set_tar_width_pub = rospy.Publisher('/clamp_cmd', Float64, queue_size=1)
        self.set_tar_fy_pub = rospy.Publisher('/fy_cmd', Float64, queue_size=1)
        self.set_tar_lateral_pub = rospy.Publisher('/lateral_cmd', Float64, queue_size=1)
        rospy.Subscriber('/ipc_state', Int8, self.ipc_state_callback)

        self.cur_height = 0
        self.ipc_state = 0
        self.start_time = None
        self.previous_changed_height = None
        self.last_check_time = None

        self.start_time_2 = None
        self.last_check_time_2 = None

        self.cur_width = 0
        self.previous_changed_width = None

    def ipc_state_callback(self, msg):
        self.ipc_state = msg.data

    def clamp_state_callback(self, msg):
        self.cur_width = msg.data

    def updown_state_callback(self, msg):
        self.cur_height = msg.data
        self.changed_height = abs(self.tar_height - self.cur_height)


    def steering_callback(self, msg):
        self.steering_state = msg.data


    def execute(self, userdata):
        global num_work
        global position_, bales_num_, delivery_point
        global position_, bales_num_
        global num_work
        global current_x, current_y, current_yaw, delivery_point, work_direction
        global num_work, num_work_max, num_work_height, work_direction
        global conveyer_belt_point

        # 推算途经的点
        if (work_direction==1):# 左边放
            reversed_point = calculate_reverse_point(delivery_point, 1.5)
            left_reversed_point  = calculate_new_point(reversed_point, 2, 8.0)# 左
            right_rotation_left_reversed_point = calculate_new_point_rotation(left_reversed_point, 1)#顺时针
        elif (work_direction==2):
            # 这里只使用第一个的放货点来计算位置
            reversed_point = calculate_reverse_point(delivery_point, 1.5)
            left_reversed_point  = calculate_new_point(reversed_point, 1, 8.0)# 右
            right_rotation_left_reversed_point = calculate_new_point_rotation(left_reversed_point, 2)
        else:
            rospy.logwarn("no work direction input, please check!!!")

        x_, y_, _ = right_rotation_left_reversed_point[0]
        # 计算距离：
        distance = math.sqrt((x_ - current_x) ** 2 + (y_ - current_y) ** 2)
        # 现在重新变成前进
        self.target_distance = -distance
        # self.target_distance = distance

        if self.start_position is None:
            self.start_position = Point(current_x, current_y, 0.0)

        if self.target_distance < 0:
            GEAR = 1
            rospy.loginfo("\033[33m"+"倒车"+"\033[0m")
        else:
            GEAR = 3
            rospy.loginfo("\033[33m"+"前进"+"\033[0m")

        count = 0  # 初始化计数器
        path_msg = Path()
        path_msg.header.frame_id = 'map'

        # 根据目标距离和步长计算路径点数量
        num_points = int(abs(self.target_distance) / 0.1)
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
        
        def first_loop():
            count = 0
            while True:
                distance_ = math.sqrt((x_ - current_x) ** 2 + (y_ - current_y) ** 2)
                count += 1  # 每次循环递增计数器
                
                if count % 50 == 0:  # 每隔100次打印一次
                    rospy.loginfo("\033[32m" + f"Target:{self.target_distance:.3f}" "\033[0m")
                    rospy.loginfo("\033[33m" + f"Current:{self.distance:.3f}" "\033[0m")

                current_position = Point(current_x, current_y, 0.0)
                dx = current_position.x - self.start_position.x
                dy = current_position.y - self.start_position.y
                self.distance = ((dx ** 2) + (dy ** 2)) ** 0.5  

                if (self.steering_state is None):
                    rospy.loginfo("\033[31m"+"wait steering"+"\033[0m")

                if self.distance >= abs(self.target_distance):
                    # 到位了之后再等待 (不能直接停住，会导致多走)
                    self.brake_pub.publish(4.0)
                    self.steering_pub.publish(0.0)
                    throttle_cmd = Float64(0.0)
                    self.throttle_pub.publish(0)
                    gear_cmd = UInt8(2)
                    self.gear_pub.publish(gear_cmd)
                    if self.start_time is None:
                        self.start_time = rospy.Time.now()
                    if (rospy.Time.now() - self.start_time).to_sec() > 0.5:
                        rospy.loginfo("\033[33m" + f"last Current:{self.distance:.3f}" "\033[0m")
                        break
                else:
                    # 运动之前先检查转角是否摆正
                    if (abs(self.steering_state)>0.1):
                        steering_cmd = Float64(0.0)
                        self.steering_pub.publish(steering_cmd)
                        throttle_cmd = Float64(0.0)
                        self.throttle_pub.publish(throttle_cmd)
                        gear_cmd = UInt8(GEAR)
                        self.gear_pub.publish(gear_cmd)
                        brake_cmd = Float64(0.0)
                        self.brake_pub.publish(brake_cmd)
                        # 发布路径消息
                        self.path_publisher.publish(path_msg)
                    if (distance_<=3):
                        steering_cmd = Float64(0.0)
                        self.steering_pub.publish(steering_cmd)
                        throttle_cmd = Float64(0.4)
                        self.throttle_pub.publish(throttle_cmd)
                        gear_cmd = UInt8(GEAR)
                        self.gear_pub.publish(gear_cmd)
                        brake_cmd = Float64(0.0)
                        self.brake_pub.publish(brake_cmd)
                        # 发布路径消息
                        self.path_publisher.publish(path_msg)
                    else:
                        steering_cmd = Float64(0.0)
                        self.steering_pub.publish(steering_cmd)
                        throttle_cmd = Float64(1.0)
                        self.throttle_pub.publish(throttle_cmd)
                        gear_cmd = UInt8(GEAR)
                        self.gear_pub.publish(gear_cmd)
                        brake_cmd = Float64(0.0)
                        self.brake_pub.publish(brake_cmd)
                        # 发布路径消息
                        self.path_publisher.publish(path_msg)
                rospy.Rate(100).sleep()

        def second_loop():
            count2 = 0
            while True:
                if (self.tar_height==0):
                    break

                # 先循环发布210才能进无人使能
                self.set_tar_height_pub.publish(self.height_between_two_cargos)
                self.set_tar_height_pub_.publish(2)

                if (self.ipc_state == 1):
                    if self.start_time_2 is None:
                        self.start_time_2 = rospy.Time.now()
                    # 先循环发布一段时间
                    self.set_tar_height_pub.publish(self.height_between_two_cargos)
                    self.set_tar_height_pub_.publish(2)

                    if (rospy.Time.now() - self.start_time_2).to_sec() > 1.0:
                        current_time = rospy.Time.now()
                        if self.last_check_time_2 is None or (current_time - self.last_check_time_2).to_sec() >= 1.0:
                            current_changed_height = abs(self.tar_height - self.cur_height)
                            if self.previous_changed_height is not None:
                                difference = abs(current_changed_height - self.previous_changed_height)
                                if difference < 10:
                                    self.start_time_2 = None
                                    self.last_check_time_2 = None
                                    self.previous_changed_height = None
                                    break
                                self.previous_changed_height = current_changed_height
                            else:
                                self.previous_changed_height = current_changed_height
                            self.last_check_time_2 = current_time

                    count2 += 1
                    if count2 % 100 == 0:
                        rospy.loginfo(f" \033[32m cur_height:{(self.cur_height):.3f} \033[0m")
                        rospy.loginfo(f" \033[32m tar_height:{(self.tar_height):.3f} \033[0m")
                else:
                    rospy.loginfo_once("\033[31m"+"Wait For auto_drive"+"\033[0m")
                rospy.Rate(100).sleep()

        thread1 = threading.Thread(target=first_loop)
        thread2 = threading.Thread(target=second_loop)
        thread1.start()
        thread2.start()
        thread1.join()
        thread2.join()


        # 到达终点->开始夹取
        if not self.brake_enable: 
            self.start_position = None
            self.start_time = None
            rospy.loginfo_once("\033[32m"+'Position reached, switching to local_planning'+"\033[0m")
            return 'set_width_height'
        # 如果有刹车
        elif self.brake_enable:
            self.start_position = None
            self.start_time = None
            rospy.loginfo_once('AEB')
            userdata.last_state = 'MOVING_TO_PICKUP_LOCATION'
            return 'braking'

class GlobalBackTask2(smach.State, SerManager):
    def __init__(self, tar_height=None,tar_width=None):
        smach.State.__init__(self, outcomes=['braking', 'set_width_height'], input_keys=['points'])
        SerManager.__init__(self)


        self.new_point_back = []
        self.new_point_reindex = []

        self.tar_height = tar_height
        self.tar_width = tar_width

        # self.timer = timer
        self.set_tar_height_pub = rospy.Publisher('/updown_cmd', Float64, queue_size=1)
        self.set_tar_height_pub_ = rospy.Publisher('/updown_mode', Int8, queue_size=1)
        
        self.updown_mode = 2
        rospy.Subscriber('/updown_state', Float64, self.updown_state_callback)
        rospy.Subscriber('/clamp_state', Float64, self.clamp_state_callback)

        self.set_tar_width_pub = rospy.Publisher('/clamp_cmd', Float64, queue_size=1)
        self.set_tar_fy_pub = rospy.Publisher('/fy_cmd', Float64, queue_size=1)
        self.set_tar_lateral_pub = rospy.Publisher('/lateral_cmd', Float64, queue_size=1)
        rospy.Subscriber('/ipc_state', Int8, self.ipc_state_callback)
        self.path_publisher = rospy.Publisher('/smach_path', Path, queue_size=1)

        self.cur_height = 0
        self.ipc_state = 0
        self.start_time = None
        self.previous_changed_height = None
        self.last_check_time = None

        self.start_time_2 = None
        self.last_check_time_2 = None

        self.cur_width = 0
        self.previous_changed_width = None

    def clamp_state_callback(self, msg):
        self.cur_width = msg.data
        self.changed_width = abs(self.tar_width - self.cur_width)

    def updown_state_callback(self, msg):
        self.cur_height = msg.data
        self.changed_height = abs(self.tar_height - self.cur_height)


    def ipc_state_callback(self, msg):
        self.ipc_state = msg.data

        
    def execute(self, userdata):
        global num_work, num_work_max, num_work_height,work_direction
        global position_, bales_num_, delivery_point, conveyer_belt_point, observer_point
        rospy.loginfo_once("\033[32m"+'Executing state: 前往取货区任务开始'+"\033[0m")
        rospy.loginfo("\033[32m"+"下发全局任务"+"\033[0m")
        self.task = 1
        
        self.send_plan_task_service(self.task)
        self.send_control_task_service(self.task)
        rospy.loginfo("\033[32m"+"任务下发成功"+"\033[0m")

        reversed_goal_yaw_point = calculate_reverse_point(observer_point, 1.5)
        self.new_point_reindex.append(reversed_goal_yaw_point)

        for point_data, quaternion_data in self.new_point_reindex:
            point = Point(*point_data)
            quaternion = Quaternion(*quaternion_data)
            self.send_init_pose(point, quaternion)
            rospy.loginfo("\033[32m"+"途经点位发布成功"+"\033[0m")

        rospy.loginfo("\033[32m"+"下发目标点位"+"\033[0m")
        goal_point, goal_quaternion = observer_point
        goal_pose = Point(goal_point[0], goal_point[1], goal_point[2])
        goal_orientation = Quaternion(goal_quaternion[0], goal_quaternion[1], goal_quaternion[2], goal_quaternion[3])
        self.send_target_pose(goal_pose, goal_orientation)
        rospy.loginfo("\033[32m"+"目标点位发布成功"+"\033[0m")


        def first_loop():
            count = 0
            count2 = 0
            while True:
                if (self.tar_width==0):
                    break
                # 先循环发布210才能进无人使能
                self.set_tar_width_pub.publish(self.tar_width)
                self.set_tar_height_pub_.publish(self.updown_mode)
                if (self.ipc_state == 1):
                    if self.start_time is None:
                        self.start_time = rospy.Time.now()
                    # 先循环发布一段时间
                    self.set_tar_width_pub.publish(self.tar_width)
                    self.set_tar_height_pub_.publish(self.updown_mode)
                    # 0.5s 过后，每 0.1s 检查一次宽度变化
                    if (rospy.Time.now() - self.start_time).to_sec() > 1.0:
                        current_time = rospy.Time.now()
                        if self.last_check_time is None or (current_time - self.last_check_time).to_sec() >= 1:
                            current_changed_width = abs(self.tar_width - self.cur_width)
                            if self.previous_changed_width is not None:
                                difference = abs(current_changed_width - self.previous_changed_width)
                                # print("difference: ", difference)
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

        def second_loop():
            count = 0
            count2 = 0
            while True:
                if (self.tar_height==0):
                    break
                # 空的路径
                path_msg = Path()
                path_msg.header.frame_id = 'map'
                self.path_publisher.publish(path_msg)

                # 先循环发布210才能进无人使能
                self.set_tar_height_pub.publish(self.tar_height)

                if (self.ipc_state == 1):
                    if self.start_time_2 is None:
                        self.start_time_2 = rospy.Time.now()
                    # 先循环发布一段时间
                    self.set_tar_height_pub.publish(self.tar_height)
                    # self.set_tar_height_pub_.publish(self.updown_mode)

                    if (rospy.Time.now() - self.start_time_2).to_sec() > 1.0:
                        current_time = rospy.Time.now()
                        if self.last_check_time_2 is None or (current_time - self.last_check_time_2).to_sec() >= 1.0:
                            current_changed_height = abs(self.tar_height - self.cur_height)
                            if self.previous_changed_height is not None:
                                difference = abs(current_changed_height - self.previous_changed_height)
                                if difference < 10:
                                    self.start_time_2 = None
                                    self.last_check_time_2 = None
                                    self.previous_changed_height = None
                                    break
                                self.previous_changed_height = current_changed_height
                            else:
                                self.previous_changed_height = current_changed_height
                            self.last_check_time_2 = current_time

                    count2 += 1
                    if count2 % 100 == 0:
                        rospy.loginfo(f" \033[32m cur_height:{(self.cur_height):.3f} \033[0m")
                        rospy.loginfo(f" \033[32m tar_height:{(self.tar_height):.3f} \033[0m")
                else:
                    rospy.loginfo_once("\033[31m"+"Wait For auto_drive"+"\033[0m")
                rospy.Rate(100).sleep()

        def third_loop():
            global position_
            count = 0
            count2 = 0
            while position_ == 0 and not self.brake_enable:
                # 空的路径
                path_msg = Path()
                path_msg.header.frame_id = 'map'
                self.path_publisher.publish(path_msg)
                if(position_==1):
                    break
                count += 1
                if count % 100 == 0:
                    rospy.loginfo("\033[32m Go to global endpoint \033[0m")
                rospy.Rate(100).sleep()

        thread1 = threading.Thread(target=first_loop)
        thread2 = threading.Thread(target=second_loop)
        thread3 = threading.Thread(target=third_loop)
        thread1.start()
        thread2.start()
        thread3.start()
        thread1.join()
        thread2.join()
        thread3.join()

        # 到达终点->开始夹取
        if position_ == 1 and not self.brake_enable: 
            rospy.loginfo_once("\033[32m"+'Position reached, switching to local_planning'+"\033[0m")
            self.task = 0
            self.send_plan_task_service(self.task)
            self.send_control_task_service(self.task)
            # self.send_camera_task_service(self.task)
            position_ = 0
            bales_num_ = 0
            self.new_point_back = []
            self.new_point_reindex = []
            
            ###################################
            # 完成一次任务
            # num_work += 1
            # if (num_work >= num_work_max-1):
            #     num_work = 0
            #     num_work_height += 1
            #     num_work_max -= 4
            return 'set_width_height'
        # 如果有刹车
        elif self.brake_enable:
            rospy.loginfo_once('AEB')
            userdata.last_state = 'MOVING_TO_PICKUP_LOCATION'
            return 'braking'


# 通过输入全局的点来进行全局任务管理
# 注：现在的点不再需要设置起点，只需要途径点+终点，起点为自身的定位
class GlobalTask(smach.State, SerManager):
    def __init__(self, points=None, goal=None):
        smach.State.__init__(self, outcomes=['braking', 'set_width_height'], input_keys=['points'])
        SerManager.__init__(self)

        self.points = points if points is not None else []  # 保存点数据，如果没有提供则为空列表
        self.goal = goal  # 保存目标点数据

    def execute(self, userdata):
        global num_work
        global position_, bales_num_
        rospy.loginfo_once("\033[32m"+'Executing state: 前往取货区任务开始'+"\033[0m")
        rospy.loginfo("\033[32m"+"下发全局任务"+"\033[0m")
        self.task = 1
        self.send_plan_task_service(self.task)
        self.send_control_task_service(self.task)
        # self.send_camera_task_service(2)
        rospy.loginfo("\033[32m"+"任务下发成功"+"\033[0m")
        
        # 如果提供了途经点，则下发全局规划点
        if self.points:
            rospy.loginfo("\033[32m"+"开始全局任务，下发全局规划点"+"\033[0m")
            for point_data, quaternion_data in self.points:
                point = Point(point_data[0], point_data[1], point_data[2])
                quaternion = Quaternion(quaternion_data[0], quaternion_data[1], quaternion_data[2], quaternion_data[3])
                self.send_init_pose(point, quaternion)
                rospy.loginfo("\033[32m"+"途经点位发布成功"+"\033[0m")
        
        # 无论是否提供途经点，都下发目标点
        rospy.loginfo("\033[32m"+"下发目标点位"+"\033[0m")
        goal_point, goal_quaternion = self.goal
        goal_pose = Point(goal_point[0], goal_point[1], goal_point[2])
        goal_orientation = Quaternion(goal_quaternion[0], goal_quaternion[1], goal_quaternion[2], goal_quaternion[3])
        self.send_target_pose(goal_pose, goal_orientation)
        rospy.loginfo("\033[32m"+"目标点位发布成功"+"\033[0m")

        count = 0
        while position_ != 1 and not self.brake_enable:
            count += 1
            if count % 100 == 0:
                rospy.loginfo_once("\033[32m Go to global endpoint \033[0m")
            rospy.Rate(100).sleep()
        
        if position_ == 1 and not self.brake_enable: 
            rospy.loginfo_once("\033[32m"+'Position reached, switching to local_planning'+"\033[0m")
            self.task = 0
            self.send_plan_task_service(self.task)
            self.send_control_task_service(self.task)
            # self.send_camera_task_service(self.task)
            position_ = 0
            bales_num_ = 0
            return 'set_width_height'

        elif self.brake_enable:
            rospy.loginfo_once('AEB')
            userdata.last_state = 'MOVING_TO_PICKUP_LOCATION'
            return 'braking'


class CAMERA_TASK(smach.State, SerManager):
    def __init__(self , goal=None):
        smach.State.__init__(self, outcomes=['finish'], input_keys=['points'])
        SerManager.__init__(self)


    def execute(self, userdata):
        global num_work
        global position_, bales_num_
        global current_x, current_y, current_yaw
        global cotton_x, cotton_y, cotton_z, cotton_yaw, cotton_width, cotton_height
        global cotton_cross_product
        global observer_point,first_time_observer,conveyer_belt_point

        time.sleep(1)
        self.task = 2
        #============相机使能=============
        self.send_camera_task_service(self.task)
        # while True:
        #     rospy.loginfo("\033[32m"+f"侧面识别棉包数目：{bales_num_}"+"\033[0m")
        #     if(bales_num_>=2):
        #         rospy.loginfo("\033[32m"+f"侧面识别棉包数目：{bales_num_}"+"\033[0m")
        #         break
        #     rospy.loginfo_once("\033[31m"+"等待识别到2个棉包"+"\033[0m")
        #     rospy.Rate(1).sleep()
        # rospy.loginfo_once("\033[31m"+"识别到两个棉包"+"\033[0m")

        
        #============相机位姿=============
        while conveyer_belt_point is None:
            # print("cotton_x:", cotton_x)
            if(conveyer_belt_point is not None):
                break
            print("111", conveyer_belt_point)
            rospy.loginfo("\033[32m"+"等待识别最近的棉包"+"\033[0m")
            rospy.Rate(1).sleep()
        rospy.loginfo_once("\033[32m"+"识别到棉包位姿，关闭相机"+"\033[0m")
        print("222", conveyer_belt_point)
        # cotton_vector = np.array([cotton_x, cotton_y, cotton_yaw])
        # current_vector = np.array([current_x, current_y, current_yaw])
        # if(cotton_yaw<current_yaw)
        goal_yaw = cotton_yaw
        
        # 不再需要，因为现在是垂直对着棉包
        direction, self.need_diff = calculate_optimal_rotation(current_yaw*180/math.pi, goal_yaw*180/math.pi)

        # 1-顺时针 2-逆时针
        if(direction==1):
            # 在右边
            cotton_cross_product = -1
        if(direction==2):
            # 在左边
            cotton_cross_product = 1
        print("cotton_cross_product", cotton_cross_product)

        self.task = 0
        # 关闭使能
        self.send_camera_task_service(0)
        return 'finish'

# 放货需要单独做一个，因为放货涉及到位置的改变
class GlobalPickupTask1(smach.State, SerManager):
    def __init__(self, goal=None):
        smach.State.__init__(self, outcomes=['braking', 'set_width_height'], input_keys=['points'])
        SerManager.__init__(self)
        # self.points = points if points is not None else []
        self.goal = goal
        
        if goal == 'delivery_point':
            self.delivery_point_mode = 1
        elif goal=="conveyer_belt_point":
            self.delivery_point_mode = 2

        self.new_point_delivery = []
        self.new_point_reindex = []
        
        self.current_yaw_reverse_distance = rospy.get_param("/smach/current_yaw_reverse_distance", 0.8)
        self.goal_yaw_reverse_distance = rospy.get_param("/smach/goal_yaw_reverse_distance", -0.8)
        self.lateral_distance = rospy.get_param("/smach/lateral_distance", 1.0)
        self.path_publisher = rospy.Publisher("/smach_path", Path, queue_size=1)
        rospy.Subscriber('/steering_state', Float64, self.steering_callback)


        self.steering_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=1)
        self.throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=1)
        self.gear_pub = rospy.Publisher('/gear_cmd', UInt8, queue_size=1)
        self.brake_pub = rospy.Publisher('/brake_cmd', Float64, queue_size=1)
        self.start_position = None
        self.start_time = None
        
    def steering_callback(self, msg):
        self.steering_state = msg.data

    def execute(self, userdata):
        global position_, bales_num_
        global num_work
        global current_x, current_y, current_yaw, delivery_point, work_direction
        global num_work, num_work_max, num_work_height,work_direction
        global conveyer_belt_point

        if self.delivery_point_mode == 1:
            self.goal = delivery_point
        elif self.delivery_point_mode == 2:
            self.goal = conveyer_belt_point
        else:
            rospy.logwarn("Invalid parameter for conveyer_belt_point.")
            
        time.sleep(1)
        goal_point1 = self.goal[0]
        goal_point2 = self.goal[1]
        goal_quaternion = goal_point2
        _, _, goal_yaw = tf.euler_from_quaternion([goal_quaternion[0], goal_quaternion[1], goal_quaternion[2], goal_quaternion[3]])

        goal_slope = math.tan(goal_yaw)
        current_slope = math.tan(current_yaw)

        if goal_slope == current_slope:
            rospy.logerr_once("slope error")
        else:
            x1, y1, _ = goal_point1
            x2, y2 = current_x, current_y
            intersection_x = (y2 - y1 + x1 * goal_slope - x2 * current_slope) / (goal_slope - current_slope)
            intersection_y = y1 + goal_slope * (intersection_x - x1)
        # 交点
        current_yaw_point = process_point(intersection_x, intersection_y, current_yaw)
        goal_yaw_point = process_point(intersection_x, intersection_y, goal_yaw)

        # 途经点
        reversed_current_yaw_point = calculate_reverse_point(current_yaw_point, self.current_yaw_reverse_distance)
        x_, y_, _ = reversed_current_yaw_point[0]
        _, _, yaw_ = tf.euler_from_quaternion([reversed_current_yaw_point[1][0], reversed_current_yaw_point[1][1], reversed_current_yaw_point[1][2], reversed_current_yaw_point[1][3]])
        # 计算距离：
        distance = math.sqrt((x_ - current_x) ** 2 + (y_ - current_y) ** 2)

        self.target_distance = distance


        if self.start_position is None:
            self.start_position = Point(current_x, current_y, 0.0)

        if self.target_distance < 0:
            GEAR = 1
            rospy.loginfo("\033[33m"+"倒车"+"\033[0m")
        else:
            GEAR = 3
            rospy.loginfo("\033[33m"+"前进"+"\033[0m")


        count = 0  # 初始化计数器
        path_msg = Path()
        path_msg.header.frame_id = 'map'

        # 根据目标距离和步长计算路径点数量
        num_points = int(abs(self.target_distance) / 0.1)
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
        
        while True:
            distance_ = math.sqrt((x_ - current_x) ** 2 + (y_ - current_y) ** 2)
            count += 1  # 每次循环递增计数器
            
            if count % 50 == 0:  # 每隔100次打印一次
                rospy.loginfo("\033[32m" + f"Target:{self.target_distance:.3f}" "\033[0m")
                rospy.loginfo("\033[33m" + f"Current:{self.distance:.3f}" "\033[0m")

            current_position = Point(current_x, current_y, 0.0)
            dx = current_position.x - self.start_position.x
            dy = current_position.y - self.start_position.y
            self.distance = ((dx ** 2) + (dy ** 2)) ** 0.5  

            if (self.steering_state is None):
                rospy.loginfo("\033[31m"+"wait steering"+"\033[0m")

            if self.distance >= abs(self.target_distance):
                # 到位了之后再等待 (不能直接停住，会导致多走)
                self.brake_pub.publish(4.0)
                self.steering_pub.publish(0.0)
                throttle_cmd = Float64(0.0)
                self.throttle_pub.publish(0)
                gear_cmd = UInt8(2)
                self.gear_pub.publish(gear_cmd)
                if self.start_time is None:
                    self.start_time = rospy.Time.now()
                if (rospy.Time.now() - self.start_time).to_sec() > 0.5:
                    rospy.loginfo_once("\033[33m" + f"last Current:{self.distance:.3f}" "\033[0m")
                    break
            else:
                # 运动之前先检查转角是否摆正
                if (abs(self.steering_state)>0.1):
                    self.steering_pub.publish(0.0)
                    self.throttle_pub.publish(0.0)
                    gear_cmd = UInt8(GEAR)
                    self.gear_pub.publish(gear_cmd)
                    self.brake_pub.publish(0.0)
                    # 发布路径消息
                    self.path_publisher.publish(path_msg)

                if (distance_<=3):
                    steering_cmd = Float64(0.0)
                    self.steering_pub.publish(steering_cmd)
                    self.throttle_pub.publish(0.2)
                    gear_cmd = UInt8(GEAR)
                    self.gear_pub.publish(gear_cmd)
                    brake_cmd = Float64(0.0)
                    self.brake_pub.publish(brake_cmd)
                    # 发布路径消息
                    self.path_publisher.publish(path_msg)
                else:
                    self.steering_pub.publish(0.0)
                    self.throttle_pub.publish(0.5)
                    gear_cmd = UInt8(GEAR)
                    self.gear_pub.publish(gear_cmd)
                    self.brake_pub.publish(0.0)
                    # 发布路径消息
                    self.path_publisher.publish(path_msg)

            rospy.Rate(100).sleep()

        # 到达终点->开始夹取
        if not self.brake_enable: 
            self.start_position = None
            self.start_time = None
            rospy.loginfo_once("\033[32m"+'Position reached, switching to local_planning'+"\033[0m")
            return 'set_width_height'
        # 如果有刹车
        elif self.brake_enable:
            self.start_position = None
            self.start_time = None
            rospy.loginfo_once('AEB')
            userdata.last_state = 'MOVING_TO_PICKUP_LOCATION'
            return 'braking'

# 放货需要单独做一个，因为放货涉及到位置的改变
class GlobalPickupTask_1(smach.State, SerManager):
    def __init__(self, goal=None,throttle=None):
        smach.State.__init__(self, outcomes=['braking', 'set_width_height'], input_keys=['points'])
        SerManager.__init__(self)
        # self.points = points if points is not None else []
        self.goal = goal
        
        if goal == 'delivery_point':
            self.delivery_point_mode = 1
        elif goal=="conveyer_belt_point":
            self.delivery_point_mode = 2

        self.new_point_delivery = []
        self.new_point_reindex = []
        
        self.current_yaw_reverse_distance = rospy.get_param("/smach/current_yaw_reverse_distance", 0.8)
        self.goal_yaw_reverse_distance = rospy.get_param("/smach/goal_yaw_reverse_distance", -0.8)
        self.lateral_distance = rospy.get_param("/smach/lateral_distance", 1.0)
        self.path_publisher = rospy.Publisher("/smach_path", Path, queue_size=1)
        rospy.Subscriber('/steering_state', Float64, self.steering_callback)

        self.steering_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=1)
        self.throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=1)
        self.gear_pub = rospy.Publisher('/gear_cmd', UInt8, queue_size=1)
        self.brake_pub = rospy.Publisher('/brake_cmd', Float64, queue_size=1)
        self.start_position = None
        self.start_time = None
        self.new_point_back = []
        self.new_point_reindex = []

    def steering_callback(self, msg):
        self.steering_state = msg.data

    def execute(self, userdata):
        global position_, bales_num_
        global num_work,num_work_height
        global current_x, current_y, current_yaw, delivery_point, work_direction
        global num_work, num_work_max, num_work_height,work_direction
        global conveyer_belt_point, conveyer_belt_point_camera

        rospy.logwarn(f"num_work: {num_work}, num_work_height: {num_work_height}.")

        if self.delivery_point_mode == 1:
            self.goal = delivery_point
        elif self.delivery_point_mode == 2:
            self.goal = conveyer_belt_point
        else:
            rospy.logwarn("Invalid parameter for conveyer_belt_point.")
        print("conveyer_belt_point", conveyer_belt_point)
        rospy.loginfo_once("\033[32m"+'Executing state: GlobalPickupTask_1'+"\033[0m")
        rospy.loginfo("\033[32m"+"下发全局任务"+"\033[0m")
        self.task = 1
        
        self.send_plan_task_service(self.task)
        self.send_control_task_service(self.task)
        rospy.loginfo("\033[32m"+"任务下发成功"+"\033[0m")

        conveyer_belt_point_camera = conveyer_belt_point
        right_conveyer_belt_point = calculate_new_point(conveyer_belt_point, 1, 0.0) # 目标点的左右移动（在camera中使用了）
        reversed_goal_yaw_point = calculate_reverse_point(right_conveyer_belt_point, 2.0) # 后移 要求检测距离为2.5m时。
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
        self.send_control_task_service(0)
        self.new_point_delivery = []
        self.new_point_reindex = []

        # 到达终点->开始夹取
        if not self.brake_enable: 
            position_ = 0
            self.start_position = None
            self.start_time = None
            conveyer_belt_point = None
            rospy.loginfo_once("\033[32m"+'Position reached, switching to local_planning'+"\033[0m")
            return 'set_width_height'
        # 如果有刹车
        elif self.brake_enable:
            position_ = 0
            conveyer_belt_point = None
            self.start_position = None
            self.start_time = None
            rospy.loginfo_once('AEB')
            userdata.last_state = 'MOVING_TO_PICKUP_LOCATION'
            return 'braking'


class GlobalPickupTask2(smach.State, SerManager):
    def __init__(self, goal=None, Throttle=None,tar_height=None):
        smach.State.__init__(self, outcomes=['braking', 'set_width_height'])
        SerManager.__init__(self)
        # 初始化相关变量
        self.conveyer_belt_yaw = None
        self.current_yaw = None
        self.steering_state = None
        self.tar_height = tar_height

        self.steering_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=1)
        rospy.Subscriber('/steering_state', Float64, self.steering_callback)
        self.throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=1)
        self.gear_pub = rospy.Publisher('/gear_cmd', UInt8, queue_size=1)
        self.brake_pub = rospy.Publisher('/brake_cmd', Float64, queue_size=1)
        self.path_publisher = rospy.Publisher('/smach_path', Path, queue_size=1)

        rospy.Subscriber('/updown_state', Float64, self.updown_state_callback)
        rospy.Subscriber('/clamp_state', Float64, self.clamp_state_callback)

        self.set_tar_height_pub = rospy.Publisher('/updown_cmd', Float64, queue_size=1)
        self.set_tar_height_pub_ = rospy.Publisher('/updown_mode', Int8, queue_size=1)
        rospy.Subscriber('/ipc_state', Int8, self.ipc_state_callback)

        self.adjust_steering_angle = rospy.get_param("/smach/adjust_steering_angle", 1.22)
        self.adjust_throttle = rospy.get_param("/smach/adjust_throttle", 0.05)
        self.Throttle = Throttle

        self.start_position = None
        self.start_time = None
        self.single_cargo_height = tar_height

        if goal == 'delivery_point':
            self.delivery_point_mode = 1
        elif goal=="conveyer_belt_point":
            self.delivery_point_mode = 2

        self.start_time_2 = None
        self.cur_height = 0
        self.last_check_time_2 = None
        self.previous_changed_height = None
        self.ipc_state = 0

    def clamp_state_callback(self, msg):
        self.cur_width = msg.data

    def updown_state_callback(self, msg):
        self.cur_height = msg.data
        self.changed_height = abs(self.tar_height - self.cur_height)

    def ipc_state_callback(self, msg):
        self.ipc_state = msg.data

    def steering_callback(self, msg):
        self.steering_state = msg.data


    def execute(self, userdata):
        global num_work
        global position_, bales_num_
        global current_x, current_y, current_yaw
        global delivery_point, conveyer_belt_point
        # 获取目标的位置和航向
        if self.delivery_point_mode == 1:
            self.goal = delivery_point
        elif self.delivery_point_mode == 2:
            self.goal = conveyer_belt_point
        else:
            rospy.logwarn("Invalid parameter for conveyer_belt_point.")

        # 等待最新的定位点
        time.sleep(1)

        # print(self.delivery_point_mode)
        goal_point1 = self.goal[0]
        goal_point2 = self.goal[1]
        # 从四元数获取目标点的航向
        goal_quaternion = goal_point2
        _, _, goal_yaw = tf.euler_from_quaternion([goal_quaternion[0], goal_quaternion[1], goal_quaternion[2], goal_quaternion[3]])
        x1, y1, _ = goal_point1
        goal_slope = math.tan(goal_yaw)

        self.distance_result = point_to_line_distance(current_x, current_y, x1, y1, -1/goal_slope)
        
        error_dis = self.distance_result
        self.target_distance = error_dis

        rospy.loginfo("\033[33m"+"任务下发成功"+"\033[0m")
        if self.start_position is None:
            self.start_position = Point(current_x, current_y, 0.0)

        if self.target_distance < 0:
            GEAR = 1
        else:
            GEAR = 3

        count = 0  # 初始化计数器
        path_msg = Path()
        path_msg.header.frame_id = 'map'

        # 根据目标距离和步长计算路径点数量
        num_points = int(abs(self.target_distance) / 0.1)
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
            # print("new_x: ", new_x)
            # print("new_y: ", new_y)

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


        def first_loop():
            count = 0
            count2 = 0
            while True:
                if (self.ipc_state == 1):
                    distance_ = point_to_line_distance(current_x, current_y, x1, y1, -1/goal_slope)

                    if (self.steering_state is None):
                        rospy.loginfo("\033[31m"+"wait steering"+"\033[0m")
                    else:
                        count += 1  # 每次循环递增计数器
                        if count % 50 == 0:  # 每隔100次打印一次
                            rospy.loginfo("\033[32m" + f"Target:{self.target_distance:.3f}" "\033[0m")
                            rospy.loginfo("\033[33m" + f"Current:{self.distance:.3f}" "\033[0m")

                        current_position = Point(current_x, current_y, 0.0)
                        dx = current_position.x - self.start_position.x
                        dy = current_position.y - self.start_position.y
                        self.distance = ((dx ** 2) + (dy ** 2)) ** 0.5

                        if self.distance >= abs(self.target_distance):
                            self.steering_pub.publish(0.0)
                            self.throttle_pub.publish(0.0)
                            gear_cmd = UInt8(GEAR)
                            self.gear_pub.publish(gear_cmd)
                            self.brake_pub.publish(4.0)
                            # 发布路径消息
                            self.path_publisher.publish(path_msg)
                            if self.start_time is None:
                                self.start_time = rospy.Time.now()
                            if (rospy.Time.now() - self.start_time).to_sec() > 0.5:
                                rospy.loginfo("\033[33m" + f"last Current:{self.distance:.3f}" "\033[0m")
                                break
                        else:
                            # 运动之前先检查转角是否摆正
                            if (abs(self.steering_state)>0.1):
                                self.steering_pub.publish(0.0)
                                self.throttle_pub.publish(0.0)
                                gear_cmd = UInt8(GEAR)
                                self.gear_pub.publish(gear_cmd)
                                self.brake_pub.publish(0.0)
                                # 发布路径消息
                                self.path_publisher.publish(path_msg)
                            else:
                                if (distance_<=3):
                                    self.steering_pub.publish(0.0)
                                    self.throttle_pub.publish(0.2)
                                    gear_cmd = UInt8(GEAR)
                                    self.gear_pub.publish(gear_cmd)
                                    self.brake_pub.publish(0.0)
                                    # 发布路径消息
                                    self.path_publisher.publish(path_msg)

                                else:
                                    self.steering_pub.publish(0.0)
                                    self.throttle_pub.publish(0.5)
                                    gear_cmd = UInt8(GEAR)
                                    self.gear_pub.publish(gear_cmd)
                                    self.brake_pub.publish(0.0)
                                    # 发布路径消息
                                    self.path_publisher.publish(path_msg)
                        
                else:
                    rospy.loginfo_once("\033[31m"+"Wait For auto_drive"+"\033[0m")
                rospy.Rate(100).sleep()

        def second_loop():
            count = 0
            count2 = 0
            while True:
                # 空的路径
                path_msg = Path()
                path_msg.header.frame_id = 'map'
                self.path_publisher.publish(path_msg)

                # 先循环发布210才能进无人使能
                self.set_tar_height_pub_.publish(2)
                self.set_tar_height_pub.publish(self.single_cargo_height)

                if (self.ipc_state == 1):
                    if self.start_time_2 is None:
                        self.start_time_2 = rospy.Time.now()
                    # 先循环发布一段时间
                    self.set_tar_height_pub_.publish(2)
                    self.set_tar_height_pub.publish(self.single_cargo_height)

                    if (rospy.Time.now() - self.start_time_2).to_sec() > 1.0:
                        current_time = rospy.Time.now()
                        if self.last_check_time_2 is None or (current_time - self.last_check_time_2).to_sec() >= 1.0:
                            current_changed_height = abs(self.tar_height - self.cur_height)
                            if self.previous_changed_height is not None:
                                difference = abs(current_changed_height - self.previous_changed_height)
                                if difference < 10:
                                    self.start_time_2 = None
                                    self.last_check_time_2 = None
                                    self.previous_changed_height = None
                                    break
                                self.previous_changed_height = current_changed_height
                            else:
                                self.previous_changed_height = current_changed_height
                            self.last_check_time_2 = current_time

                    count2 += 1
                    if count2 % 100 == 0:
                        rospy.loginfo(f" \033[32m cur_height:{(self.cur_height):.3f} \033[0m")
                        rospy.loginfo(f" \033[32m tar_height:{(self.tar_height):.3f} \033[0m")
                else:
                    rospy.loginfo_once("\033[31m"+"Wait For auto_drive"+"\033[0m")
                rospy.Rate(100).sleep()

        thread1 = threading.Thread(target=first_loop)
        thread2 = threading.Thread(target=second_loop)
        thread1.start()
        thread2.start()
        thread1.join()
        thread2.join()

        self.send_plan_task_service(0)
        self.send_control_task_service(0)
        self.new_point_delivery = []
        self.new_point_reindex = []
        # 到达终点->开始夹取
        if not self.brake_enable: 
            self.start_position = None
            self.steering_state = None
            self.start_time = None
            rospy.loginfo_once("\033[32m"+'Position reached, switching to local_planning'+"\033[0m")

            return 'set_width_height'
        # 如果有刹车
        elif self.brake_enable:
            self.start_position = None
            self.steering_state = None
            self.start_time = None
            rospy.loginfo_once('AEB')
            userdata.last_state = 'MOVING_TO_PICKUP_LOCATION'
            return 'braking'



# 放货需要单独做一个，因为放货也需要垂直转向
class GlobalDeliveryTask1(smach.State, SerManager):
    def __init__(self, goal=None):
        smach.State.__init__(self, outcomes=['braking', 'set_width_height'], input_keys=['points'])
        SerManager.__init__(self)

        if goal == 'delivery_point':
            self.delivery_point_mode = 1  # 保存目标点数据
        self.goal = goal
        self.new_point_delivery = []
        self.new_point_reindex = []
        
        self.current_yaw_reverse_distance = rospy.get_param("/smach/current_yaw_reverse_distance", 0.8)
        self.goal_yaw_reverse_distance = rospy.get_param("/smach/goal_yaw_reverse_distance", -0.8)
        self.lateral_distance = rospy.get_param("/smach/lateral_distance", 1.0)

        self.path_publisher = rospy.Publisher("/smach_path", Path, queue_size=1)

    def execute(self, userdata):
        global position_, bales_num_, delivery_point
        global num_work, work_direction

        self.task = 1
        self.send_plan_task_service(self.task)
        self.send_control_task_service(self.task)

        if self.delivery_point_mode == 1:
            self.goal = delivery_point
        
        # # 推算途经的点
        # if (work_direction==1):# 左边放
        #     self.goal = calculate_new_point(self.goal, 1, self.lateral_distance*num_work) #1 右边
        #     reversed_point = calculate_reverse_point(self.goal, 1.5)
        #     left_reversed_point  = calculate_new_point(reversed_point, 2, 3.0) #2 左边
        #     right_rotation_left_reversed_point = calculate_new_point_rotation(left_reversed_point, 1) #1顺时针
        #     reversed_right_rotation_left_reversed_point = calculate_reverse_point(right_rotation_left_reversed_point, 6.0)
        #     self.new_point_reindex.append(reversed_right_rotation_left_reversed_point)
        # elif (work_direction==2):
        #     self.goal = calculate_new_point(self.goal, 2, self.lateral_distance*num_work) #2 左边
        #     reversed_point = calculate_reverse_point(self.goal, 1.5)
        #     left_reversed_point  = calculate_new_point(reversed_point, 1, 3.0) #1 右边
        #     right_rotation_left_reversed_point = calculate_new_point_rotation(left_reversed_point, 2) #2逆时针
        #     reversed_right_rotation_left_reversed_point = calculate_reverse_point(right_rotation_left_reversed_point, 6.0)
        #     self.new_point_reindex.append(reversed_right_rotation_left_reversed_point)
        # else:
        #     rospy.logwarn("no work direction input, please check!!!")
        reversed_point = calculate_reverse_point(self.goal, 3)
        reversed_reversed_point = calculate_reverse_point(self.goal, 5)
        self.new_point_reindex.append(reversed_reversed_point)


        for point_data, quaternion_data in self.new_point_reindex:
            point = Point(*point_data)
            quaternion = Quaternion(*quaternion_data)
            self.send_init_pose(point, quaternion)
            rospy.loginfo("\033[32m"+"途经点位发布成功"+"\033[0m")

        goal_point, goal_quaternion = self.goal#reversed_point
        goal_pose = Point(goal_point[0], goal_point[1], goal_point[2])
        goal_orientation = Quaternion(goal_quaternion[0], goal_quaternion[1], goal_quaternion[2], goal_quaternion[3])
        self.send_target_pose(goal_pose, goal_orientation)
        rospy.loginfo("\033[32m"+"目标点位发布成功"+"\033[0m")

        count = 0
        while position_ != 1 and not self.brake_enable:
            # 空的路径
            path_msg = Path()
            path_msg.header.frame_id = 'map'
            self.path_publisher.publish(path_msg)
            
            count += 1 
            if count % 100 == 0:
                rospy.loginfo_once("\033[32m Go to global endpoint \033[0m")
            rospy.Rate(100).sleep()
        
        if position_ == 1 and not self.brake_enable: 
            rospy.loginfo_once("\033[32m"+'Position reached, switching to local_planning'+"\033[0m")
            self.task = 0
            self.send_plan_task_service(self.task)
            self.send_control_task_service(self.task)
            self.new_point_delivery = []
            self.new_point_reindex = []
            # self.send_camera_task_service(self.task)
            position_ = 0
            bales_num_ = 0
            
            return 'set_width_height'
        # 如果有刹车
        elif self.brake_enable:
            rospy.loginfo_once('AEB')
            userdata.last_state = 'MOVING_TO_PICKUP_LOCATION'
            return 'braking'



# 放货需要单独做一个，因为放货涉及到位置的改变
class GlobalDeliveryTask2(smach.State, SerManager):
    def __init__(self, goal=None,tar_height=None,updown_mode=None):
        smach.State.__init__(self, outcomes=['braking', 'set_width_height'], input_keys=['points'])
        SerManager.__init__(self)
        # self.points = points if points is not None else []
        self.goal = goal
        
        if goal == 'delivery_point':
            self.delivery_point_mode = 1
        elif goal=="conveyer_belt_point":
            self.delivery_point_mode = 2

        self.new_point_delivery = []
        self.new_point_reindex = []
        
        self.current_yaw_reverse_distance = rospy.get_param("/smach/current_yaw_reverse_distance", 0.8)
        self.goal_yaw_reverse_distance = rospy.get_param("/smach/goal_yaw_reverse_distance", -0.8)
        self.lateral_distance = rospy.get_param("/smach/lateral_distance", 1.0)
        self.path_publisher = rospy.Publisher("/smach_path", Path, queue_size=1)

        self.steering_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=1)
        self.throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=1)
        self.gear_pub = rospy.Publisher('/gear_cmd', UInt8, queue_size=1)
        self.brake_pub = rospy.Publisher('/brake_cmd', Float64, queue_size=1)
        self.start_position = None
        
        self.cur_height = 0
        self.start_time = None
        self.previous_changed_height = None
        self.last_check_time = None
        self.tar_height = tar_height

        # self.timer = timer
        self.set_tar_height_pub = rospy.Publisher('/updown_cmd', Float64, queue_size=1)
        self.set_tar_height_pub_ = rospy.Publisher('/updown_mode', Int8, queue_size=1)
        
        self.updown_mode = updown_mode
        rospy.Subscriber('/updown_state', Float64, self.updown_state_callback)
        rospy.Subscriber('/clamp_state', Float64, self.clamp_state_callback)
        self.set_tar_width_pub = rospy.Publisher('/clamp_cmd', Float64, queue_size=1)
        self.set_tar_fy_pub = rospy.Publisher('/fy_cmd', Float64, queue_size=1)
        self.set_tar_lateral_pub = rospy.Publisher('/lateral_cmd', Float64, queue_size=1)
        rospy.Subscriber('/ipc_state', Int8, self.ipc_state_callback)

        # 创建publisher对象
        # 规划的topic TODO
        self.path_pub = rospy.Publisher("/global_path", Path, queue_size=10)

        # 两个之间的货物高度
        self.height_between_two_cargos = rospy.get_param('/real/height_between_two_cargos', 234)
        # 放货的高度
        self.above_two_cargos_height_max = rospy.get_param('/real/above_two_cargos_height_max', 1125)
        # 最终放货高度
        self.pickoff_height_ = rospy.get_param('/real/pickoff_height_', 35)

        self.ipc_state = 0
        self.start_time_2 = None
        self.last_check_time_2 = None

        self.cur_width = 0
        self.previous_changed_width = None


    def clamp_state_callback(self, msg):
        self.cur_width = msg.data

    def updown_state_callback(self, msg):
        self.cur_height = msg.data
        self.changed_height = abs(self.tar_height - self.cur_height)

    def ipc_state_callback(self, msg):
        self.ipc_state = msg.data

    def execute(self, userdata):
        global position_, bales_num_
        global num_work
        global current_x, current_y, current_yaw, delivery_point, work_direction
        global num_work, num_work_max, num_work_height
        global conveyer_belt_point
        

        if self.delivery_point_mode == 1:
            self.goal = delivery_point
        elif self.delivery_point_mode == 2:
            self.goal = conveyer_belt_point
        else:
            rospy.logwarn("Invalid parameter for conveyer_belt_point.")

        goal_reversed = calculate_reverse_point(self.goal,3)
        # 规划的轨迹
        trajectory = generate_trajectory(goal_reversed, self.goal, step_size=0.1)
       
        # 创建Path消息
        path_msg = Path()
        path_msg.header.frame_id = "map"  # 假设路径是在'map'坐标系下
        path_msg.header.stamp = rospy.Time.now()
        # 遍历轨迹中的所有点并添加到Path消息中
        for pose in trajectory:
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "map"
            pose_msg.header.stamp = path_msg.header.stamp
            pose_msg.pose.position.x = pose[0][0]
            pose_msg.pose.position.y = pose[0][1]
            pose_msg.pose.position.z = pose[0][2]
            pose_msg.pose.orientation.x = pose[1][0]
            pose_msg.pose.orientation.y = pose[1][1]
            pose_msg.pose.orientation.z = pose[1][2]
            pose_msg.pose.orientation.w = pose[1][3]
            path_msg.poses.append(pose_msg)

        # 发布控制任务，不发规划任务
        self.task = 1
        self.send_plan_task_service(0)
        self.send_control_task_service(self.task)

        # 发布路径
        rate = rospy.Rate(10)  # 10hz
        while position_ == 0 and not self.brake_enable:
            self.path_pub.publish(path_msg)
            # 等待到达终点
            count = 0
            # 空的路径
            if(position_==1):
                break
            count += 1
            if count % 100 == 0:
                rospy.loginfo("\033[32m" + f"Go to global endpoint: {position_}"+ "\033[0m")
            rate.sleep()

        self.send_plan_task_service(0)
        self.send_control_task_service(0)
        self.new_point_delivery = []
        self.new_point_reindex = []
        
        # 到达终点->开始夹取
        if not self.brake_enable:
            position_ = 0 
            conveyer_belt_point = None
            self.start_position = None
            rospy.loginfo_once("\033[32m"+'Position reached, switching to local_planning'+"\033[0m")
            return 'set_width_height'
        # 如果有刹车
        elif self.brake_enable:
            position_ = 0
            conveyer_belt_point = None
            self.start_position = None
            rospy.loginfo_once('AEB')
            userdata.last_state = 'MOVING_TO_PICKUP_LOCATION'
            return 'braking'


class GlobalDeliveryTask3(smach.State, SerManager):
    def __init__(self, goal=None, Throttle=None):
        smach.State.__init__(self, outcomes=['braking', 'set_width_height'])
        SerManager.__init__(self)
        # 初始化相关变量
        self.conveyer_belt_yaw = None
        self.current_yaw = None
        self.steering_state = None

        self.steering_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=1)
        rospy.Subscriber('/steering_state', Float64, self.steering_callback)
        self.throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=1)
        self.gear_pub = rospy.Publisher('/gear_cmd', UInt8, queue_size=1)
        self.brake_pub = rospy.Publisher('/brake_cmd', Float64, queue_size=1)
        self.path_publisher = rospy.Publisher('/smach_path', Path, queue_size=1)

        self.adjust_steering_angle = rospy.get_param("/smach/adjust_steering_angle", 1.22)
        self.adjust_throttle = rospy.get_param("/smach/adjust_throttle", 0.05)
        self.Throttle = Throttle

        self.start_position = None

        if goal == 'delivery_point':
            self.delivery_point_mode = 1
        elif goal=="conveyer_belt_point":
            self.delivery_point_mode = 2

    def steering_callback(self, msg):
        self.steering_state = msg.data


    def execute(self, userdata):
        global num_work
        global position_, bales_num_
        global current_x, current_y, current_yaw
        global delivery_point, conveyer_belt_point
        # 获取目标的位置和航向
        if self.delivery_point_mode == 1:
            self.goal = delivery_point
        elif self.delivery_point_mode == 2:
            self.goal = conveyer_belt_point
        else:
            rospy.logwarn("Invalid parameter for conveyer_belt_point.")

        # 等待最新的定位点
        time.sleep(1)

        # print(self.delivery_point_mode)
        goal_point1 = self.goal[0]
        goal_point2 = self.goal[1]
        # 从四元数获取目标点的航向
        goal_quaternion = goal_point2
        _, _, goal_yaw = tf.euler_from_quaternion([goal_quaternion[0], goal_quaternion[1], goal_quaternion[2], goal_quaternion[3]])
        x1, y1, _ = goal_point1
        goal_slope = math.tan(goal_yaw)

        self.distance_result = point_to_line_distance(current_x, current_y, x1, y1, -1/goal_slope)
        # 如果距离不够那么先倒车不够的距离：
        error_dis = self.distance_result
        self.target_distance = error_dis - 0.1 # 少10cm左右 ##########################test 0.15

        rospy.loginfo("\033[33m"+"任务下发成功"+"\033[0m")
        if self.start_position is None:
            self.start_position = Point(current_x, current_y, 0.0)

        if self.target_distance < 0:
            GEAR = 1
        else:
            GEAR = 3

        count = 0  # 初始化计数器
        path_msg = Path()
        path_msg.header.frame_id = 'map'

        # 根据目标距离和步长计算路径点数量
        num_points = int(abs(self.target_distance) / 0.1)
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
            # print("new_x: ", new_x)
            # print("new_y: ", new_y)

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
        
        while True:
            if (self.steering_state is None):
                rospy.loginfo("\033[31m"+"wait steering"+"\033[0m")
            else:
                count += 1  # 每次循环递增计数器
                if count % 50 == 0:  # 每隔100次打印一次
                    rospy.loginfo("\033[32m" + f"Target:{self.target_distance:.3f}" "\033[0m")
                    rospy.loginfo("\033[33m" + f"Current:{self.distance:.3f}" "\033[0m")

                current_position = Point(current_x, current_y, 0.0)
                dx = current_position.x - self.start_position.x
                dy = current_position.y - self.start_position.y
                self.distance = ((dx ** 2) + (dy ** 2)) ** 0.5

                if self.distance >= abs(self.target_distance):
                    break
                else:
                    if (abs(self.steering_state)>0.1):
                        steering_cmd = Float64(0.0)
                        self.steering_pub.publish(steering_cmd)
                        throttle_cmd = Float64(0.0)
                        self.throttle_pub.publish(throttle_cmd)
                        gear_cmd = UInt8(GEAR)
                        self.gear_pub.publish(gear_cmd)
                        brake_cmd = Float64(0.0)
                        self.brake_pub.publish(brake_cmd)
                        # 发布路径消息
                        self.path_publisher.publish(path_msg)
                    else:
                        if (self.distance<=3.0):
                            steering_cmd = Float64(0.0)
                            self.steering_pub.publish(steering_cmd)
                            throttle_cmd = Float64(self.Throttle)
                            self.throttle_pub.publish(throttle_cmd)
                            gear_cmd = UInt8(GEAR)
                            self.gear_pub.publish(gear_cmd)
                            brake_cmd = Float64(0.0)
                            self.brake_pub.publish(brake_cmd)
                            # 发布路径消息
                            self.path_publisher.publish(path_msg)
                        else:
                            steering_cmd = Float64(0.0)
                            self.steering_pub.publish(steering_cmd)
                            throttle_cmd = Float64(0.5)
                            self.throttle_pub.publish(throttle_cmd)
                            gear_cmd = UInt8(GEAR)
                            self.gear_pub.publish(gear_cmd)
                            brake_cmd = Float64(0.0)
                            self.brake_pub.publish(brake_cmd)
                            # 发布路径消息
                            self.path_publisher.publish(path_msg)
            rospy.Rate(100).sleep()

        # 到达终点->开始夹取
        if not self.brake_enable: 
            self.start_position = None
            self.steering_state = None
            rospy.loginfo_once("\033[32m"+'Position reached, switching to local_planning'+"\033[0m")

            return 'set_width_height'
        # 如果有刹车
        elif self.brake_enable:
            self.start_position = None
            self.steering_state = None
            rospy.loginfo_once('AEB')
            userdata.last_state = 'MOVING_TO_PICKUP_LOCATION'
            return 'braking'


# 通过rviz给出规划的点位，不需要输入点
class GlobalTaskRviz(smach.State, SerManager):
    def __init__(self, points=None, goal=None):
        smach.State.__init__(self, outcomes=['braking', 'set_width_height'], input_keys=['points'])
        SerManager.__init__(self)
        self.points = points if points is not None else []  # 保存点数据，如果没有提供则为空列表
        self.goal = goal  # 保存目标点数据

    def execute(self, userdata):
        global position_
        rospy.loginfo_once("\033[32m"+'Executing state: 前往取货区任务开始'+"\033[0m")
        rospy.loginfo("\033[32m"+"下发全局任务"+"\033[0m")
        self.task = 1
        self.send_plan_task_service(self.task)
        self.send_control_task_service(self.task)
        
        # self.send_camera_task_service(self.task)
        rospy.loginfo("\033[32m"+"任务下发成功"+"\033[0m")
        
        # 如果提供了途经点，则下发全局规划点
        if self.points:
            rospy.loginfo("\033[32m"+"开始全局任务，下发全局规划点"+"\033[0m")
            for point_data, quaternion_data in self.points:
                point = Point(point_data[0], point_data[1], point_data[2])
                quaternion = Quaternion(quaternion_data[0], quaternion_data[1], quaternion_data[2], quaternion_data[3])
                self.send_init_pose(point, quaternion)
                rospy.loginfo("\033[32m"+"途经点位发布成功"+"\033[0m")
        
        # 无论是否提供途经点，都下发目标点
        rospy.loginfo("\033[32m"+"下发目标点位"+"\033[0m")
        goal_point, goal_quaternion = self.goal
        goal_pose = Point(goal_point[0], goal_point[1], goal_point[2])
        goal_orientation = Quaternion(goal_quaternion[0], goal_quaternion[1], goal_quaternion[2], goal_quaternion[3])
        self.send_target_pose(goal_pose, goal_orientation)
        rospy.loginfo("\033[32m"+"目标点位发布成功"+"\033[0m")
        rospy.loginfo("\033[32m"+"开始全局任务，请从RVIZ下发全局规划点"+"\033[0m")

        count = 0
        while position_ != 1 and not self.brake_enable:
            count += 1
            if count % 100 == 0:
                rospy.loginfo_once("\033[32m Go to global endpoint \033[0m")
            rospy.Rate(100).sleep()


        if position_ == 1 and not self.brake_enable: 
            rospy.loginfo_once("\033[32m"+'Position reached, switching to local_planning'+"\033[0m")
            self.task = 0
            self.send_plan_task_service(self.task)
            self.send_control_task_service(self.task)
            self.new_point_delivery = []
            self.new_point_reindex = []
            position_ = 0
            bales_num_ = 0
            return 'set_width_height'

        elif self.brake_enable:
            rospy.loginfo_once('AEB')
            userdata.last_state = 'MOVING_TO_PICKUP_LOCATION'
            return 'braking'


# 局部任务，通过相机识别来进行局部规划，当前版本未使用
class LocalTask(smach.State, SerManager):
    def __init__(self):
        smach.State.__init__(self, outcomes=['braking', 'set_width_height'], input_keys=['points'])
        SerManager.__init__(self)
        # self.position_state_service = rospy.Service("position_service", PositionTask, self.handle_position_state)

    def execute(self, userdata):
        rospy.loginfo_once("\033[32m"+'Executing state: 前往取货区任务开始'+"\033[0m")
        rospy.loginfo("\033[32m"+"下发全局任务"+"\033[0m")
        self.task = 2
        self.send_plan_task_service(self.task)
        self.send_control_task_service(self.task)
        self.send_camera_task_service(self.task)

        rospy.loginfo("\033[32m"+"任务下发成功"+"\033[0m")
        
        # 等待完成全局规划任务
        while self.position_state != 2 and not self.brake_enable:
            rospy.loginfo_once("\033[32m Go to global endpoint \033[0m")
            time.sleep(1)
        
        # 到达终点->开始夹取
        if self.position_state == 2 and not self.brake_enable: 
            rospy.loginfo_once("\033[32m"+'Position reached, switching to local_planning'+"\033[0m")
            # self.position_state_service.shutdown("Shutting down the service")
            self.task = 0
            self.send_plan_task_service(self.task)
            self.send_control_task_service(self.task)
            return 'set_width_height'
        # 如果有刹车
        elif self.brake_enable:
            rospy.loginfo_once('AEB')
            userdata.last_state = 'MOVING_TO_PICKUP_LOCATION'
            return 'braking'


class Braking(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['moving_to_pickup_location'])
        self.decision_sub_ = rospy.Subscriber('/emergency_brake_from_decision', Decision, self.decision_callback)
        self.brake_enable = 0

    def decision_callback(self, msg):
        self.brake_enable = msg.brake_enable

    def execute(self, userdata):
        rospy.loginfo("\033[32m"+'Executing state: BRAKING'+"\033[0m")
        
        while self.brake_enable:
            rospy.sleep(1) #等待brake消消失
            break

        # 状态切换
        if not self.brake_enable and userdata.last_state == 'MOVING_TO_PICKUP_LOCATION':
            rospy.loginfo("\033[32m"+'Braking complete, switching to MOVING_TO_PICKUP_LOCATION'+"\033[0m")
            return 'moving_to_pickup_location'
        # elif not self.brake_enable and userdata.last_state == 'MOVING_TO_DROPOFF_LOCATION':
        #     rospy.loginfo("\033[32m"+'Braking complete, switching to MOVING_TO_DROPOFF_LOCATION'+"\033[0m")
        #     return 'moving_to_dropoff_location'
        

class SetWidth(smach.State):
    def __init__(self, tar_width):
        smach.State.__init__(self, outcomes=['finish'])
        self.tar_width = tar_width
        # self.timer = timer
        self.set_tar_width_pub = rospy.Publisher('/clamp_cmd', Float64, queue_size=1)
        rospy.Subscriber('/clamp_state', Float64, self.clamp_state_callback)
        self.set_tar_height_pub = rospy.Publisher('/updown_cmd', Float64, queue_size=1)
        self.set_tar_height_pub_ = rospy.Publisher('/updown_mode', Int8, queue_size=1)

        self.set_tar_fy_pub = rospy.Publisher('/fy_cmd', Float64, queue_size=1)
        self.set_tar_lateral_pub = rospy.Publisher('/lateral_cmd', Float64, queue_size=1)
        rospy.Subscriber('/ipc_state', Int8, self.ipc_state_callback)

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
        rospy.loginfo_once("\033[32m"+'Executing state: SET_WIDTH'+"\033[0m")
        count = 0  # 初始化计数器
        
        while True:
            # 先循环发布210才能进无人使能
            self.set_tar_width_pub.publish(self.tar_width)
            self.set_tar_height_pub_.publish(2)

            if (self.ipc_state == 1):
                if self.start_time is None:
                    self.start_time = rospy.Time.now()
                # 先循环发布一段时间
                self.set_tar_width_pub.publish(self.tar_width)
                self.set_tar_height_pub_.publish(2)

                # 0.5s 过后，每 0.1s 检查一次宽度变化
                if (rospy.Time.now() - self.start_time).to_sec() > 1.0:
                    current_time = rospy.Time.now()
                    if self.last_check_time is None or (current_time - self.last_check_time).to_sec() >= 1:
                        current_changed_width = abs(self.tar_width - self.cur_width)
                        # print("current_changed_width: ", current_changed_width)
                        if self.previous_changed_width is not None:
                            difference = abs(current_changed_width - self.previous_changed_width)
                            # print("difference: ", difference)
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

        rospy.loginfo_once("\033[32m"+"Width set successfully"+"\033[0m")
        self.start_time = None
        return 'finish'


class SetHeight(smach.State):
    def __init__(self, tar_height,updown_mode):
        smach.State.__init__(self, outcomes=['finish'])
        self.tar_height = tar_height
        # self.timer = timer
        self.set_tar_height_pub = rospy.Publisher('/updown_cmd', Float64, queue_size=1)
        rospy.Subscriber('/updown_state', Float64, self.updown_state_callback)
        self.set_tar_width_pub = rospy.Publisher('/clamp_cmd', Float64, queue_size=1)
        self.set_tar_fy_pub = rospy.Publisher('/fy_cmd', Float64, queue_size=1)
        self.set_tar_lateral_pub = rospy.Publisher('/lateral_cmd', Float64, queue_size=1)
        rospy.Subscriber('/ipc_state', Int8, self.ipc_state_callback)
        self.path_publisher = rospy.Publisher('/smach_path', Path, queue_size=1)
        self.set_tar_height_pub_ = rospy.Publisher('/updown_mode', Int8, queue_size=1)
        self.updown_mode = updown_mode

        # 两个之间的货物高度
        self.height_between_two_cargos = rospy.get_param('/real/height_between_two_cargos', 234)

        # 放货的高度
        self.above_two_cargos_height_max = rospy.get_param('/real/above_two_cargos_height_max', 1125)

        # 最终放货高度
        self.pickoff_height_ = rospy.get_param('/real/pickoff_height_', 35)


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
        global num_work, num_work_max, num_work_height

        if (self.tar_height==self.height_between_two_cargos):
            if(num_work_height==0):
                self.tar_height = self.height_between_two_cargos
            if(num_work_height==1):
                self.tar_height = self.height_between_two_cargos + 953
            if(num_work_height==2):
                self.tar_height = self.height_between_two_cargos + 1900

        if(self.tar_height==self.pickoff_height_):
            if(num_work_height==0):
                self.tar_height = self.pickoff_height_
            if(num_work_height==1):
                self.tar_height = self.pickoff_height_ + 1138
            if(num_work_height==2):
                self.tar_height = self.pickoff_height_ + 1900


        if(self.tar_height==self.above_two_cargos_height_max):
            if(num_work_height==0):
                self.tar_height = self.above_two_cargos_height_max
            if(num_work_height==1):
                self.tar_height = self.above_two_cargos_height_max + 1042
            if(num_work_height==2):
                self.tar_height = self.above_two_cargos_height_max + 1042


        count = 0  # 初始化计数器
        rospy.loginfo_once("\033[32m"+'Executing state: SET_HEIGHT'+"\033[0m")
        while True:
            # 空的路径
            path_msg = Path()
            path_msg.header.frame_id = 'map'
            self.path_publisher.publish(path_msg)
            
            # 先循环发布210才能进无人使能
            self.set_tar_height_pub.publish(self.tar_height)
            self.set_tar_height_pub_.publish(self.updown_mode)

            if (self.ipc_state == 1):
                if self.start_time is None:
                    self.start_time = rospy.Time.now()
                # 先循环发布一段时间
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


        rospy.loginfo_once("\033[32m"+"Height set successfully"+"\033[0m")
        return 'finish'



class SetHeightWeight(smach.State):
    def __init__(self, tar_height, tar_width,updown_mode):
        smach.State.__init__(self, outcomes=['finish'])
        self.tar_height = tar_height
        self.tar_width = tar_width

        # self.timer = timer
        self.set_tar_height_pub = rospy.Publisher('/updown_cmd', Float64, queue_size=1)
        self.set_tar_height_pub_ = rospy.Publisher('/updown_mode', Int8, queue_size=1)
        
        self.updown_mode = updown_mode
        rospy.Subscriber('/updown_state', Float64, self.updown_state_callback)
        rospy.Subscriber('/clamp_state', Float64, self.clamp_state_callback)

        self.set_tar_width_pub = rospy.Publisher('/clamp_cmd', Float64, queue_size=1)
        self.set_tar_fy_pub = rospy.Publisher('/fy_cmd', Float64, queue_size=1)
        self.set_tar_lateral_pub = rospy.Publisher('/lateral_cmd', Float64, queue_size=1)
        rospy.Subscriber('/ipc_state', Int8, self.ipc_state_callback)
        self.path_publisher = rospy.Publisher('/smach_path', Path, queue_size=1)

        self.cur_height = 0
        self.ipc_state = 0
        self.start_time = None
        self.previous_changed_height = None
        self.last_check_time = None

        self.start_time_2 = None
        self.last_check_time_2 = None

        self.cur_width = 0
        self.previous_changed_width = None

    def clamp_state_callback(self, msg):
        self.cur_width = msg.data
        self.changed_width = abs(self.tar_width - self.cur_width)

    def updown_state_callback(self, msg):
        self.cur_height = msg.data
        self.changed_height = abs(self.tar_height - self.cur_height)


    def ipc_state_callback(self, msg):
        self.ipc_state = msg.data

    def execute(self, userdata):
        global num_work, num_work_max, num_work_height

        rospy.loginfo_once("\033[32m"+'Executing state: SET_HEIGHT_WIDTH'+"\033[0m")

        def first_loop():
            count = 0  # 初始化计数器
            while True:
                if (self.tar_width==0):
                    break
                # 先循环发布210才能进无人使能
                self.set_tar_width_pub.publish(self.tar_width)
                self.set_tar_height_pub_.publish(self.updown_mode)
                if (self.ipc_state == 1):
                    if self.start_time is None:
                        self.start_time = rospy.Time.now()
                    # 先循环发布一段时间
                    self.set_tar_width_pub.publish(self.tar_width)
                    self.set_tar_height_pub_.publish(self.updown_mode)
                    # 0.5s 过后，每 0.1s 检查一次宽度变化
                    if (rospy.Time.now() - self.start_time).to_sec() > 1.0:
                        current_time = rospy.Time.now()
                        if self.last_check_time is None or (current_time - self.last_check_time).to_sec() >= 1.0:
                            current_changed_width = abs(self.tar_width - self.cur_width)
                            if self.previous_changed_width is not None:
                                difference = abs(current_changed_width - self.previous_changed_width)
                                # print("difference: ", difference)
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

        def second_loop():
            count2 = 0
            while True:
                if (self.tar_height==0):
                    break
                # 空的路径
                path_msg = Path()
                path_msg.header.frame_id = 'map'
                self.path_publisher.publish(path_msg)
                # 先循环发布210才能进无人使能
                self.set_tar_height_pub.publish(self.tar_height)                

                if (self.ipc_state == 1):
                    if self.start_time_2 is None:
                        self.start_time_2 = rospy.Time.now()
                    # 先循环发布一段时间
                    self.set_tar_height_pub.publish(self.tar_height)
                    # self.set_tar_height_pub_.publish(self.updown_mode)

                    if (rospy.Time.now() - self.start_time_2).to_sec() > 1.0:
                        current_time = rospy.Time.now()
                        if self.last_check_time_2 is None or (current_time - self.last_check_time_2).to_sec() >= 1.0:
                            current_changed_height = abs(self.tar_height - self.cur_height)
                            if self.previous_changed_height is not None:
                                difference = abs(current_changed_height - self.previous_changed_height)
                                if difference < 10:
                                    self.start_time_2 = None
                                    self.last_check_time_2 = None
                                    self.previous_changed_height = None
                                    break
                                self.previous_changed_height = current_changed_height
                            else:
                                self.previous_changed_height = current_changed_height
                            self.last_check_time_2 = current_time

                    count2 += 1
                    if count2 % 100 == 0:
                        rospy.loginfo(f" \033[32m cur_height:{(self.cur_height):.3f} \033[0m")
                        rospy.loginfo(f" \033[32m tar_height:{(self.tar_height):.3f} \033[0m")
                else:
                    rospy.loginfo_once("\033[31m"+"Wait For auto_drive"+"\033[0m")
                rospy.Rate(100).sleep()

        thread1 = threading.Thread(target=first_loop)
        thread2 = threading.Thread(target=second_loop)
        thread1.start()
        thread2.start()
        thread1.join()
        thread2.join()

        rospy.loginfo_once("\033[32m"+"Height set successfully"+"\033[0m")
        return 'finish'


class SetFY(smach.State):
    def __init__(self, tar_fy):
        smach.State.__init__(self, outcomes=['finish'])
        self.tar_fy = tar_fy
        # self.timer = timer
        self.set_tar_height_pub = rospy.Publisher('/updown_cmd', Float64, queue_size=1)
        rospy.Subscriber('/fy_state', Float64, self.fy_state_callback)
        self.set_tar_width_pub = rospy.Publisher('/clamp_cmd', Float64, queue_size=1)
        self.set_tar_fy_pub = rospy.Publisher('/fy_cmd', Float64, queue_size=1)
        self.set_tar_lateral_pub = rospy.Publisher('/lateral_cmd', Float64, queue_size=1)
        self.cur_fy = 0
        self.start_time = None
        self.previous_changed_fy = None
        self.last_check_time = None

    def fy_state_callback(self, msg):
        self.cur_fy = msg.data
        self.changed_fy = abs(self.tar_fy - self.cur_fy)

    def execute(self, userdata):
        count = 0
        rospy.loginfo_once("\033[32m"+'Executing state: SET_FY'+"\033[0m")
        while True:
            if self.start_time is None:
                self.start_time = rospy.Time.now()
            self.set_tar_fy_pub.publish(self.tar_fy)

            if (rospy.Time.now() - self.start_time).to_sec() > 3:
                current_time = rospy.Time.now()
                if self.last_check_time is None or (current_time - self.last_check_time).to_sec() >= 1:
                    current_changed_fy = abs(self.tar_fy - self.cur_fy)
                    if self.previous_changed_fy is not None:
                        difference = abs(current_changed_fy - self.previous_changed_fy)
                        if difference < 10:
                            break
                        self.previous_changed_fy = current_changed_fy
                    else:
                        self.previous_changed_fy = current_changed_fy
                    self.last_check_time = current_time

            count += 1
            if count % 100 == 0:
                rospy.loginfo(f"\033[33m cur_fy:{self.cur_fy} tar_fy:{self.tar_fy} \033[0m")
            rospy.Rate(100).sleep()

        rospy.loginfo_once("\033[32m"+"FY set successfully"+"\033[0m")
        return 'finish'



class SetWidthSim(smach.State):
    def __init__(self, tar_width):
        smach.State.__init__(self, outcomes=['finish'])
        self.tar_width = tar_width
        self.pub_left_width = rospy.Publisher('/base_link_to_fork_left_horizontal_controller/command', Float64, queue_size=10)
        self.pub_right_width = rospy.Publisher('/base_link_to_fork_right_horizontal_controller/command', Float64, queue_size=10)
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        self.cur_width = 0
        self.start_time = None
        self.previous_changed_width = None
        self.last_check_time = None
        

    def joint_state_callback(self, msg):
        self.cur_width = msg.position[msg.name.index('base_link_to_fork_left_horizontal')]  # 替换为实际关节名称
        self.changed_width = abs(self.tar_width - self.cur_width)


    def execute(self, userdata):
        global num_work
        rospy.loginfo_once("\033[32m"+'Executing state: SET_WIDTH_SIM'+"\033[0m")
        
        
        while True:
            count = 0
            if self.start_time is None:
                self.start_time = rospy.Time.now()
            self.pub_left_width.publish(Float64(self.tar_width))
            self.pub_right_width.publish(Float64(-self.tar_width))

            if (rospy.Time.now() - self.start_time).to_sec() > 1:
                current_time = rospy.Time.now()
                if self.last_check_time is None or (current_time - self.last_check_time).to_sec() >= 1: # 3s监控一次
                    current_changed_width = abs(self.tar_width - self.cur_width)
                    # print("current_changed_width: ", current_changed_width)
                    if self.previous_changed_width is not None:
                        difference = abs(current_changed_width - self.previous_changed_width)
                        # print("difference: ", difference)
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
                rospy.loginfo(f" \033[33m cur_height:{(self.cur_width):.3f} \033[0m")
                rospy.loginfo(f" \033[33m tar_height:{(self.tar_width):.3f} \033[0m")

            rospy.Rate(100).sleep()
        rospy.loginfo_once("\033[32m"+"Width set successfully"+"\033[0m")
        return 'finish'


class SetHeightSim(smach.State):
    def __init__(self, tar_height):
        smach.State.__init__(self, outcomes=['finish'])
        self.tar_height = tar_height
        self.pub_left_vertical = rospy.Publisher('/fork_left_horizontal_to_fork_left_vertical_controller/command', Float64, queue_size=10)
        self.pub_right_vertical = rospy.Publisher('/fork_right_horizontal_to_fork_right_vertical_controller/command', Float64, queue_size=10)
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        self.cur_height = 0.0  # 初始化当前高度
        self.start_time = None
        self.previous_changed_height = None
        self.last_check_time = None

    def joint_state_callback(self, msg):
        self.cur_height = msg.position[msg.name.index('fork_left_horizontal_to_fork_left_vertical')]  # 替换为实际关节名称
        self.changed_height = abs(self.tar_height - self.cur_height)

    def execute(self, userdata):
        count = 0
        rospy.loginfo_once("\033[32m"+'Executing state: SET_HEIGHT_SIM'+"\033[0m")
        while True:
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
        return 'finish'

# 开始
class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start'])
    def execute(self, userdata):
        rospy.loginfo("\033[32m"+'Executing state: Start'+"\033[0m")
        return 'start'
    
# 自检
# 俯仰，夹紧,降低的自检
class SelfInspection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finish'])

        single_cargo_height = rospy.get_param('/real/single_cargo_height', 40)
        self.tar_height = single_cargo_height+30
        self.tar_width = 1
        self.tar_fy = 1111
        self.updown_mode = 2

        self.set_tar_height_pub = rospy.Publisher('/updown_cmd', Float64, queue_size=1)
        self.set_tar_height_pub_ = rospy.Publisher('/updown_mode', Int8, queue_size=1)
        
        rospy.Subscriber('/updown_state', Float64, self.updown_state_callback)
        rospy.Subscriber('/clamp_state', Float64, self.clamp_state_callback)

        self.set_tar_width_pub = rospy.Publisher('/clamp_cmd', Float64, queue_size=1)
        self.set_tar_fy_pub = rospy.Publisher('/fy_cmd', Float64, queue_size=1)
        self.set_tar_lateral_pub = rospy.Publisher('/lateral_cmd', Float64, queue_size=1)
        rospy.Subscriber('/ipc_state', Int8, self.ipc_state_callback)
        self.path_publisher = rospy.Publisher('/smach_path', Path, queue_size=1)

        self.cur_height = 0
        self.ipc_state = 0
        self.start_time = None
        self.previous_changed_height = None
        self.last_check_time = None

        self.start_time_2 = None
        self.last_check_time_2 = None

        self.cur_width = 0
        self.previous_changed_width = None

    def clamp_state_callback(self, msg):
        self.cur_width = msg.data
        self.changed_width = abs(self.tar_width - self.cur_width)

    def updown_state_callback(self, msg):
        self.cur_height = msg.data
        self.changed_height = abs(self.tar_height - self.cur_height)

    def ipc_state_callback(self, msg):
        self.ipc_state = msg.data

    def execute(self, userdata):
        global num_work, num_work_max, num_work_height

        rospy.loginfo_once("\033[32m"+'Executing state: SelfInspection'+"\033[0m")

        def first_loop():
            count = 0  # 初始化计数器
            while True:
                if (self.tar_width==0):
                    break
                # 先循环发布210才能进无人使能
                self.set_tar_width_pub.publish(self.tar_width)
                self.set_tar_height_pub_.publish(1)
                self.set_tar_fy_pub.publish(self.tar_fy)
                if (self.ipc_state == 1):
                    if self.start_time is None:
                        self.start_time = rospy.Time.now()
                    # 先循环发布一段时间
                    self.set_tar_width_pub.publish(self.tar_width)
                    self.set_tar_height_pub_.publish(1)
                    self.set_tar_fy_pub.publish(self.tar_fy)

                    # 0.5s 过后，每 0.1s 检查一次宽度变化
                    if (rospy.Time.now() - self.start_time).to_sec() > 2.0:
                        current_time = rospy.Time.now()
                        if self.last_check_time is None or (current_time - self.last_check_time).to_sec() >= 1.0:
                            current_changed_width = abs(self.tar_width - self.cur_width)
                            if self.previous_changed_width is not None:
                                difference = abs(current_changed_width - self.previous_changed_width)
                                # print("difference: ", difference)
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

        def second_loop():
            count2 = 0
            while True:
                if (self.tar_height==0):
                    break
                # 空的路径
                path_msg = Path()
                path_msg.header.frame_id = 'map'
                self.path_publisher.publish(path_msg)
                # 先循环发布210才能进无人使能
                self.set_tar_height_pub.publish(self.tar_height)                

                if (self.ipc_state == 1):
                    if self.start_time_2 is None:
                        self.start_time_2 = rospy.Time.now()
                    # 先循环发布一段时间
                    self.set_tar_height_pub.publish(self.tar_height)

                    # self.set_tar_height_pub_.publish(self.updown_mode)

                    if (rospy.Time.now() - self.start_time_2).to_sec() > 1.0:
                        current_time = rospy.Time.now()
                        if self.last_check_time_2 is None or (current_time - self.last_check_time_2).to_sec() >= 1.0:
                            current_changed_height = abs(self.tar_height - self.cur_height)
                            if self.previous_changed_height is not None:
                                difference = abs(current_changed_height - self.previous_changed_height)
                                if difference < 10:
                                    self.start_time_2 = None
                                    self.last_check_time_2 = None
                                    self.previous_changed_height = None
                                    break
                                self.previous_changed_height = current_changed_height
                            else:
                                self.previous_changed_height = current_changed_height
                            self.last_check_time_2 = current_time

                    count2 += 1
                    if count2 % 100 == 0:
                        rospy.loginfo(f" \033[32m cur_height:{(self.cur_height):.3f} \033[0m")
                        rospy.loginfo(f" \033[32m tar_height:{(self.tar_height):.3f} \033[0m")
                else:
                    rospy.loginfo_once("\033[31m"+"Wait For auto_drive"+"\033[0m")
                rospy.Rate(100).sleep()

        thread1 = threading.Thread(target=first_loop)
        thread2 = threading.Thread(target=second_loop)
        thread1.start()
        thread2.start()
        thread1.join()
        thread2.join()

        rospy.loginfo_once("\033[32m"+"Height set successfully"+"\033[0m")
        return 'finish'


# 完成状态
class Finish(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeed'])
    def execute(self, userdata):
        rospy.loginfo("\033[32m"+'Executing state: FINISH'+"\033[0m")
        return 'succeed'


class DistanceCheck(smach.State, SerManager):
    def __init__(self, goal=None, Throttle=None,distance=None, first_time_observer_flag=None):
        smach.State.__init__(self, outcomes=['braking', 'finish'])
        SerManager.__init__(self)
        # 初始化相关变量
        self.conveyer_belt_yaw = None
        self.current_yaw = None
        self.steering_state = None
        self.distance_check = distance
        self.re_first_time_observer_flag = first_time_observer_flag

        self.steering_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=1)
        # rospy.Subscriber('/steering_state', Float64, self.steering_callback)
        self.throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=1)
        self.gear_pub = rospy.Publisher('/gear_cmd', UInt8, queue_size=1)
        self.brake_pub = rospy.Publisher('/brake_cmd', Float64, queue_size=1)
        self.path_publisher = rospy.Publisher('/smach_path', Path, queue_size=1)
        self.current_yaw_reverse_distance = rospy.get_param("/smach/current_yaw_reverse_distance", 0.8)
        self.goal_yaw_reverse_distance = rospy.get_param("/smach/goal_yaw_reverse_distance", -0.8)
        self.adjust_steering_angle = rospy.get_param("/smach/adjust_steering_angle", 1.22)
        self.adjust_throttle = rospy.get_param("/smach/adjust_throttle", 0.05)
        self.Throttle = Throttle
        self.distance = 0
        self.start_position = None
        self.start_time = None

        if goal == 'delivery_point':
            self.delivery_point_mode = 1
        elif goal=="conveyer_belt_point":
            self.delivery_point_mode = 2


    def execute(self, userdata):
        global num_work,num_work_height
        global position_, bales_num_
        global current_x, current_y, current_yaw
        global delivery_point, conveyer_belt_point,first_time_observer,observer_point
        
        if (self.re_first_time_observer_flag):
            first_time_observer = True
        
        # 获取目标的位置和航向
        if self.delivery_point_mode == 1:
            self.goal = delivery_point
        elif self.delivery_point_mode == 2:
            self.goal = conveyer_belt_point
        else:
            rospy.logwarn("Invalid parameter for conveyer_belt_point.")

        rospy.logwarn(f"num_work{num_work}, num_work_height{num_work_height}.")
        goal_point1 = self.goal[0]
        goal_point2 = self.goal[1]
        # 从四元数获取目标点的航向
        goal_quaternion = goal_point2
        _, _, goal_yaw = tf.euler_from_quaternion([goal_quaternion[0], goal_quaternion[1], goal_quaternion[2], goal_quaternion[3]])
        x1, y1, _ = goal_point1
        goal_slope = math.tan(goal_yaw)

        self.distance_result = point_to_line_distance(current_x, current_y, x1, y1, -1/goal_slope)
        rospy.loginfo("\033[33m" + f"Need Distance: {(self.distance_result):.3f}" "\033[0m")

        # 如果距离不够那么先倒车不够的距离：
        if (self.distance_result < self.distance_check):
            error_dis = self.distance_check - self.distance_result #稳定多倒10cm 然后
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
        path_msg = Path()
        path_msg.header.frame_id = 'map'

        # 根据目标距离和步长计算路径点数量
        num_points = int(abs(self.target_distance) / 0.1)
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
            # print("new_x: ", new_x)
            # print("new_y: ", new_y)

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
    
        rospy.loginfo("\033[32m" + f"Target:{self.target_distance:.3f}" "\033[0m")
        rospy.loginfo("\033[33m" + f"Current:{self.distance:.3f}" "\033[0m")

        while True:
            count += 1  # 每次循环递增计数器
            if count % 100 == 0:  # 每隔100次打印一次
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
                self.path_publisher.publish(path_msg)
            
            rospy.Rate(100).sleep()

        # 记录此时的观察点作为以后的观察点：
        if (first_time_observer):
            observer_point = process_point(current_x, current_y, current_yaw)
            print("observer_point", observer_point)
            first_time_observer = False
            # 只记录一次观察点

        # 到达终点->开始夹取
        if not self.brake_enable: 
            self.start_position = None
            self.distance = 0
            self.start_time = None
            return 'finish'
        # 如果有刹车
        elif self.brake_enable:
            self.start_position = None
            self.start_time = None
            self.distance = 0
            rospy.loginfo_once('AEB')
            userdata.last_state = 'MOVING_TO_PICKUP_LOCATION'
            return 'braking'


class DistanceCheck2(smach.State, SerManager):
    def __init__(self, goal=None,Throttle=None):
        smach.State.__init__(self, outcomes=['braking', 'finish'])
        SerManager.__init__(self)
        # 初始化相关变量
        self.conveyer_belt_yaw = None
        self.current_yaw = None
        self.steering_state = None

        self.steering_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=1)
        # rospy.Subscriber('/steering_state', Float64, self.steering_callback)
        self.throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=1)
        self.gear_pub = rospy.Publisher('/gear_cmd', UInt8, queue_size=1)
        self.brake_pub = rospy.Publisher('/brake_cmd', Float64, queue_size=1)
        self.path_publisher = rospy.Publisher('/smach_path', Path, queue_size=1)

        self.adjust_steering_angle = rospy.get_param("/smach/adjust_steering_angle", 1.22)
        self.adjust_throttle = rospy.get_param("/smach/adjust_throttle", 0.05)
        self.current_yaw_reverse_distance = rospy.get_param("/smach/current_yaw_reverse_distance", 0.8)
        self.goal_yaw_reverse_distance = rospy.get_param("/smach/goal_yaw_reverse_distance", -0.8)
        self.start_position = None
        self.start_time = None
        self.distance = 0
        self.Throttle = Throttle

        if goal == 'delivery_point':
            self.delivery_point_mode = 1
        elif goal=="conveyer_belt_point":
            self.delivery_point_mode = 2


    def execute(self, userdata):
        global num_work
        global position_, bales_num_
        global current_x, current_y, current_yaw
        global delivery_point, conveyer_belt_point
        # 获取目标的位置和航向
        if self.delivery_point_mode == 1:
            self.goal = delivery_point
        elif self.delivery_point_mode == 2:
            self.goal = conveyer_belt_point
        else:
            rospy.logwarn("Invalid parameter for conveyer_belt_point.")

        # 等待最新的定位点
        time.sleep(1)
        # print(self.delivery_point_mode)
        goal_point1 = self.goal[0]
        goal_point2 = self.goal[1]
        # 从四元数获取目标点的航向
        goal_quaternion = goal_point2
        _, _, goal_yaw = tf.euler_from_quaternion([goal_quaternion[0], goal_quaternion[1], goal_quaternion[2], goal_quaternion[3]])
        x1, y1, _ = goal_point1
        goal_slope = math.tan(goal_yaw)

        self.distance_result = point_to_line_distance(current_x, current_y, x1, y1, goal_slope)
        rospy.loginfo("\033[33m" + f"Current Distance2: {self.distance_result:.3f}" "\033[0m")
        rospy.loginfo("\033[33m" + f"Need Distance2: {abs(self.current_yaw_reverse_distance):.3f}" "\033[0m")

        # 如果距离不够那么先倒车不够的距离：
        if (self.distance_result < abs(self.current_yaw_reverse_distance)):
            error_dis = abs(self.current_yaw_reverse_distance) - self.distance_result + 0.4 #稳定多倒10cm 然后
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
        path_msg = Path()
        path_msg.header.frame_id = 'map'

        # 根据目标距离和步长计算路径点数量
        num_points = int(abs(self.target_distance) / 0.1)
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
            # print("new_x: ", new_x)
            # print("new_y: ", new_y)

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
        
        while True:
            count += 1  # 每次循环递增计数器
            
            if count % 100 == 0:  # 每隔100次打印一次
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
                self.path_publisher.publish(path_msg)
            
            rospy.Rate(100).sleep()

        # 到达终点->开始夹取
        if not self.brake_enable: 
            rospy.loginfo_once("\033[32m"+'Position reached, switching to local_planning'+"\033[0m")
            self.start_position = None
            self.start_time = None
            self.distance = 0
            return 'finish'
        # 如果有刹车
        elif self.brake_enable:
            self.start_position = None
            self.start_time = None
            self.distance = 0
            rospy.loginfo_once('AEB')
            userdata.last_state = 'MOVING_TO_PICKUP_LOCATION'
            return 'braking'


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
            quaternion_tar = yaw_to_quaternion(yaw_tar)
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


class HandleCloudOrder(smach.State, SerManager):
    def __init__(self):
        smach.State.__init__(self, outcomes=['A0'],
                             output_keys=['delivery_point','task_id']
                             )
        SerManager.__init__(self)
        
        self.steering_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=1)
        self.throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=1)
        self.gear_pub = rospy.Publisher('/gear_cmd', UInt8, queue_size=1)
        self.brake_pub = rospy.Publisher('/brake_cmd', Float64, queue_size=1)

        self.ori_point = None
        self.start_time = None
        self.ori_quaternion = None
        self.tar_point = None
        self.tar_quaternion = None
        self.delivery_point = delivery_point

        
    def execute(self, userdata):
        global delivery_point, work_direction
        global num_work,num_work_max,num_work_height

        rospy.loginfo("\033[32m"+'Executing state: HANDLE_CLOUD_ORDER'+"\033[0m")
        self.task = 0
        self.send_plan_task_service(self.task)
        self.send_control_task_service(self.task)
        
        if self.start_time is None:
            self.start_time = rospy.Time.now()
        # rospy.loginfo("\033[32m"+'Test1111111'+"\033[0m")
        while True:
            # 先发布1s一下空挡，不要再倒挡启动车辆
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
                # rospy.loginfo("\033[32m"+'Test2222222222222'+"\033[0m")
                break
            rospy.Rate(100).sleep()


        while delivery_point == ():
            rospy.loginfo_once("\033[91m 等待云端任务... \033[0m")
            rospy.sleep(1)

        rospy.loginfo("\033[32m"+'收到云端任务'+"\033[0m")
        rospy.loginfo("\033[33m"+f'取货点:{delivery_point}'+"\033[0m")


        self.delivery_point = delivery_point
        if shelf_name:
            num_work = int(str(shelf_name)[-1]) - 1
            work_direction = ord(shelf_name[0]) % 2 + 1
            num_work_height = int(str(shelf_name)[2]) -1
            rospy.loginfo(f'num_work:{num_work}  work_direction:{work_direction}  num_work_height:{num_work_height}   ')
        self.start_time = None
        
        return 'A0'
    
class CloudTaskFeedback(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'],input_keys=['task_id'])

    def send_task_sts_message(self, req):
        rospy.wait_for_service('task_sts_service')
        try:
            task_sts_service = rospy.ServiceProxy('task_sts_service', TaskSts)
            response = task_sts_service(req)
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def execute(self, userdata):
        global delivery_point
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
            if success:
                rospy.loginfo("Task status message sent successfully.")
                delivery_point = ()
                return 'success'
        except :
            rospy.logerr("No task_id found in userdata. Skipping task status update.")
            return 'success'


#---------------------------lib-------------------------------
def signal_handler(sig, frame):
    print(f"\nReceived {sig} signal, exiting gracefully.")
    sys.exit(0)


def yaw_to_quaternion(yaw):
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

def calculate_heading(point_start, point_end):
    """计算从起点到终点的航向角"""
    dx = point_end[0] - point_start[0]
    dy = point_end[1] - point_start[1]
    return math.atan2(dy, dx)

def generate_trajectory(pointA, pointB, step_size=0.1):
    """生成从pointA到pointB的离散轨迹，每个点都有坐标和航向"""
    start_pos = pointA[0][:2]  # 只取x,y坐标
    end_pos = pointB[0][:2]    # 只取x,y坐标
    
    yaw = calculate_heading(start_pos, end_pos)
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

def calculate_reverse_point(point_goal, distance):
    x, y, z, w = point_goal[1]
    yaw = math.atan2(2 * (w * z), 1 - 2 * z * z)
    x_new = point_goal[0][0] - distance * math.cos(yaw)
    y_new = point_goal[0][1] - distance * math.sin(yaw)
    return ((x_new, y_new, 0.0), (0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2)))


def calculate_new_point(point_goal, direction, distance):
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


def calculate_new_point_rotation(point_goal, direction):
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

def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

def calculate_optimal_rotation(current_yaw, goal_yaw):
    raw_angle = goal_yaw - current_yaw
    
    normalized_angle = normalize_angle(raw_angle)
    
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

def process_point(intersection_x, intersection_y, intersection_yaw):
    quat = tf.quaternion_from_euler(0, 0, intersection_yaw)
    return ((intersection_x, intersection_y, 0.0), (quat[0], quat[1], quat[2], quat[3]))

def point_to_line_distance(current_x, current_y, x1, y1, goal_slope):
    A = goal_slope
    B = -1
    C = y1 - goal_slope * x1
    distance = abs(A * current_x + B * current_y + C) / math.sqrt(A ** 2 + B ** 2)

    return distance

def create_path_segment(x1, y1, x2, y2, step=0.1):
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

def merge_paths(path1, path2):
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


def create_state_machine():
    global delivery_point, conveyer_belt_point, observer_point
    global work_direction, num_work, num_work_max, num_work_height

    ##---------------------param参数配置---------------------
    # 从参数服务器中获取参数：
    use_sim = rospy.get_param('/use_sim', False) 
    single_cargo_height_sim = rospy.get_param('/sim/single_cargo_height', 0.1)
    height_between_two_cargos_sim = rospy.get_param('/sim/height_between_two_cargos', 0.5)
    two_cargos_height_sim = rospy.get_param('/sim/two_cargos_height', 0.8)
    above_two_cargos_height_sim = rospy.get_param('/sim/above_two_cargos_height', 0.8)
    clamping_width_sim = rospy.get_param('/sim/clamping_width', 0.05)
    open_width_sim = rospy.get_param('/sim/open_width', 0.6)

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
    use_camera_det = rospy.get_param('/real/use_camera_det', True)
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
    
    global_task_instance = GlobalTaskInstance()
    # # 创建服务
    position_service = rospy.Service(
        'position_service', PositionTask, global_task_instance.handle_position_state
    )

    bales_num_in_camera_service = rospy.Service(
        'bales_num_in_camera_service', BalesNumInCamera, global_task_instance.handle_bales_num_state
    )

    odom_sub = rospy.Subscriber("/dis_odom",Odometry,  global_task_instance.odom_callback)
    
    handle_cloud_order_service = rospy.Service(
        'cloud_order_service', cloud_order, global_task_instance.handle_cloud_order)

    camera_service = rospy.Service(
        'fusion_det_task', FusionDetTask, global_task_instance.camera_cotton_callback)
    
    sm = smach.StateMachine(outcomes=['succeed', 'aborted', 'preempted'])
    
    ##---------------------状态机---------------------
    with sm:
        smach.StateMachine.add('SelfInspection', SelfInspection(), transitions={'finish':'START'})

        smach.StateMachine.add('START', Start(), transitions={'start':'H0'}) 

        smach.StateMachine.add('HANDLE_ClOUD_ORDER', HandleCloudOrder(), transitions={'A0':'H0'})
        
        smach.StateMachine.add('H0', SetHeightWeight(single_cargo_height+30, open_width,updown_mode=2), transitions={'finish': 'C0'})

        if (use_camera_det):
            smach.StateMachine.add('C0', CAMERA_TASK(), transitions={'finish':'DC'})


        smach.StateMachine.add('DC', DistanceCheck(goal="conveyer_belt_point", Throttle=0.3,distance=2.5, first_time_observer_flag=False),
                                transitions={'braking':'BRAKING',
                                            'finish':'G0'})

        smach.StateMachine.add('G0', GlobalPickupTask_1(goal="conveyer_belt_point", throttle=0.3),
                                transitions={'braking':'BRAKING', 
                                            'set_width_height':'W0'})

        if(not use_sim):
            # 取一个包
            smach.StateMachine.add('W0', SetWidth(clamping_width), transitions={'finish': 'H1'})
            # 升高，准备后退
            smach.StateMachine.add('H1', SetHeight(height_between_two_cargos+5,updown_mode=2), transitions={'finish': 'G1'})
        else:
            smach.StateMachine.add('H1', SetHeightSim(height_between_two_cargos_sim), transitions={'finish': 'W1'})
            smach.StateMachine.add('W1', SetWidthSim(clamping_width_sim), transitions={'finish': 'H2'})
            smach.StateMachine.add('H2', SetHeightSim(height_between_two_cargos_sim), transitions={'finish': 'G1'})
        

        smach.StateMachine.add('G1', GlobalPickupTask_2(goal="conveyer_belt_point", throttle=0.4),
                                transitions={'braking':'BRAKING', 
                                            'set_width_height':'W01'},
                                remapping={'input_userdata':'last_state'})
        
        smach.StateMachine.add('W01', SetHeight(170+5,updown_mode=2), transitions={'finish': 'C1'})

        # 等待识别
        if (use_camera_det):
            smach.StateMachine.add('C1', CAMERA_TASK(), transitions={'finish':'G2'})

        # 规划继续取第二个包
        smach.StateMachine.add('G2', GlobalPickupTask_3(goal="conveyer_belt_point", Throttle=0.3, tar_height=above_two_cargos_height,updown_mode=2,tar_width=0),
                                transitions={'braking':'BRAKING', 
                                            'set_width_height':'H2'},
                                remapping={'input_userdata':'last_state'})
        
        if(not use_sim):
            # 叠放 参考值
            smach.StateMachine.add('H2', SetHeight(two_cargos_height,updown_mode=2), transitions={'finish': 'W1'})
            # 张开 参考值
            smach.StateMachine.add('W1', SetWidth(open_width_), transitions={'finish': 'M1'})
        else:
            smach.StateMachine.add('H4', SetHeightSim(two_cargos_height_sim), transitions={'finish': 'W2'})

        # 倒车
        smach.StateMachine.add('M1', GlobalMoveTask(target_distance=(-0.1), Throttle=0.2, tar_height=0, updown_mode=0,tar_width=0),
                                transitions={'braking':'BRAKING',
                                            'set_width_height':'H3'},
                                remapping={'input_userdata':'last_state'})

        # 下降
        smach.StateMachine.add('H3', SetHeight(single_cargo_height_half,updown_mode=1), transitions={'finish': 'G3'})

        # # 前进，顶齐
        smach.StateMachine.add('G3', GlobalMoveTask(target_distance=(0.2+0.15), Throttle=0.3, tar_height=0, updown_mode=0,tar_width=0),
                                transitions={'braking':'BRAKING',
                                            'set_width_height':'W2'},
                                remapping={'input_userdata':'last_state'})
                                
        if(not use_sim):
            # 夹紧，参考值：
            smach.StateMachine.add('W2', SetWidth(clamping_width), transitions={'finish': 'H4'})
            # 升高 参考值：
            smach.StateMachine.add('H4', SetHeight(height_between_two_cargos+1,updown_mode=2), transitions={'finish': 'G4'})
        else:
            smach.StateMachine.add('W4', SetWidthSim(clamping_width_sim), transitions={'finish': 'H6'})
            smach.StateMachine.add('H6', SetHeightSim(above_two_cargos_height_sim), transitions={'finish': 'G5'})


        # TODO NEW STATE:
        smach.StateMachine.add('G4', TemporaryParking1(goal="observer_point", throttle=0.4),
                                transitions={'braking':'BRAKING', 
                                            'set_width_height':'AD0'},
                                remapping={'input_userdata':'last_state'})

        # 调整航向
        smach.StateMachine.add('AD0', AdjustHeading(goal='observer_point',changeYawangle=-90,gear=3,throttle=0.1,control_com=0.103),
                                transitions={'braking':'BRAKING',
                                            'finish':'G5'},
                                remapping={'input_userdata':'last_state'})
        
        # 前进2m
        smach.StateMachine.add('G5', GlobalMoveTask(target_distance=(2.0), Throttle=0.3, tar_height=0, updown_mode=0,tar_width=0),
                                transitions={'braking':'BRAKING',
                                            'set_width_height':'H5'},
                                remapping={'input_userdata':'last_state'})
        
        # 动作
        # 放下货物
        smach.StateMachine.add('H5', SetHeight(pickoff_height_,updown_mode=2), transitions={'finish': 'W3'})
        # 张开
        smach.StateMachine.add('W3', SetWidth(open_width_pickoff), transitions={'finish': 'M2'})
        
        # 倒车2m
        smach.StateMachine.add('M2', GlobalMoveTask(target_distance=(-2), Throttle=0.3, tar_height=0, updown_mode=0,tar_width=0),
                                transitions={'braking':'BRAKING',
                                            'set_width_height':'A01'},
                                remapping={'input_userdata':'last_state'})
        
        # 调整航向
        smach.StateMachine.add('A01', AdjustHeading(goal='observer_point',changeYawangle=0,gear=1,throttle=0.1,control_com=0.103),
                                transitions={'braking':'BRAKING',
                                            'finish':'M5'},
                                remapping={'input_userdata':'last_state'})
        
        # 前进2.3m
        smach.StateMachine.add('M5', GlobalMoveTask(target_distance=(2.3), Throttle=0.3, tar_height=0, updown_mode=0,tar_width=0),
                                transitions={'braking':'BRAKING',
                                            'set_width_height':'H6'},
                                remapping={'input_userdata':'last_state'})
        
        # 初始取货宽高
        smach.StateMachine.add('H6', SetHeightWeight(single_cargo_height+30, open_width,updown_mode=2), transitions={'finish': 'C2'})

        # 相机使能与识别
        if (use_camera_det):
            smach.StateMachine.add('C2', CAMERA_TASK(), transitions={'finish':'G6'})

        # 规划前往第一个进行取包
        smach.StateMachine.add('G6', GlobalPickupTask_1(goal="conveyer_belt_point", throttle=0.3),
                                transitions={'braking':'BRAKING', 
                                            'set_width_height':'W4'},
                                remapping={'input_userdata':'last_state'})

        if(not use_sim):
            # 取一个包
            smach.StateMachine.add('W4', SetWidth(clamping_width), transitions={'finish': 'H7'})
            # 升高，准备后退
            smach.StateMachine.add('H7', SetHeight(height_between_two_cargos+5,updown_mode=2), transitions={'finish': 'G7'})
        else:
            smach.StateMachine.add('H1', SetHeightSim(height_between_two_cargos_sim), transitions={'finish': 'W1'})
            smach.StateMachine.add('W1', SetWidthSim(clamping_width_sim), transitions={'finish': 'H2'})
            smach.StateMachine.add('H2', SetHeightSim(height_between_two_cargos_sim), transitions={'finish': 'G1'})
        

        # 规划回到观察点 加速点
        # TODO 不要倒退太多 2m左右即可 不再依赖于观察点，依赖于我之前的识别的棉包位置conveyer_belt_point
        smach.StateMachine.add('G7', GlobalPickupTask_2(goal="conveyer_belt_point", throttle=0.4),
                                transitions={'braking':'BRAKING', 
                                            'set_width_height':'W02'},
                                remapping={'input_userdata':'last_state'})
        
        # 先下降防止遮挡
        smach.StateMachine.add('W02', SetHeight(170+5,updown_mode=2), transitions={'finish': 'C4'})

        # 等待识别
        if (use_camera_det):
            smach.StateMachine.add('C4', CAMERA_TASK(), transitions={'finish':'G8'})

        # 规划继续取第二个包
        smach.StateMachine.add('G8', GlobalPickupTask_3(goal="conveyer_belt_point", Throttle=0.3, tar_height=above_two_cargos_height,updown_mode=2,tar_width=0),
                                transitions={'braking':'BRAKING', 
                                            'set_width_height':'H8'},
                                remapping={'input_userdata':'last_state'})
        
        if(not use_sim):
            # 叠放 参考值
            smach.StateMachine.add('H8', SetHeight(two_cargos_height,updown_mode=2), transitions={'finish': 'W5'})
            # 张开 参考值
            smach.StateMachine.add('W5', SetWidth(open_width_), transitions={'finish': 'M3'})
        else:
            smach.StateMachine.add('H4', SetHeightSim(two_cargos_height_sim), transitions={'finish': 'W2'})

        # 倒车
        smach.StateMachine.add('M3', GlobalMoveTask(target_distance=(-0.1), Throttle=0.2, tar_height=0, updown_mode=0,tar_width=0),
                                transitions={'braking':'BRAKING',
                                            'set_width_height':'H9'},
                                remapping={'input_userdata':'last_state'})

        # 下降
        smach.StateMachine.add('H9', SetHeight(single_cargo_height_half,updown_mode=1), transitions={'finish': 'G9'})

        # # 前进，顶齐
        smach.StateMachine.add('G9', GlobalMoveTask(target_distance=(0.2+0.15), Throttle=0.3, tar_height=0, updown_mode=0,tar_width=0),
                                transitions={'braking':'BRAKING',
                                            'set_width_height':'W6'},
                                remapping={'input_userdata':'last_state'})
                                
        if(not use_sim):
            # 夹紧，参考值：
            smach.StateMachine.add('W6', SetWidth(clamping_width), transitions={'finish': 'H10'})
            # 升高 参考值：
            smach.StateMachine.add('H10', SetHeight(height_between_two_cargos+1,updown_mode=2), transitions={'finish': 'G10'})
        else:
            smach.StateMachine.add('W4', SetWidthSim(clamping_width_sim), transitions={'finish': 'H6'})
            smach.StateMachine.add('H6', SetHeightSim(above_two_cargos_height_sim), transitions={'finish': 'G5'})
        
        
        # TODO NEW STATE:
        smach.StateMachine.add('G10', TemporaryParking2(goal="observer_point", throttle=0.3),
                                transitions={'braking':'BRAKING', 
                                            'set_width_height':'A02'},
                                remapping={'input_userdata':'last_state'})

        # 调整航向
        smach.StateMachine.add('A02', AdjustHeading(goal='observer_point',changeYawangle=-90,gear=3,throttle=0.1,control_com=0.103),
                                transitions={'braking':'BRAKING',
                                            'finish':'G11'},
                                remapping={'input_userdata':'last_state'})
        
        # 前进2m
        smach.StateMachine.add('G11', GlobalMoveTask(target_distance=(2), Throttle=0.3, tar_height=0, updown_mode=0,tar_width=0),
                                transitions={'braking':'BRAKING',
                                            'set_width_height':'H11'},
                                remapping={'input_userdata':'last_state'})
        
        # 动作
        # 放下货物
        smach.StateMachine.add('H11', SetHeight(pickoff_height_,updown_mode=2), transitions={'finish': 'W7'})
        # 张开
        smach.StateMachine.add('W7', SetWidth(open_width_pickoff), transitions={'finish': 'M4'})
        
        # 倒车 准备前往第二个取货点位
        smach.StateMachine.add('M4', GlobalMoveTask(target_distance=(-2-1.5), Throttle=0.3, tar_height=0, updown_mode=0,tar_width=0),
                                transitions={'braking':'BRAKING',
                                            'set_width_height':'A03'},
                                remapping={'input_userdata':'last_state'})
        
        # TODO 准备夹取四个包
        # 原地转向
        smach.StateMachine.add('A03', AdjustHeading(goal='observer_point',changeYawangle=0,gear=1,throttle=0.1,control_com=0.103),
                                transitions={'braking':'BRAKING',
                                            'finish':'M6'},
                                remapping={'input_userdata':'last_state'})
        
        # 前进1.5m
        smach.StateMachine.add('M6', GlobalMoveTask(target_distance=(1.5), Throttle=0.3, tar_height=0, updown_mode=0,tar_width=0),
                                transitions={'braking':'BRAKING',
                                            'set_width_height':'H_6'},
                                remapping={'input_userdata':'last_state'})
        
        # 再次运行
        # 初始取货宽高
        smach.StateMachine.add('H_6', SetHeightWeight(single_cargo_height+30, open_width,updown_mode=2), transitions={'finish': 'C3'})

        # 相机使能与识别
        if (use_camera_det):
            smach.StateMachine.add('C3', CAMERA_TASK(goal=False), transitions={'finish':'DC_1'})

        smach.StateMachine.add('DC_1', DistanceCheck(goal="conveyer_belt_point", Throttle=0.3,distance=2.5, first_time_observer_flag=True),
                                transitions={'braking':'BRAKING',
                                            'finish':'G_0'},
                                remapping={'input_userdata':'last_state'})


        # 规划前往第一个进行取包
        smach.StateMachine.add('G_0', GlobalPickupTask_1(goal="conveyer_belt_point", throttle=0.3),
                                transitions={'braking':'BRAKING', 
                                            'set_width_height':'W_0'},
                                remapping={'input_userdata':'last_state'})

        smach.StateMachine.add('BRAKING', Braking(),transitions={'moving_to_pickup_location':'FINISH' })

        if(not use_sim):
            # 取一个包
            smach.StateMachine.add('W_0', SetWidth(clamping_width), transitions={'finish': 'H_1'})
            # 升高，准备后退
            smach.StateMachine.add('H_1', SetHeight(height_between_two_cargos+5,updown_mode=2), transitions={'finish': 'G_1'})
        else:
            smach.StateMachine.add('H1', SetHeightSim(height_between_two_cargos_sim), transitions={'finish': 'W1'})
            smach.StateMachine.add('W1', SetWidthSim(clamping_width_sim), transitions={'finish': 'H2'})
            smach.StateMachine.add('H2', SetHeightSim(height_between_two_cargos_sim), transitions={'finish': 'G1'})
        

        # 规划回到观察点 加速点
        # TODO 不要倒退太多 2m左右即可 不再依赖于观察点，依赖于我之前的识别的棉包位置conveyer_belt_point
        smach.StateMachine.add('G_1', GlobalPickupTask_2(goal="conveyer_belt_point", throttle=0.4),
                                transitions={'braking':'BRAKING', 
                                            'set_width_height':'W_01'},
                                remapping={'input_userdata':'last_state'})
        
        # 先下降防止遮挡
        smach.StateMachine.add('W_01', SetHeight(170+5,updown_mode=2), transitions={'finish': 'C_1'})

        # 等待识别
        if (use_camera_det):
            smach.StateMachine.add('C_1', CAMERA_TASK(), transitions={'finish':'G_2'})

        # 规划继续取第二个包
        smach.StateMachine.add('G_2', GlobalPickupTask_3(goal="conveyer_belt_point", Throttle=0.3, tar_height=above_two_cargos_height,updown_mode=2,tar_width=0),
                                transitions={'braking':'BRAKING', 
                                            'set_width_height':'H_2'},
                                remapping={'input_userdata':'last_state'})
        
        if(not use_sim):
            # 叠放 参考值
            smach.StateMachine.add('H_2', SetHeight(two_cargos_height,updown_mode=2), transitions={'finish': 'W_1'})
            # 张开 参考值
            smach.StateMachine.add('W_1', SetWidth(open_width_), transitions={'finish': 'M_1'})
        else:
            smach.StateMachine.add('H4', SetHeightSim(two_cargos_height_sim), transitions={'finish': 'W2'})

        # 倒车
        smach.StateMachine.add('M_1', GlobalMoveTask(target_distance=(-0.1), Throttle=0.2, tar_height=0, updown_mode=0,tar_width=0),
                                transitions={'braking':'BRAKING',
                                            'set_width_height':'H_3'},
                                remapping={'input_userdata':'last_state'})

        # 下降
        smach.StateMachine.add('H_3', SetHeight(single_cargo_height_half,updown_mode=1), transitions={'finish': 'G_3'})

        # # 前进，顶齐
        smach.StateMachine.add('G_3', GlobalMoveTask(target_distance=(0.2+0.15), Throttle=0.3, tar_height=0, updown_mode=0,tar_width=0),
                                transitions={'braking':'BRAKING',
                                            'set_width_height':'W_2'},
                                remapping={'input_userdata':'last_state'})
                                
        if(not use_sim):
            # 夹紧，参考值：
            smach.StateMachine.add('W_2', SetWidth(clamping_width), transitions={'finish': 'H_4'})
            # 升高 参考值：
            smach.StateMachine.add('H_4', SetHeight(height_between_two_cargos+1,updown_mode=2), transitions={'finish': 'G_4'})
        else:
            smach.StateMachine.add('W4', SetWidthSim(clamping_width_sim), transitions={'finish': 'H6'})
            smach.StateMachine.add('H6', SetHeightSim(above_two_cargos_height_sim), transitions={'finish': 'G5'})


        # TODO NEW STATE:
        smach.StateMachine.add('G_4', TemporaryParking1(goal="observer_point", throttle=0.4),
                                transitions={'braking':'BRAKING', 
                                            'set_width_height':'AD_0'},
                                remapping={'input_userdata':'last_state'})

        # 调整航向
        smach.StateMachine.add('AD_0', AdjustHeading(goal='observer_point',changeYawangle=-90,gear=3,throttle=0.1,control_com=0.103),
                                transitions={'braking':'BRAKING',
                                            'finish':'G_5'},
                                remapping={'input_userdata':'last_state'})
        
        # 前进2m
        smach.StateMachine.add('G_5', GlobalMoveTask(target_distance=(2.0), Throttle=0.3, tar_height=0, updown_mode=0,tar_width=0),
                                transitions={'braking':'BRAKING',
                                            'set_width_height':'H_5'},
                                remapping={'input_userdata':'last_state'})
        
        # 动作
        # 放下货物
        smach.StateMachine.add('H_5', SetHeight(pickoff_height_,updown_mode=2), transitions={'finish': 'W_3'})
        # 张开
        smach.StateMachine.add('W_3', SetWidth(open_width_pickoff), transitions={'finish': 'M_2'})
        
        # 倒车2m
        smach.StateMachine.add('M_2', GlobalMoveTask(target_distance=(-2), Throttle=0.3, tar_height=0, updown_mode=0,tar_width=0),
                                transitions={'braking':'BRAKING',
                                            'set_width_height':'A_01'},
                                remapping={'input_userdata':'last_state'})
        
        # 调整航向
        smach.StateMachine.add('A_01', AdjustHeading(goal='observer_point',changeYawangle=0,gear=1,throttle=0.1,control_com=0.103),
                                transitions={'braking':'BRAKING',
                                            'finish':'M7'},
                                remapping={'input_userdata':'last_state'})
        
        # 前进2.3m
        smach.StateMachine.add('M7', GlobalMoveTask(target_distance=(2.3), Throttle=0.3, tar_height=0, updown_mode=0,tar_width=0),
                                transitions={'braking':'BRAKING',
                                            'set_width_height':'H_61'},
                                remapping={'input_userdata':'last_state'})

        # 初始取货宽高
        smach.StateMachine.add('H_61', SetHeightWeight(single_cargo_height+30, open_width,updown_mode=2), transitions={'finish': 'C_2'})

        # 相机使能与识别
        if (use_camera_det):
            smach.StateMachine.add('C_2', CAMERA_TASK(), transitions={'finish':'G_6'})

        # 规划前往第一个进行取包
        smach.StateMachine.add('G_6', GlobalPickupTask_1(goal="conveyer_belt_point", throttle=0.3),
                                transitions={'braking':'BRAKING', 
                                            'set_width_height':'W_4'},
                                remapping={'input_userdata':'last_state'})

        if(not use_sim):
            # 取一个包
            smach.StateMachine.add('W_4', SetWidth(clamping_width), transitions={'finish': 'H_7'})
            # 升高，准备后退
            smach.StateMachine.add('H_7', SetHeight(height_between_two_cargos+5,updown_mode=2), transitions={'finish': 'G_7'})
        else:
            smach.StateMachine.add('H1', SetHeightSim(height_between_two_cargos_sim), transitions={'finish': 'W1'})
            smach.StateMachine.add('W1', SetWidthSim(clamping_width_sim), transitions={'finish': 'H2'})
            smach.StateMachine.add('H2', SetHeightSim(height_between_two_cargos_sim), transitions={'finish': 'G1'})
        

        # 规划回到观察点 加速点
        # TODO 不要倒退太多 2m左右即可 不再依赖于观察点，依赖于我之前的识别的棉包位置conveyer_belt_point
        smach.StateMachine.add('G_7', GlobalPickupTask_2(goal="conveyer_belt_point", throttle=0.4),transitions={'braking':'BRAKING', 'set_width_height':'W_02'})
        
        # 先下降防止遮挡
        smach.StateMachine.add('W_02', SetHeight(170+5,updown_mode=2), transitions={'finish': 'C_4'})

        # 等待识别
        if (use_camera_det):
            smach.StateMachine.add('C_4', CAMERA_TASK(), transitions={'finish':'G_8'})

        # 规划继续取第二个包
        smach.StateMachine.add('G_8', GlobalPickupTask_3(goal="conveyer_belt_point", Throttle=0.3, tar_height=above_two_cargos_height,updown_mode=2,tar_width=0),
                                transitions={'braking':'BRAKING', 
                                            'set_width_height':'H_8'},
                                remapping={'input_userdata':'last_state'})
        
        if(not use_sim):
            # 叠放 参考值
            smach.StateMachine.add('H_8', SetHeight(two_cargos_height,updown_mode=2), transitions={'finish': 'W_5'})
            # 张开 参考值
            smach.StateMachine.add('W_5', SetWidth(open_width_), transitions={'finish': 'M_3'})
        else:
            smach.StateMachine.add('H4', SetHeightSim(two_cargos_height_sim), transitions={'finish': 'W2'})

        # 倒车
        smach.StateMachine.add('M_3', GlobalMoveTask(target_distance=(-0.1), Throttle=0.2, tar_height=0, updown_mode=0,tar_width=0),
                                transitions={'braking':'BRAKING',
                                            'set_width_height':'H_9'},
                                remapping={'input_userdata':'last_state'})

        # 下降
        smach.StateMachine.add('H_9', SetHeight(single_cargo_height_half,updown_mode=1), transitions={'finish': 'G_9'})

        # # 前进，顶齐
        smach.StateMachine.add('G_9', GlobalMoveTask(target_distance=(0.2+0.15), Throttle=0.3, tar_height=0, updown_mode=0,tar_width=0),transitions={'braking':'BRAKING','set_width_height':'W_6'})
                                
        if(not use_sim):
            # 夹紧，参考值：
            smach.StateMachine.add('W_6', SetWidth(clamping_width), transitions={'finish': 'H_10'})
            # 升高 参考值：
            smach.StateMachine.add('H_10', SetHeight(height_between_two_cargos+1,updown_mode=2), transitions={'finish': 'G_10'})
        else:
            smach.StateMachine.add('W4', SetWidthSim(clamping_width_sim), transitions={'finish': 'H6'})
            smach.StateMachine.add('H6', SetHeightSim(above_two_cargos_height_sim), transitions={'finish': 'G5'})
        
        
        # TODO NEW STATE:
        smach.StateMachine.add('G_10', TemporaryParking4(goal="observer_point", throttle=0.3),
                                transitions={'braking':'BRAKING', 
                                            'set_width_height':'A_02'},
                                remapping={'input_userdata':'last_state'})

        # 调整航向
        smach.StateMachine.add('A_02', AdjustHeading(goal='observer_point',changeYawangle=-90,gear=3,throttle=0.1,control_com=0.103),
                                transitions={'braking':'BRAKING',
                                            'finish':'G_11'},
                                remapping={'input_userdata':'last_state'})
        
        # 前进2m
        smach.StateMachine.add('G_11', GlobalMoveTask(target_distance=(2), Throttle=0.3, tar_height=0, updown_mode=0,tar_width=0),
                                transitions={'braking':'BRAKING',
                                            'set_width_height':'H_11'},
                                remapping={'input_userdata':'last_state'})
        
        # 动作
        # 放下货物
        smach.StateMachine.add('H_11', SetHeight(pickoff_height_,updown_mode=2), transitions={'finish': 'W_7'})
        # 张开
        smach.StateMachine.add('W_7', SetWidth(open_width_pickoff), transitions={'finish': 'M_4'})
        
        # 倒车2.5m
        smach.StateMachine.add('M_4', GlobalMoveTask(target_distance=(-2), Throttle=0.3, tar_height=0, updown_mode=0,tar_width=0),
                                transitions={'braking':'BRAKING',
                                            'set_width_height':'FINISH'},
                                remapping={'input_userdata':'last_state'})


        smach.StateMachine.add('FINISH', Finish(),
                               transitions={'succeed':'succeed'})

    return sm