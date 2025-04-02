#!/usr/bin/env python3

'''
Author: CYUN && cyun@tju.enu.cn
Date: 2024-10-08 02:32:06
LastEditors: CYUN && cyun@tju.enu.cn
LastEditTime: 2024-10-08 14:27:00
FilePath: /undefined/home/cyun/forklift_sim_ws/src/smach_fork/scripts/state_machine_tasks.py
Description: 

Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
'''
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool, Float64, Int8
from car_interfaces.msg import CarOriInterface, PathSpeedCtrlInterface, GpsImuInterface, Decision
# from smach_fork.msg import SetHeightAction, SetHeightGoal, SetWidthAction, SetWidthGoal, SetTarPoseAction, SetTarPoseGoal, SetInitPoseAction, SetInitPoseGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Point, Quaternion
from smach_ros import SimpleActionState
from nav_msgs.msg import Odometry
import signal
import sys
import time
from car_interfaces.srv import GetInitPose, GetTargetPose,GetInitPoseRequest,GetTargetPoseRequest
from car_interfaces.srv import PlanTask, ControlTask, CameraTask, PositionTask,PositionTaskRequest,PositionTaskResponse

# 父类,初始化服务端和客户端
class SerManager:

    def __init__(self):
        self.position_state = 0  # 初始化 position_state 属性
        self.brake_enable = 0
        self.decision_sub_ = rospy.Subscriber('/emergency_brake_from_decision', Decision, self.decision_callback)
        self.position_state_service = None
        self.camera_state = 0
        self.control_state = 0
        self.plan_state = 0
    def decision_callback(self, msg):
        self.brake_enable = msg.brake_enable
        # self.decision_received = 1
    # 服务端接收终点的位置信息：
    def handle_position_state(self, req):
        rospy.loginfo("request: x = %d", req.data)
        self.position_state = req.data
        resp = PositionTaskResponse()
        resp.success = True
        resp.message = "Position state updated successfully."
        return resp
    #TODO 使用服务来进行管理:
    def send_plan_task_service(self, request_value):
        rospy.loginfo("\033[91m Waiting plan_task server... \033[0m ")
        rospy.wait_for_service('plan_task')
        rospy.loginfo("\033[32m"+"Sending plan_task server ready!"+"\033[0m")
        try:
            client = rospy.ServiceProxy("plan_task", PlanTask)
            resp = client(request_value)
            if resp.success:
                rospy.loginfo("plan_task successfully: %s", resp.message)
            else:
                rospy.loginfo("Failed to plan_task: %s", resp.message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def send_control_task_service(self, request_value):
        rospy.loginfo("\033[91m Waiting control_task server... \033[0m ")
        rospy.wait_for_service('control_task')
        rospy.loginfo("\033[32m"+"Sending control_task server ready!"+"\033[0m")
        try:
            client = rospy.ServiceProxy("control_task", ControlTask)
            resp = client(request_value)
            if resp.success:
                rospy.loginfo("control_task successfully: %s", resp.message)
            else:
                rospy.loginfo("Failed to control_task: %s", resp.message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def send_camera_task_service(self, request_value):
        rospy.loginfo("\033[91m Waiting camera_task server... \033[0m ")
        rospy.wait_for_service('camera_task')
        rospy.loginfo("\033[32m"+"Sending camera_task server ready!"+"\033[0m")
        try:
            client = rospy.ServiceProxy("camera_task", CameraTask)
            resp = client(request_value)
            if resp.success:
                rospy.loginfo("camera_task successfully: %s", resp.message)
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
            # 创建请求
            request = GetInitPoseRequest()
            # 填充请求的位姿信息
            request.init_pose.pose.pose.position = Point(point.x, point.y, point.z)
            request.init_pose.pose.pose.orientation = Quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w)
            # 发送请求并等待响应
            response = get_init_pose(request)
            # 打印响应
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
            # return resp
            if response.success:
                rospy.loginfo("Tar pose sent successfully: %s", response.message)
            else:
                rospy.loginfo("Failed to send tar pose: %s", response.message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
def signal_handler(sig, frame):
    print(f"\nReceived {sig} signal, exiting gracefully.")
    sys.exit(0)

class MovingToPickupLocation(smach.State,SerManager):
    def __init__(self):
        smach.State.__init__(self, outcomes=['braking', 'set_width_height'])
        SerManager.__init__(self)
        self.position_state_service = rospy.Service("position_service", PositionTask, self.handle_position_state)
        
    def execute(self, userdata):
        
        rospy.loginfo_once("\033[32m"+'Executing state: 前往取货区任务开始'+"\033[0m")
        rospy.loginfo("\033[32m"+"下发全局任务"+"\033[0m")
        self.task = 1
        self.send_plan_task_service(self.task)
        self.send_control_task_service(self.task)
        # self.send_camera_task_service(self.task)
        rospy.loginfo("\033[32m"+"任务下发成功"+"\033[0m")
        # self.send_camera_task_service(self.task)
        # 接收到全局到达终点的服务端
        rospy.loginfo("\033[32m"+"开始全局任务，下发全局规划点"+"\033[0m")
        # point = Point(1.650403976, 1.7104908, 0.0)
        # quaternion = Quaternion(0.0, 0.0, 0.9999986, 0.00167186579)
        # # success = self.send_init_pose_action(point, quaternion)
        # self.send_init_pose(point, quaternion)
        
        # # 途径点
        # point = Point(27.3194770812, 7.4006414413, 0.0)
        # quaternion = Quaternion(0.0, 0.0,-0.730132375, 0.6833067255)
        # # success = self.send_init_pose_action(point, quaternion)
        # self.send_init_pose(point, quaternion)
        
        # point = Point(26.2201671600, -12.3958, 0.0)
        # quaternion = Quaternion(0.0, 0.0, -0.772302686, 0.63525472)
        # # success = self.send_target_pose_action(point, quaternion)
        # self.send_target_pose(point, quaternion)
        rospy.loginfo("\033[32m"+"点位发布成功"+"\033[0m")
        
        #============================================================
        # 起点 途经点 终点已经发布并且收到了反馈，等待到达全局任务的终点
        # 等待完成全局规划任务，通过是否到达终点的反馈信息来判断
        # 如果中途刹车，那么就会进入Brake，再进来时还是从这里开始
        
        while self.position_state != 1 and not self.brake_enable:
            rospy.loginfo("\033[32m"+"Go to global endpoint"+f"position_state:{self.position_state} expect :1" "\033[0m")
            print("self.position_state", self.position_state)
            time.sleep(1)
        
        # 到达终点->开始夹取
        if self.position_state == 1 and not self.brake_enable: 
            rospy.loginfo_once("\033[32m"+'Position reached, switching to local_planning'+"\033[0m")
            # 释放服务
            self.position_state_service.shutdown("Shutting down the service")
            
            self.task = 0
            self.send_plan_task_service(self.task)
            self.send_control_task_service(self.task)
            return 'set_width_height'
        # 如果有刹车
        elif self.brake_enable:
            rospy.loginfo_once('AEB')
            #NOTE *输入的状态，保证brake出来还是这个状态*
            userdata.last_state = 'MOVING_TO_PICKUP_LOCATION'
            return 'braking'

class Back_And_Forward_For_Bale_Drop(smach.State,SerManager):
    def __init__(self):
        SerManager.__init__(self)
        smach.State.__init__(self, outcomes=['braking', 'set_width_height'])
        
    def execute(self, userdata):
        # 状态机会先初始化全部的类,所以如果position_service 都放在init中,会报错:已经注册服务
        self.position_state_service = rospy.Service("position_service", PositionTask, self.handle_position_state)

        rospy.loginfo_once("\033[32m"+'Executing state: 让出位置让第二个棉包从斜坡上落下任务开始'+"\033[0m")
        rospy.loginfo("\033[32m"+"下发全局任务"+"\033[0m")
        self.task = 1
        self.send_plan_task_service(self.task)
        self.send_control_task_service(self.task)
        # self.send_camera_task_service(self.task)
        rospy.loginfo("\033[32m"+"任务下发成功"+"\033[0m")
        # self.send_camera_task_service(self.task)
        
        # 接收到全局到达终点的服务端
        rospy.loginfo("\033[32m"+"开始全局任务，下发全局规划点"+"\033[0m")
        point = Point(13.0841, 2.7900, 0.0)
        quaternion = Quaternion(0.0, 0.0, 0.9998, 0.0195)
        self.send_init_pose(point, quaternion)

        # 途径点
        point = Point(13.0841, 2.7900, 0.0)
        quaternion = Quaternion(0.0, 0.0, 0.9998, 0.0195)
        self.send_init_pose(point, quaternion)

        # 终点
        point = Point(27.0779, -6.6748, 0.0)
        quaternion = Quaternion(0.0, 0.0, -0.6937, 0.7202)
        self.send_target_pose(point, quaternion)

        rospy.loginfo("\033[32m"+"点位发布成功"+"\033[0m")
        #============================================================
        # 起点 途经点 终点已经发布并且收到了反馈，等待到达全局任务的终点
        # 等待完成全局规划任务，通过是否到达终点的反馈信息来判断
        # 如果中途刹车，那么就会进入Brake，再进来时还是从这里开始
        while self.position_state != 1 and not self.brake_enable:
            rospy.loginfo("\033[32m"+"Go to global endpoint"+"\033[0m" +f"position_state:{self.position_state} expect :1" )
            time.sleep(1)
        
        # 到达终点->开始夹取
        if self.position_state == 1 and not self.brake_enable: 
            rospy.loginfo_once("\033[32m"+'Position reached, switching to local_planning'+"\033[0m")
            # 释放服务
            self.position_state_service.shutdown("Shutting down the service")
            return 'set_width_height'
        # 如果有刹车
        elif self.brake_enable:
            rospy.loginfo_once('AEB')
            #NOTE *输入的状态，保证brake出来还是这个状态*
            userdata.last_state = 'MOVING_TO_PICKUP_LOCATION'
            return 'braking'

class Back_2_Work_Point(smach.State,SerManager):
    def __init__(self):
        SerManager.__init__(self)
        smach.State.__init__(self, outcomes=['braking', 'start'])
        
    def execute(self, userdata):
        # 状态机会先初始化全部的类,所以如果position_service 都放在init中,会报错:已经注册服务
        self.position_state_service = rospy.Service("position_service", PositionTask, self.handle_position_state)

        rospy.loginfo_once("\033[32m"+'Executing state: 让出位置让第二个棉包从斜坡上落下任务开始'+"\033[0m")
        rospy.loginfo("\033[32m"+"下发全局任务"+"\033[0m")
        self.task = 1
        self.send_plan_task_service(self.task)
        self.send_control_task_service(self.task)
        # self.send_camera_task_service(self.task)
        rospy.loginfo("\033[32m"+"任务下发成功"+"\033[0m")
        # self.send_camera_task_service(self.task)
        # 接收到全局到达终点的服务端
        rospy.loginfo("\033[32m"+"开始全局任务，下发全局规划点"+"\033[0m")
        point = Point(13.0841, 2.7900, 0.0)
        quaternion = Quaternion(0.0, 0.0, 0.9998, 0.0195)
        self.send_init_pose(point, quaternion)
        
        # 途径点
        point = Point(13.0841, 2.7900, 0.0)
        quaternion = Quaternion(0.0, 0.0, 0.9998, 0.0195)
        self.send_init_pose(point, quaternion)
 
        # 终点        
        point = Point(27.0779, -6.6748, 0.0)
        quaternion = Quaternion(0.0, 0.0, -0.6937, 0.7202)
        self.send_target_pose(point, quaternion)
        rospy.loginfo("\033[32m"+"点位发布成功"+"\033[0m")
        while self.position_state != 1 and not self.brake_enable:
            rospy.loginfo("\033[32m"+"Go to global endpoint"+"\033[0m" +f"position_state:{self.position_state} expect :1" )
            time.sleep(1)
        
        
        if self.position_state == 1 and not self.brake_enable: 
            rospy.loginfo_once("\033[32m"+'Position reached, switching to local_planning'+"\033[0m")
            # 释放服务
            self.position_state_service.shutdown("Shutting down the service")
            return 'start'
        # 如果有刹车
        elif self.brake_enable:
            rospy.loginfo_once('AEB')
            #NOTE *输入的状态，保证brake出来还是这个状态*
            userdata.last_state = 'MOVING_TO_PICKUP_LOCATION'
            return 'braking'  

class Braking(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['moving_to_pickup_location', 'moving_to_dropoff_location'])
        
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
        elif not self.brake_enable and userdata.last_state == 'MOVING_TO_DROPOFF_LOCATION':
            rospy.loginfo("\033[32m"+'Braking complete, switching to MOVING_TO_DROPOFF_LOCATION'+"\033[0m")
            return 'moving_to_dropoff_location'


# 设置宽度和高度状态 需要分开，因为取货和放货的顺序不一样
class SetWidth(smach.State):
    def __init__(self, tar_width):
        smach.State.__init__(self, outcomes=['finish'])
        self.tar_width = tar_width
        self.set_tar_width_pub = rospy.Publisher('/clamp_cmd', Float64, queue_size=1)
        rospy.Subscriber('/clamp_state', Float64, self.clamp_state_callback)
        
        self.cur_width = 0

    def clamp_state_callback(self, msg):
        self.cur_width = msg.data

    def execute(self, userdata):
        rospy.loginfo_once("\033[32m"+'Executing state: SET_WIDTH'+"\033[0m")
        while not (abs(self.cur_width - self.tar_width) < 12):
            rospy.loginfo(f"\033[33m cur_width:{self.cur_width} tar_width:{self.tar_width} \033[0m")
            self.set_tar_width_pub.publish(self.tar_width)
            rospy.Rate(1).sleep()
        rospy.loginfo_once("\033[32m"+"Width set successfully"+"\033[0m")
        return 'finish'

# 设置高度状态
class SetHeight(smach.State):
    def __init__(self, tar_height):
        smach.State.__init__(self, outcomes=['finish'])
        self.tar_height = tar_height
        self.set_tar_height_pub = rospy.Publisher('/updown_cmd', Float64, queue_size=1)
        rospy.Subscriber('/updown_state', Float64, self.updown_state_callback)
        
        self.cur_height = 0

    def updown_state_callback(self, msg):
        self.cur_height = msg.data

    def execute(self, userdata):
        rospy.loginfo_once("\033[32m"+'Executing state: SET_HEIGHT'+"\033[0m")
        while not (abs(self.cur_height - self.tar_height) < 10):
            rospy.loginfo(f" \033[90m cur_height:{self.cur_height} tar_height:{self.tar_height} \033[0m")
            self.set_tar_height_pub.publish(self.tar_height)
            rospy.Rate(1).sleep()
        rospy.loginfo_once("\033[32m"+"Height set successfully"+"\033[0m")
        return 'finish'


# 移动到放置位置状态  也是一个全局任务
class MovingToDropoffLocation(smach.State,SerManager):
    def __init__(self):
        SerManager.__init__(self)
        smach.State.__init__(self, outcomes=['braking', 'set_width_height'])

    def execute(self, userdata):
        # 状态机会先初始化全部的类,所以如果position_service 都放在init中,会报错:已经注册服务
        self.position_state_service = rospy.Service("position_service", PositionTask, self.handle_position_state)

        rospy.loginfo_once("\033[32m"+'Executing state: 让出位置让第二个棉包从斜坡上落下任务开始'+"\033[0m")
        rospy.loginfo("\033[32m"+"下发全局任务"+"\033[0m")
        self.task = 1
        self.send_plan_task_service(self.task)
        self.send_control_task_service(self.task)
        # self.send_camera_task_service(self.task)
        rospy.loginfo("\033[32m"+"任务下发成功"+"\033[0m")
        # self.send_camera_task_service(self.task)
        # 接收到全局到达终点的服务端
        rospy.loginfo("\033[32m"+"开始全局任务，下发全局规划点"+"\033[0m")
        point = Point(13.0841, 2.7900, 0.0)
        quaternion = Quaternion(0.0, 0.0, 0.9998, 0.0195)
        self.send_init_pose(point, quaternion)
        
        # 途径点
        point = Point(13.0841, 2.7900, 0.0)
        quaternion = Quaternion(0.0, 0.0, 0.9998, 0.0195)
        self.send_init_pose(point, quaternion)
 
        # 终点        
        point = Point(27.0779, -6.6748, 0.0)
        quaternion = Quaternion(0.0, 0.0, -0.6937, 0.7202)
        self.send_target_pose(point, quaternion)
        rospy.loginfo("\033[32m"+"点位发布成功"+"\033[0m")
        #============================================================
        # 起点 途经点 终点已经发布并且收到了反馈，等待到达全局任务的终点
        # 等待完成全局规划任务，通过是否到达终点的反馈信息来判断
        # 如果中途刹车，那么就会进入Brake，再进来时还是从这里开始
        while self.position_state != 1 and not self.brake_enable:
            rospy.loginfo("\033[32m"+"Go to global endpoint"+"\033[0m" +f"position_state:{self.position_state} expect :1" )
            time.sleep(1)
        
        # 到达终点->开始夹取
        if self.position_state == 1 and not self.brake_enable: 
            rospy.loginfo_once("\033[32m"+'Position reached, switching to local_planning'+"\033[0m")
            # 释放服务
            self.position_state_service.shutdown("Shutting down the service")
            return 'set_width_height'
        # 如果有刹车
        elif self.brake_enable:
            rospy.loginfo_once('AEB')
            #NOTE *输入的状态，保证brake出来还是这个状态*
            userdata.last_state = 'MOVING_TO_PICKUP_LOCATION'
            return 'braking'

      

# 开始  
class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start'])

    def execute(self, userdata):
        rospy.loginfo("\033[32m"+'Executing state: Start'+"\033[0m")
        return 'start'

# 完成状态
class Finish(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeed'])

    def execute(self, userdata):
        rospy.loginfo("\033[32m"+'Executing state: FINISH'+"\033[0m")
        return 'succeed'

def create_state_machine():
    sm = smach.StateMachine(outcomes=['succeed', 'aborted', 'preempted'])

    with sm:
        smach.StateMachine.add('START', Start(), 
                               transitions={'start':'MOVING_TO_PICKUP_LOCATION'}) #正常发布全局任务

        smach.StateMachine.add('MOVING_TO_PICKUP_LOCATION', MovingToPickupLocation(), 
                            transitions={'braking':'BRAKING', 
                                        'set_width_height':'SET_WIDTH'},
                            remapping={'input_userdata':'last_state'})
        
        smach.StateMachine.add('BRAKING', Braking(), 
                               transitions={'moving_to_pickup_location':'MOVING_TO_PICKUP_LOCATION', 
                                            'moving_to_dropoff_location':'MOVING_TO_DROPOFF_LOCATION',
                                            })
        smach.StateMachine.add('SET_WIDTH', SetWidth(159), transitions={'finish': 'SET_HEIGHT'})

        smach.StateMachine.add('SET_HEIGHT', SetHeight(359), transitions={'finish': 'Back_And_Forward_For_Bale_Drop'})
        
        smach.StateMachine.add('Back_And_Forward_For_Bale_Drop',Back_And_Forward_For_Bale_Drop(),transitions={'braking':'BRAKING', 'set_width_height':'SET_WIDTH1_1'})
        
        smach.StateMachine.add('SET_WIDTH1_1', SetWidth(159), transitions={'finish': 'SET_HEIGHT1_1'})

        smach.StateMachine.add('SET_HEIGHT1_1', SetHeight(359), transitions={'finish': 'SET_WIDTH1_2'})
        
        smach.StateMachine.add('SET_WIDTH1_2', SetWidth(159), transitions={'finish': 'MOVING_TO_DROPOFF_LOCATION'})
        
        smach.StateMachine.add('MOVING_TO_DROPOFF_LOCATION', MovingToDropoffLocation(), 
                            transitions={'braking':'BRAKING', 
                                        'set_width_height':'SET_HEIGHT2'},
                            remapping={'input_userdata':'last_state'})
        
        smach.StateMachine.add('SET_HEIGHT2', SetHeight(355), transitions={'finish': 'SET_WIDTH2'})

        smach.StateMachine.add('SET_WIDTH2', SetWidth(159), transitions={'finish': 'FINISH'})

        smach.StateMachine.add('FINISH', Finish(),
                               transitions={'succeed':'Back_2_Work_Point'})
        
        smach.StateMachine.add('Back_2_Work_Point', Back_2_Work_Point(),
                               transitions={'braking':'BRAKING',
                                            'start':'START'})

    return sm