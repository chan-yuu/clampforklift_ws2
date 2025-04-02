#!/usr/bin/env python
'''
Author: CYUN && cyun@tju.enu.cn
Date: 2024-10-07 09:45:19
LastEditors: CYUN && cyun@tju.enu.cn
LastEditTime: 2025-03-13 00:15:51
FilePath: /src/home/cyun/forklift_sim_ws3/src/clamp_fork/scripts/control_fork.py
Description: 1.将控制信息进行了分解，可以对转角，速度，档位，刹车进行分别控制
                2.根据运动学模型计算两轮的转速实现控速和转向

Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
'''

import rospy
from std_msgs.msg import Float64
from car_interfaces.msg import PathSpeedCtrlInterface
import math
from std_msgs.msg import UInt8,Float64,Int8
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist



class CmdVel2Gazebo:
    def __init__(self):
        rospy.init_node('cmdvel2gazebo', anonymous=True)
        
        # 订阅 /cmd_vel 话题
        rospy.Subscriber('/cmd_vel', Twist, self.callback, queue_size=1)
        # new interface:
        # 订阅转角
        rospy.Subscriber('/steering_cmd', Float64, self.steering_callback, queue_size=1)
        # 订阅速度
        rospy.Subscriber('/throttle_cmd', Float64, self.throttle_callback, queue_size=1)
        # 订阅档位
        rospy.Subscriber('/gear_cmd', UInt8, self.gear_callback, queue_size=1)
        # 订阅刹车
        rospy.Subscriber('/brake_cmd', Float64, self.brake_callback, queue_size=1)

        rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)
        self.twist_pub = rospy.Publisher('twist', TwistStamped, queue_size=1)
        self.gear_pub = rospy.Publisher('gear_state', Int8, queue_size=1)
        self.steering_state_pub_ = rospy.Publisher('steering_state', Float64, queue_size=1)

        self.pub_steerL = rospy.Publisher('/front_left_steering_position_controller/command', Float64, queue_size=1)
        self.pub_steerR = rospy.Publisher('/front_right_steering_position_controller/command', Float64, queue_size=1)
        self.pub_frontL = rospy.Publisher('/rear_left_velocity_controller/command', Float64, queue_size=1)
        self.pub_frontR = rospy.Publisher('/rear_right_velocity_controller/command', Float64, queue_size=1)


        # initial velocity and tire angle are 0
        self.x = 0
        self.z = 0

        # car Wheelbase (in m)
        self.L = 1.868

        # car Tread
        self.T_front = 1.284
        self.T_rear = 1.284

        # how many seconds delay for the dead man's switch
        self.timeout=rospy.Duration.from_sec(0.2)
        self.lastMsg=rospy.Time.now()
        self.lastMsg_=rospy.Time.now()

        self.gear = 0
        self.throttle = 0
        self.brake = 0
        self.steering = 0

        self.gear_ = False
        self.throttle_ = False
        self.brake_ = False
        self.steering_ = False

        self.axle_z = 0
        self.axle_x = 0
        
        # loop
        rate = rospy.Rate(50) # run at 10Hz
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()
        
    def callback(self, data):
        # print("111111111111111")
        self.maxsteer = 1.1
        # w = v / r
        self.axle_x = data.linear.x / 0.3
        # constrain the ideal steering angle such that the ackermann steering is maxed out
        # 方向，取负值即可
        self.axle_z = -max(-self.maxsteer,min(self.maxsteer,data.angular.z))
        # self.axle_z = -self.z
        print(self.axle_x, "/n", self.axle_z)
        self.lastMsg_ = rospy.Time.now()
        self.lastMsg = rospy.Time.now()
        
    # def callback(self, data):
    #     # 从 /control 话题获取速度和转角
    #     print(data.Target_velocity, "/n", data.Target_steering_angle)
    #     # v = v / r
    #     self.x = data.Target_velocity / 0.3
    #     self.z = data.Target_steering_angle * math.pi / 180
    #     self.lastMsg = rospy.Time.now()

    def odom_callback(self, msg):
        # 初始化 TwistStamped 消息
        twist_msg = TwistStamped()
        twist_msg.header.stamp = rospy.Time.now()
        twist_msg.header.frame_id = "base_footprint"

        twist_msg.twist.linear.x = msg.twist.twist.linear.x
        self.twist_pub.publish(twist_msg)
        
    def throttle_callback(self, msg):
        self.throttle = msg.data
        
        self.lastMsg_ = rospy.Time.now()
        self.lastMsg = rospy.Time.now()
        
        # self.throttle_ = True
    def steering_callback(self, msg):
        self.steering = msg.data
        self.maxsteer = 1.1
        self.steering = max(-self.maxsteer,min(self.maxsteer,self.steering)) * 180 /math.pi
        
        # self.steering_ = True
    def gear_callback(self, msg):
        self.gear = msg.data
        gear_msg = Int8()
        gear_msg.data = self.gear
        self.gear_pub.publish(gear_msg)

    def brake_callback(self, msg):
        self.brake = msg.data
        # self.brake_ = True
    def publish(self):
        # 处理所有的订阅内容进行整合：
        # print("self.gear: ",self.gear)
        # print("self.throttle: ", self.throttle)
        # print("self.steering: ",self.steering)
        # print("self.brake: ",self.brake)
        # print("self.axle_x: ", self.axle_x, "self.axle_z: ", self.axle_z)
        msgFront = Float64()
        msgSteer = Float64()
        
        msgFrontR = Float64()
        msgFrontL = Float64()
        msgSteerL = Float64()
        msgSteerR = Float64()
        
        self.steering_state_pub_.publish(self.steering* math.pi / 180)

        if (self.gear==3):
            self.x = self.throttle / 0.3
            self.z = self.steering * math.pi / 180
        elif (self.gear==1):
            self.x = -self.throttle / 0.3
            # 不能放到can中，会导致倒车反转
            self.z = self.steering * math.pi / 180
        elif(self.gear==0):
            self.x = self.axle_x / 0.3
            self.z = self.axle_z
        else:
            self.x = 0
            msgFront.data = 0#self.x
            self.pub_frontL.publish(msgFront)
            self.pub_frontR.publish(msgFront)
            msgSteer.data = 0
            self.pub_steerL.publish(msgSteer)
            self.pub_steerR.publish(msgSteer)
            return

        if (self.brake>0):
            self.x = 0#self.throttle / 0.3
            self.z = 0#self.steering * math.pi / 180
            self.brake = 0

        delta_last_msg_time = rospy.Time.now() - self.lastMsg
        msgs_too_old = delta_last_msg_time > self.timeout
        msgs_too_old_ = rospy.Time.now() - self.lastMsg_ > self.timeout
        if msgs_too_old or msgs_too_old_:
            self.x = 0
            msgFront.data = 0#self.x
            self.pub_frontL.publish(msgFront)
            self.pub_frontR.publish(msgFront)
            msgSteer.data = 0
            self.pub_steerL.publish(msgSteer)
            self.pub_steerR.publish(msgSteer)
            print("return0000")
            return

        if self.z != 0:
            T_front = self.T_front
            T_rear = self.T_rear
            L = self.L
            r = L/math.fabs(math.tan(self.z))

            rL_rear = r-(math.copysign(1,self.z)*(T_rear/2.0))
            rR_rear = r+(math.copysign(1,self.z)*(T_rear/2.0))
            rL_front = r-(math.copysign(1,self.z)*(T_front/2.0))
            rR_front = r+(math.copysign(1,self.z)*(T_front/2.0))

            msgFrontR.data = self.x * rL_front / r
            msgFrontL.data = self.x * rR_front / r

            self.pub_frontL.publish(msgFrontL)
            self.pub_frontR.publish(msgFrontR)


            msgSteerL.data = math.atan2(L, rR_rear) * math.copysign(1, self.z)
            self.pub_steerL.publish(msgSteerL)
    
            msgSteerR.data = math.atan2(L, rL_rear) * math.copysign(1, self.z)
            self.pub_steerR.publish(msgSteerR)
            print('--------------')
            print("msgSteerR: ", msgSteerR.data)
            print("msgSteerL: ", msgSteerL.data)
            print("msgFrontL: ", msgFrontL.data)
            print("msgFrontR: ", msgFrontR.data)

        else:
            msgFront.data = self.x
            self.pub_frontL.publish(msgFront)
            self.pub_frontR.publish(msgFront)
            
            msgSteer.data = self.z
            self.pub_steerL.publish(msgSteer)
            self.pub_steerR.publish(msgSteer)
            print('****************')
            print("steering: ", 0)
            print("msgFront: ", msgFront.data)
            print("msgSteer: ", msgSteer.data)


if __name__ == '__main__':
    try:
        CmdVel2Gazebo()
    except rospy.ROSInterruptException:
        pass
