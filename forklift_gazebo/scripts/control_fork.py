#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from car_interfaces.msg import PathSpeedCtrlInterface  # 假设你定义了一个叫做 Control 的自定义消息类型，包含 velocity 和 wheel_angle
import math
from std_msgs.msg import UInt8,Float64

class CmdVel2Gazebo:
    def __init__(self):
        rospy.init_node('cmdvel2gazebo', anonymous=True)
        
        # 订阅 /control 话题
        # rospy.Subscriber('/path_speed_tracking_data', PathSpeedCtrlInterface, self.callback, queue_size=1)
        # new interface:
        # 订阅转角
        rospy.Subscriber('/steering_cmd', Float64, self.steering_callback, queue_size=1)
        # 订阅速度
        rospy.Subscriber('/throttle_cmd', Float64, self.throttle_callback, queue_size=1)
        # 订阅档位
        rospy.Subscriber('/gear_cmd', UInt8, self.gear_callback, queue_size=1)
        # 订阅刹车
        rospy.Subscriber('/brake_cmd', Float64, self.brake_callback, queue_size=1)


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
        self.gear = 0
        self.throttle = 0
        self.brake = 0
        self.steering = 0
        # loop
        rate = rospy.Rate(10) # run at 10Hz
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()
        

    # def callback(self, data):
    #     # 从 /control 话题获取速度和转角
    #     print(data.Target_velocity, "/n", data.Target_steering_angle)
    #     # v = v / r
    #     self.x = data.Target_velocity / 0.3
    #     self.z = data.Target_steering_angle * math.pi / 180
    #     self.lastMsg = rospy.Time.now()

    def throttle_callback(self, msg):
        self.throttle = msg.data

    def steering_callback(self, msg):
        self.steering = msg.data

    def gear_callback(self, msg):
        self.gear = msg.data

    def brake_callback(self, msg):
        self.brake = msg.data

    def publish(self):
        self.lastMsg = rospy.Time.now()
        # 处理所有的订阅内容进行整合：
        if (self.gear==3):
            self.x = self.throttle / 0.3
            self.z = self.steering * math.pi / 180

        elif (self.gear==1):
            self.x = -self.throttle / 0.3
            self.z = -self.steering * math.pi / 180
        else:
            return

        if (self.brake>0):
            self.x = 0#self.throttle / 0.3
            self.z = 0#self.steering * math.pi / 180

        delta_last_msg_time = rospy.Time.now() - self.lastMsg
        msgs_too_old = delta_last_msg_time > self.timeout
        if msgs_too_old:
            self.x = 0
            msgFront = Float64()
            msgFront.data = 0#self.x
            self.pub_frontL.publish(msgFront)
            self.pub_frontR.publish(msgFront)
            msgSteer = Float64()
            msgSteer.data = 0
            self.pub_steerL.publish(msgSteer)
            self.pub_steerR.publish(msgSteer)
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

            msgFrontR = Float64()
            msgFrontR.data = self.x * rL_front / r
            msgFrontL = Float64()
            msgFrontL.data = self.x * rR_front / r

            self.pub_frontL.publish(msgFrontL)
            self.pub_frontR.publish(msgFrontR)

            msgSteerL = Float64()
            msgSteerR = Float64()
            msgSteerL.data = math.atan2(L, rR_rear) * math.copysign(1, self.z)
            self.pub_steerL.publish(msgSteerL)
    
            msgSteerR.data = math.atan2(L, rL_rear) * math.copysign(1, self.z)
            self.pub_steerR.publish(msgSteerR)

        else:
            msgFront = Float64()
            msgFront.data = self.x
            self.pub_frontL.publish(msgFront)
            self.pub_frontR.publish(msgFront)

            msgSteer = Float64()
            msgSteer.data = self.z

            self.pub_steerL.publish(msgSteer)
            self.pub_steerR.publish(msgSteer)


if __name__ == '__main__':
    try:
        CmdVel2Gazebo()
    except rospy.ROSInterruptException:
        pass
