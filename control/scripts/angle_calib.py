#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
from std_msgs.msg import Int32
import tf.transformations as tf
from std_msgs.msg import Float64, UInt8, Bool
import math
import time


class VehicleController:
    def __init__(self):
        self.current_odom = None
        self.initial_heading = None
        self.steering_state = None
        self.target_heading = None
        self.is_heading_reached = False

        # =================参数配置=================
        self.send_wheel_angle = 1.2  # deg
        self.send_gear = 3
        self.send_throttle = 0.3  # m/s
        angle_rotation = -80
        self.angle_rotation = angle_rotation * math.pi / 180  # 左负右正
        # =================参数配置=================

        self.turning_threshold = 0.1  # 缩小航向偏差阈值，提高精度
        self.head_error = 0

        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/steering_state', Float64, self.steering_state_callback)

        self.throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=10)
        self.gear_pub = rospy.Publisher('/gear_cmd', UInt8, queue_size=10)
        self.stop_pub = rospy.Publisher('/brake_cmd', Float64, queue_size=10)
        self.steering_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=10)

    def turn_angle(self, current_heading, angle):
        new_heading = current_heading + angle
        if new_heading > math.pi:
            new_heading -= 2 * math.pi
        elif new_heading < -math.pi:
            new_heading += 2 * math.pi
        return new_heading

    def odom_callback(self, data):
        self.current_odom = data
        current_heading = self.get_heading_from_quaternion(data.pose.pose.orientation)
        # print('odom received!', current_heading)
        if self.initial_heading is None:
            self.initial_heading = current_heading

    def steering_state_callback(self, data):
        self.steering_state = data
        # print('steering state received', data)

    def get_heading_from_quaternion(self, quaternion):
        _, _, yaw = tf.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        if yaw > math.pi:
            yaw -= 2 * math.pi
        elif yaw < -math.pi:
            yaw += 2 * math.pi
        return yaw

    def check_heading_reached(self):
        current_heading = self.get_heading_from_quaternion(self.current_odom.pose.pose.orientation)
        # print("current_heading", current_heading)
        # print("self.initial_heading", self.initial_heading)
        head_error = (current_heading - self.initial_heading+math.pi)%(2*math.pi)-math.pi

        return abs(head_error)

    def run(self):
        rate = rospy.Rate(100)  # 提高频率到 50Hz
        last_print_time = time.time()
        flag=False

        if (self.angle_rotation>0):
            self.send_wheel_angle = -self.send_wheel_angle
        else:
            self.send_wheel_angle = self.send_wheel_angle
        
        while not rospy.is_shutdown():
            
            if self.current_odom is None or self.steering_state is None:
                rospy.loginfo_once("wait odom & steering")
                continue

            # 先发布转角，然后订阅到 steering_state，两者偏差小于阈值后再继续
            # print(self.send_wheel_angle, self.steering_state.data)
            steering_diff = abs(abs(self.send_wheel_angle) - abs(self.steering_state.data))
            # print("steering_diff******", steering_diff)
            if steering_diff > self.turning_threshold and flag==False:
                # print("aaaaaaaaaaaaaaa")
                self.gear_pub.publish(self.send_gear)
                self.throttle_pub.publish(0)
                self.steering_pub.publish(self.send_wheel_angle)
                self.stop_pub.publish(0)
            else:
                flag=True
            if flag:
                self.head_error = self.check_heading_reached()
                if self.head_error >= abs(self.angle_rotation):
                    self.steering_pub.publish(0)
                    self.throttle_pub.publish(0)
                    self.gear_pub.publish(2)
                    self.stop_pub.publish(0)
                    self.is_heading_reached = True
                    print("last head error: ", self.head_error)
                    rospy.loginfo("goal reached")
                    break
                else:
                    # 转向完成后，开始行驶并持续检查航向是否达到目标
                    # print("bbbbbbbbbbbbbbbbbbbbbbbb")
                    self.gear_pub.publish(self.send_gear)
                    self.throttle_pub.publish(self.send_throttle)
                    self.steering_pub.publish(self.send_wheel_angle)
                    self.stop_pub.publish(0)
            # 1s打印一次：
            current_time = time.time()
            if current_time - last_print_time >= 1:
                rospy.loginfo(f"Steering state: {self.steering_state.data}")
                rospy.loginfo(f"abs(self.angle_rotation:{abs(self.angle_rotation)}")
                rospy.loginfo(f"Head error: {self.head_error}")
                last_print_time = current_time

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('vehicle_controller_node')
    controller = VehicleController()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        pass