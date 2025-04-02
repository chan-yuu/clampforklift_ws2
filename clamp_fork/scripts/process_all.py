#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: CYUN && cyun@tju.enu.cn
Date: 2024-10-07 10:00:24
LastEditors: CYUN && cyun@tju.enu.cn
LastEditTime: 2024-10-07 12:01:37
FilePath: /undefined/home/cyun/forklift_sim_ws/src/clamp_fork/scripts/process_all.py
Description: 所有功能的合并程序

Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
'''

import rospy
import tf
import tf.transformations as tft
import geometry_msgs.msg
from car_interfaces.msg import GpsImuInterface, PathSpeedCtrlInterface
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion
from std_msgs.msg import Float64, UInt8
import math

class CombinedNode:
    def __init__(self):
        rospy.init_node('combined_node', anonymous=True)

        # Odometry Publisher
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        rospy.Subscriber('map_to_base', GpsImuInterface, self.odom_callback)

        # TF Publisher
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.initial_tf_published = False
        self.initial_transform = None
        self.map_pose_sub = rospy.Subscriber('/map_pose', GpsImuInterface, self.map_pose_callback)

        # CmdVel2Gazebo
        self.pub_steerL = rospy.Publisher('/front_left_steering_position_controller/command', Float64, queue_size=1)
        self.pub_steerR = rospy.Publisher('/front_right_steering_position_controller/command', Float64, queue_size=1)
        self.pub_frontL = rospy.Publisher('/rear_left_velocity_controller/command', Float64, queue_size=1)
        self.pub_frontR = rospy.Publisher('/rear_right_velocity_controller/command', Float64, queue_size=1)

        rospy.Subscriber('/steering_cmd', Float64, self.steering_callback, queue_size=1)
        rospy.Subscriber('/throttle_cmd', Float64, self.throttle_callback, queue_size=1)
        rospy.Subscriber('/gear_cmd', UInt8, self.gear_callback, queue_size=1)
        rospy.Subscriber('/brake_cmd', Float64, self.brake_callback, queue_size=1)

        # MapToBasePublisher
        self.tf_listener = tf.TransformListener()
        rospy.sleep(1.0)  # Wait for TF buffer to be ready
        self.map_to_base_pub = rospy.Publisher('/map_to_base', GpsImuInterface, queue_size=10)

        # Odom to base_footprint transform
        rospy.Subscriber('/odom_p3d', Odometry, self.odom_to_base_callback)

        self.x = 0
        self.z = 0
        self.L = 1.868
        self.T_front = 1.284
        self.T_rear = 1.284
        self.timeout = rospy.Duration.from_sec(0.2)
        self.lastMsg = rospy.Time.now()
        self.gear = 0
        self.throttle = 0
        self.brake = 0
        self.steering = 0

        self.rate = rospy.Rate(100)  # 100Hz

    def odom_callback(self, msg):
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "map"
        odom_msg.pose.pose.position = Point(msg.x, msg.y, 0)
        odom_msg.pose.pose.orientation = Quaternion(0, 0, msg.yaw * math.pi / 180, 1)
        odom_msg.child_frame_id = "base_footprint"
        self.odom_pub.publish(odom_msg)

    def map_pose_callback(self, msg):
        if not self.initial_tf_published:
            self.initial_transform = geometry_msgs.msg.TransformStamped()
            self.initial_transform.header.stamp = rospy.Time.now()
            self.initial_transform.header.frame_id = "map"
            self.initial_transform.child_frame_id = "odom"
            self.initial_transform.transform.translation.x = msg.x
            self.initial_transform.transform.translation.y = msg.y
            self.initial_transform.transform.translation.z = msg.z
            quat = tft.quaternion_from_euler(math.radians(msg.roll), math.radians(msg.pitch), math.radians(msg.yaw))
            self.initial_transform.transform.rotation.x = quat[0]
            self.initial_transform.transform.rotation.y = quat[1]
            self.initial_transform.transform.rotation.z = quat[2]
            self.initial_transform.transform.rotation.w = quat[3]
            self.initial_tf_published = True
            rospy.loginfo("Initial map to odom transform has been set.")

    def throttle_callback(self, msg):
        self.throttle = msg.data
        self.lastMsg = rospy.Time.now()

    def steering_callback(self, msg):
        self.steering = msg.data * 180 / math.pi

    def gear_callback(self, msg):
        self.gear = msg.data

    def brake_callback(self, msg):
        self.brake = msg.data

    def odom_to_base_callback(self, msg):
        br = tf.TransformBroadcaster()
        br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                         (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                         rospy.Time.now(),
                         "base_footprint",
                         "odom")

    def publish_cmdvel(self):
        if (self.gear == 3):
            self.x = self.throttle / 0.3
            self.z = self.steering * math.pi / 180
        elif (self.gear == 1):
            self.x = -self.throttle / 0.3
            self.z = -self.steering * math.pi / 180
        else:
            return

        if (self.brake > 0):
            self.x = 0
            self.z = 0

        delta_last_msg_time = rospy.Time.now() - self.lastMsg
        msgs_too_old = delta_last_msg_time > self.timeout
        if msgs_too_old:
            self.x = 0
            msgFront = Float64()
            msgFront.data = 0
            self.pub_frontL.publish(msgFront)
            self.pub_frontR.publish(msgFront)
            msgSteer = Float64()
            msgSteer.data = 0
            self.pub_steerL.publish(msgSteer)
            self.pub_steerR.publish(msgSteer)
            return

        if self.z != 0:
            r = self.L / math.fabs(math.tan(self.z))
            rL_rear = r - (math.copysign(1, self.z) * (self.T_rear / 2.0))
            rR_rear = r + (math.copysign(1, self.z) * (self.T_rear / 2.0))
            rL_front = r - (math.copysign(1, self.z) * (self.T_front / 2.0))
            rR_front = r + (math.copysign(1, self.z) * (self.T_front / 2.0))

            msgFrontR = Float64()
            msgFrontR.data = self.x * rL_front / r
            msgFrontL = Float64()
            msgFrontL.data = self.x * rR_front / r

            self.pub_frontL.publish(msgFrontL)
            self.pub_frontR.publish(msgFrontR)

            msgSteerL = Float64()
            msgSteerR = Float64()
            msgSteerL.data = math.atan2(self.L, rR_rear) * math.copysign(1, self.z)
            self.pub_steerL.publish(msgSteerL)
    
            msgSteerR.data = math.atan2(self.L, rL_rear) * math.copysign(1, self.z)
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

    def publish_map_to_base(self):
        try:
            # Get transform from map to odom
            (trans_map_to_odom, rot_map_to_odom) = self.tf_listener.lookupTransform('/map', '/odom', rospy.Time(0))
            # Get transform from odom to base_footprint
            (trans_odom_to_base, rot_odom_to_base) = self.tf_listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))
            
            # Convert translations and rotations to matrix form
            mat_map_to_odom = tf.transformations.concatenate_matrices(
                tf.transformations.translation_matrix(trans_map_to_odom),
                tf.transformations.quaternion_matrix(rot_map_to_odom)
            )
            
            mat_odom_to_base = tf.transformations.concatenate_matrices(
                tf.transformations.translation_matrix(trans_odom_to_base),
                tf.transformations.quaternion_matrix(rot_odom_to_base)
            )

            # Combine matrices to get final transformation
            mat_final = tf.transformations.concatenate_matrices(mat_map_to_odom, mat_odom_to_base)

            # Extract translation and rotation from final matrix
            trans_final = tf.transformations.translation_from_matrix(mat_final)
            rot_final = tf.transformations.quaternion_from_matrix(mat_final)
            
            # Convert quaternion to Euler angles
            euler = tf.transformations.euler_from_quaternion(rot_final)
            
            # Create and fill custom message
            msg = GpsImuInterface()
            msg.x = trans_final[0]
            msg.y = trans_final[1]
            msg.z = trans_final[2]
            msg.roll = euler[0]
            msg.pitch = euler[1]
            msg.yaw = euler[2] * 180 / math.pi

            # Publish message
            self.map_to_base_pub.publish(msg)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("TF lookup failed: {}".format(e))

    def run(self):
        while not rospy.is_shutdown():
            if self.initial_tf_published:
                self.initial_transform.header.stamp = rospy.Time.now()
                self.tf_broadcaster.sendTransform(
                    (self.initial_transform.transform.translation.x,
                     self.initial_transform.transform.translation.y,
                     self.initial_transform.transform.translation.z),
                    (self.initial_transform.transform.rotation.x,
                     self.initial_transform.transform.rotation.y,
                     self.initial_transform.transform.rotation.z,
                     self.initial_transform.transform.rotation.w),
                    rospy.Time.now(),
                    self.initial_transform.child_frame_id,
                    self.initial_transform.header.frame_id
                )
            
            self.publish_cmdvel()
            self.publish_map_to_base()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = CombinedNode()
        node.run()
    except rospy.ROSInterruptException:
        pass