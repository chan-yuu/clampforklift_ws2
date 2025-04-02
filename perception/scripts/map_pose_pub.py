#!/usr/bin/python3
# -*- coding: utf-8 -*-

# 作者： cyun
import rospy
import tf
import geometry_msgs.msg
from math import radians, pi
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from car_interfaces.msg import GpsImuInterface
import math

def convert(north_bearing):
    east_bearing = 90 - north_bearing
    east_bearing = (east_bearing + 360) % 360
    return east_bearing

def convert_to_rad(angle):
    if angle > 180:
        angle -= 360
    return angle * pi / 180
def handle_pose(msg, args):
    listener, pub = args
    
    try:
        pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.header.frame_id = "world"
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose.position.x = msg.x
        pose_stamped.pose.position.y = msg.y
        yaw_ = convert(msg.yaw)
        yaw = convert_to_rad(yaw_)
        pose_stamped.pose.orientation = quaternion_from_euler(0, 0, yaw)

        # 确保变换在特定时间是可用的
        listener.waitForTransform('map', 'world', rospy.Time.now(), rospy.Duration(4.0))
        pose_transformed = listener.transformPose('map', pose_stamped)

        msg_pose = GpsImuInterface()
        msg_pose.x = pose_transformed.pose.position.x
        msg_pose.y = pose_transformed.pose.position.y
        _, _, msg_pose.yaw = euler_from_quaternion([
            pose_transformed.pose.orientation.x,
            pose_transformed.pose.orientation.y,
            pose_transformed.pose.orientation.z,
            pose_transformed.pose.orientation.w
        ])
        pub.publish(msg_pose)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logwarn("TF error: {}".format(e))
        rospy.logwarn("Failed to transform pose: x={}, y={}, yaw={}".format(msg.x, msg.y, msg.yaw))

def listener():
    rospy.init_node('pose_transformer')
    listener = tf.TransformListener()
    pub = rospy.Publisher('map_pose', GpsImuInterface, queue_size=10)
    rospy.Subscriber('gps_imu', GpsImuInterface, handle_pose, (listener, pub))
    rospy.spin()

if __name__ == '__main__':
    listener()
