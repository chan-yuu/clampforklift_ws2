#!/usr/bin/python3
# -*- coding: utf-8 -*-


import rospy
import numpy as np
import tf_conversions
import geometry_msgs.msg
from math import radians, pi
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from car_interfaces.msg import GpsImuInterface

def convert(north_bearing):
    '''
    Convert north-based bearing to east-based bearing.
    '''
    east_bearing = 90 - north_bearing
    east_bearing = (east_bearing + 360) % 360
    return east_bearing

def convert_to_rad(angle):
    """
    Convert degree angle to radian and adjust from 0-360 to -pi to pi.
    """
    if angle > 180:
        angle -= 360
    return angle * pi / 180

def apply_transform(x, y, yaw):
    """
    Apply a static transformation to the coordinates and yaw.
    """
    # Assuming a static translation and rotation for the example
    # These values would be your static transform values
    translate_x = 1.56133  # Static X translation
    translate_y = 4.49949  # Static Y translation
    init_yaw = convert(188.56)

    yaw_ = convert_to_rad(init_yaw)
    print("yaw_yuan", yaw_)
    rotate_angle = yaw_  # Static rotation in degrees
    R = np.array([
        [np.cos(rotate_angle), -np.sin(rotate_angle)],
                 [np.sin(rotate_angle), -np.cos(rotate_angle)]
    ]
    )
    translation = np.array([-1.56133,-4.49949])
    world_coods = np.array([x,y])
    map_coords = R.dot(world_coods - translation)
    map_yaw = yaw - rotate_angle
    # Convert rotation to radians and apply rotation
    cos_theta = math.cos(rotate_angle)
    sin_theta = math.sin(rotate_angle)
    print("x",map_coords,map_yaw, map_coords[0])
    # print(sin_theta * x , cos_theta * y,translate_y)

    x_new = map_coords[0]#cos_theta * x - sin_theta * y + translate_x
    y_new = map_coords[1]#sin_theta * x + cos_theta * y + translate_y
    yaw_new = map_yaw#yaw + rotate_angle

    return x_new, y_new, yaw_new

def handle_pose(msg, pub):
    """
    Callback function to handle pose updates and convert them to map coordinates.
    """
    yaw_ = convert(msg.yaw)
    # print(yaw_)
    yaw = convert_to_rad(yaw_)  #-pi -pi
    x = -1.56133
    y = -4.49949
    yaw_ = convert(msg.yaw)
    yaw = convert_to_rad(yaw_)  #-pi -pi
    print("yaw_now",x, y,yaw)

    x_new, y_new, yaw_new = apply_transform(msg.x, msg.y, yaw)
    # x_new, y_new, yaw_new = apply_transform(x, y, yaw)

    # Create and publish new pose message
    msg_pose = GpsImuInterface()
    msg_pose.x = x_new
    msg_pose.y = y_new
    msg_pose.yaw = yaw_new*180/pi
    pub.publish(msg_pose)

def listener():
    rospy.init_node('pose_transformer')
    pub = rospy.Publisher('map_pose', GpsImuInterface, queue_size=10)
    rospy.Subscriber('gps_imu', GpsImuInterface, handle_pose, pub)
    rospy.spin()

if __name__ == '__main__':
    listener()
