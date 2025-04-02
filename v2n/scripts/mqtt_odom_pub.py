#!/usr/bin/env python3
'''
Author: CYUN && cyun@tju.enu.cn
Date: 2025-01-12 04:57:45
LastEditors: CYUN && cyun@tju.enu.cn
LastEditTime: 2025-01-13 22:07:08
FilePath: /undefined/home/cyun/forklift_sim_ws3/src/v2n/scripts/mqtt_odom_pub.py
Description: 

Copyright (c) 2025 by Tianjin University, All Rights Reserved. 
'''
import rospy
from nav_msgs.msg import Odometry
import paho.mqtt.client as mqtt
import json
import time


class RosToMqttBridge:
    def __init__(self):
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.username_pw_set(username="tju_me", password="tjuME!@#")
        self.is_mqtt_connected = False
        self.connect_mqtt()

        rospy.init_node('ros_to_mqtt_bridge')
        rospy.Subscriber('rear_odom', Odometry, self.odom_callback)

    def connect_mqtt(self):
        max_retries = 10
        retry_delay = 5
        for attempt in range(max_retries):
            try:
                self.mqtt_client.connect(host="60.28.24.166", port=1883)
                print("Successfully connected to MQTT broker.")
                self.is_mqtt_connected = True
                return
            except Exception as e:
                print(f"Failed to connect to MQTT broker: {e}. Retrying in {retry_delay} seconds...")
                time.sleep(retry_delay)
        print("Failed to connect to MQTT broker after multiple attempts.")

    def odom_callback(self, odom_msg):
        if self.is_mqtt_connected:
            # 将Odometry消息转为字典，方便后续转JSON
            odom_dict = {
                "pose": {
                    "position": {
                        "x": odom_msg.pose.pose.position.x,
                        "y": odom_msg.pose.pose.position.y,
                        "z": odom_msg.pose.pose.position.z
                    },
                    "orientation": {
                        "x": odom_msg.pose.pose.orientation.x,
                        "y": odom_msg.pose.pose.orientation.y,
                        "z": odom_msg.pose.pose.orientation.z,
                        "w": odom_msg.pose.pose.orientation.w
                    }
                },
                "twist": {
                    "linear": {
                        "x": odom_msg.twist.twist.linear.x,
                        "y": odom_msg.twist.twist.linear.y,
                        "z": odom_msg.twist.twist.linear.z
                    },
                    "angular": {
                        "x": odom_msg.twist.twist.angular.x,
                        "y": odom_msg.twist.twist.angular.y,
                        "z": odom_msg.twist.twist.angular.z
                    }
                }
            }
            # print(odom_dict)
            # print( "odom_msg.pose.pose.position.x", odom_msg.pose.pose.position.x)
            self.mqtt_client.publish(topic="/odom/v2n", payload=json.dumps(odom_dict), qos=1, retain=False)
            print("Published odometry data to MQTT")
        else:
            print("MQTT is not connected, cannot publish odometry data.")

    def start(self):
        if self.is_mqtt_connected:
            self.mqtt_client.loop_start()
            try:
                rospy.spin()
            except KeyboardInterrupt:
                self.mqtt_client.loop_stop()
        else:
            print("MQTT client is not properly connected, exiting.")


if __name__ == '__main__':
    try:
        mqtt_odom_pub = RosToMqttBridge()
        mqtt_odom_pub.start()
    except rospy.ROSInterruptException:
        pass