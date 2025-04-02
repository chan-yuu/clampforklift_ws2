#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# @Time    : 2024/2/27
# @Author  : Zhang Lianhui CYUN
# @Update  : message feedback & msg check

import json
import time
import paho.mqtt.client as mqtt  # pip install paho-mqtt==1
import rospy
from car_interfaces.msg import  GpsImuInterface
from car_interfaces.srv import TaskSts,TaskStsResponse
import signal
import sys
import math
from collections import deque
import matplotlib.pyplot as plt
import socket
from std_msgs.msg import Int8,Float64
import subprocess


class VehicleStatusReport:
    def __init__(self):
        rospy.init_node('vehicle_status_report', anonymous=True)
        # MQTT settings
        self.username = 'tju_me'
        self.password = 'tjuME!@#'
        self.broker_address = "60.28.24.166"
        self.broker_port = 1883
        self.SN = 'XEIPY30011JA98744'


        self.clamp_width = 0
        self.clamp_height = 0
        self.car_run_mode = 0
        self.EPOSts = 0
        self.Gear = 0
        self.Car_Speed = 0
        self.Mileage = 0
        self.CarStartState = 1
        self.vcu_sts = 0
        self.VCU_Service_Voltage = 0
        self.Fault = 0
        self.CarSts1 = 0
        self.CarSts2 = 0
        self.soc = 0
        self.mass = 0
        self.ad_mode = 0
        self.d_head = 0.0
        self.cte = 0.0
        self.head = 0
        self.assignment_sts = 0
        self.pos_x = 0
        self.pos_y = 0
        self.head = 0

        # Kalman filters
        self.kf_x = KalmanFilter(0.1, 0.1)
        self.kf_y = KalmanFilter(0.1, 0.1)
        self.kf_cos = KalmanFilter(0.1, 0.1)
        self.kf_sin = KalmanFilter(0.1, 0.1)

        # MQTT client
        self.client = mqtt.Client()
        self.client.on_disconnect = self.on_disconnect
        self.client.username_pw_set(self.username, self.password)
        # self.client.connect(self.broker_address, self.broker_port)
        self.connect_to_broker()
    def connect_to_broker(self):
        while True:
            try:
                self.client.connect(self.broker_address, self.broker_port)
                self.client.loop_start()
                # rospy.loginfo("\033[32m Connected to MQTT broker \033[0m")
                break  # 连接成功后退出循环
            except (socket.error, mqtt.socket.error)  as e:
                rospy.logerr(f"\033[91m Failed to connect to MQTT broker. cause: {e}. Retrying in 3 seconds... \033[0m")
                time.sleep(3)
    def on_disconnect(self, client, userdata, rc):
        rospy.logerr("\033[91m Disconnected from MQTT broker \033[0m")
        while not client.is_connected():
            try:
                print("Connection lost. Reconnecting...")
                client.reconnect()
                print("Reconnected successfully.")
            except mqtt.MQTTException:
                print("Reconnect failed. Retrying in 5 seconds...")
                time.sleep(5)
    def GPS_callback(self, msg):
        self.pos_x = self.kf_x.update(round(msg.x, 2))
        self.pos_y = self.kf_y.update(round(msg.y, 2))
        self.head = round(msg.AngleHeading, 2) + 10

        cos_head = math.cos(math.radians(self.head))
        sin_head = math.sin(math.radians(self.head))

        smooth_cos = self.kf_cos.update(cos_head)
        smooth_sin = self.kf_sin.update(sin_head)

        self.head = math.degrees(math.atan2(smooth_sin, smooth_cos))
    def EPO_callback(self, msg):
        self.EPOSts = msg.data

    def SOC_callback(self, msg):
        self.soc = msg.data

    def is_program_running(self, program_name):
        try:
            result = subprocess.check_output(f"ps -ef | grep {program_name} | grep -v grep | wc -l", shell=True)
            count = int(result.decode('utf-8').strip())
            return count > 0
        except subprocess.CalledProcessError as e:
            # 打印异常信息，方便排查问题
            print(f"检查程序运行状态时出现错误: {e}")
            return False

    def send_base_message(self):
        state = {
            "reported": {
                'car_run_mode': self.car_run_mode,
                'epos_ts': self.EPOSts,
                'gear': self.Gear,
                'car_speed': self.Car_Speed,
                'mileage': self.Mileage,
                'clamp_width': self.clamp_width,
                'clamp_height': self.clamp_height,
                'car_start_state': self.CarStartState,
                'fault': self.Fault,
                'car_sts1': self.CarSts1,
                'car_sts2': self.CarSts2,
                'soc': self.soc,
                'assignment_sts': self.assignment_sts,
                'vcu_sts':self.vcu_sts
            }
        }

        rviz_status = self.is_program_running("sim.launch")
        fusion_pointclouds_status = self.is_program_running("fusion_pointclouds.launch")
        location_ndt_gps_status = self.is_program_running("location_ndt_gps.launch")
        camera_node_status = self.is_program_running("camera_node.launch")
        lidar_camera_det_status = self.is_program_running("lidar_camera_det.launch")
        run_hybrid_a_star_status = self.is_program_running("run_hybrid_a_star.launch")
        vehicle_control_status = self.is_program_running("vehicle_control.launch")
        smach_fork_status = self.is_program_running("smach_fork.launch")
        v2n_status = self.is_program_running("v2n.launch")

        driverless_state = {
            "timestamp": int(time.time_ns() // 1000000),
            "x": self.pos_x,
            "y": self.pos_y,
            "head": self.head,
            "unattended_state": 1,
            "sim": 1,#rviz_status,
            "fusion_pointclouds": 1,#fusion_pointclouds_status,
            "location_ndt_gps": 1,#location_ndt_gps_status,
            "camera_node": 1,#camera_node_status,
            "lidar_camera_det": 1,#lidar_camera_det_status,
            "run_hybrid_a_star": 1,#run_hybrid_a_star_status,
            "vehicle_control": 1,#vehicle_control_status,
            "smach_fork": 0,#smach_fork_status,
            "v2n": 0,#v2n_status
        }

        v2n_json_data = json.dumps({"timestamp": int(time.time_ns() // 1000000), "state": state}, indent=4)
        driverless_json = json.dumps(driverless_state, indent=4)
        self.client.publish(f"suntae/agv/{self.SN}/base/up", v2n_json_data, qos=1)
        self.client.publish(f"suntae/agv/{self.SN}/driveless/up", driverless_json, qos=1)
        rospy.loginfo(f"\033[32m driverless_json:{driverless_json} \033[0m")
        rospy.loginfo(f"\033[32m v2n_json_data:{v2n_json_data} \033[0m")


    def send_task_sts_message(self,req):
        # 完成任务后，上传任务状态
        task_sts_data = {
            "order_id": req.order_id,
            "task_id": req.task_id,
            "business_type" : req.business_type,
            "origin_shelf_area": req.origin_shelf_area,
            "destination_shelf_area": req.destination_shelf_area,
            "bale_num_to_handle": req.bale_num_to_handle,
            "task_sts": req.task_sts, 
            "final_pose":{
                "x": req.final_pose.x,
                "y": req.final_pose.y,
                "yaw": req.final_pose.yaw
            }
        }
        task_sts_json = json.dumps(task_sts_data, indent=4)
        self.client.publish(f"suntae/agv/{self.SN}/task_sts/up",task_sts_json,qos=1)
        rospy.loginfo(f"\033[33m task_sts_data:{task_sts_data} \033[0m")
        # rospy.loginfo(f"\033[32m 任务{str(req.task_id)[-4:]}完成,反馈云端成功 \033[0m")
        res = TaskStsResponse()
        res.success = True
        return res
        
    def heartbeat(self):
        v2n_json_data = json.dumps({"timestamp": int(time.time_ns() // 1000000)}, indent=4)
        self.client.publish(f"suntae/agv/{self.SN}/heartbeat/up", v2n_json_data, qos=1)

    def run(self):
        rospy.Service("task_sts_service",TaskSts,self.send_task_sts_message)
        rospy.Subscriber('gps_imu', GpsImuInterface, self.GPS_callback)
        rospy.Subscriber('eposts_state', Int8, self.EPO_callback)
        rospy.Subscriber('soc_state', Float64, self.SOC_callback)  
        self.client.loop_start()

        signal.signal(signal.SIGINT, self.signal_handler)
        try:
            while True:
                self.send_base_message()
                self.heartbeat()
                time.sleep(1)
        except KeyboardInterrupt:
            self.client.loop_stop()
            sys.exit(1)

    def start_plotting(self):
        plt.show()

    @staticmethod
    def signal_handler(self, signal):
        sys.exit(0)

class KalmanFilter:
    def __init__(self, process_var, measurement_var, estimate_var=3):
        self.process_var = process_var
        self.measurement_var = measurement_var
        self.estimate_var = estimate_var
        self.estimate = 0.0

    def update(self, measurement):
        kalman_gain = self.estimate_var / (self.estimate_var + self.measurement_var)
        self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)
        self.estimate_var = (1 - kalman_gain) * self.estimate_var + self.process_var
        return self.estimate


if __name__ == '__main__':
    reporter = VehicleStatusReport()
    reporter.run()