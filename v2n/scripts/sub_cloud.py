#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# @Time    : 2024/8/30
# @Author  : Zhang Lianhui CYUN
# @Update  : Network reconnection && Cloud platform startup program and feedback

import rospy
from car_interfaces.srv import cloud_order, cloud_orderRequest
from car_interfaces.msg import single_task
from car_interfaces.srv import EmergencyStopTask, EmergencyStopTaskRequest, BalesNumInCamera, BalesNumInCameraRequest
from std_msgs.msg import Header
import json
import paho.mqtt.client as mqtt
import time
import signal
import random
import sys
import threading

import os
import rospkg


class TaskManager:
    def __init__(self):
        self.username = "tju_me"
        self.password = "tjuME!@#"
        self.broker_address = "60.28.24.166"
        self.broker_port = 1883
        self.client_id = f"vehicle_task_planning_client{random.randint(1, 1000)}"
        self.cloud_json_received = {}
        self.new_mesg = 0
        self.SN = "XEIPY30011JA98744"
        self.cmd_json = ""
        self.order_id = ""
        self.task_info = []
        self.emergency_stop_en = 0
        self.tar_task_id = None
        self.set_task_sts = None
        self.camrea_enable = -1
        self.on_connect_ok = False
        self.task_id = None
        self.retry_interval = 5
        self.max_connection_attempts = 10

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.on_connect_ok = True
            rospy.loginfo("\033[32m 与云端连接成功\033[0m")
            topics = [
                (f"suntae/agv/{self.SN}/task", 2)
            ]
            client.subscribe(topics)
        else:
            print("Connection failed. Retrying...")
            client.reconnect()

    def on_disconnect(self, client, userdata, rc):
        if rc!= 0:
            print("Connection lost. Reconnecting...")
            client.reconnect()

    def on_message(self, client, userdata, msg):
        self.cloud_json_received = json.loads(msg.payload.decode())
        self.new_mesg = 1
        rospy.loginfo('\033[33m 收到云平台消息\033[0m')
        print(f"云端:{self.cloud_json_received}")

    def signal_handler(self, sig, frame):
        rospy.signal_shutdown("KeyboardInterrupt")
        sys.exit(0)

    @staticmethod
    def waiting_animation(message, duration=1):
        symbols = ['/', '-', '\\', '|']
        start_time = time.time()
        symbol_index = 0

        while time.time() - start_time < duration:
            symbol = symbols[symbol_index % len(symbols)]
            print(f"\r{message} {symbol}", end='', flush=True)
            symbol_index += 1
            time.sleep(0.1)

    def handle_inbound_task(self):
        self.order_id = self.cloud_json_received["order_id"]
        self.business_type = self.cloud_json_received["business_type"]
        self.task_info = self.cloud_json_received["task_info"]
        self.task_id = str(self.task_info[0]['task_id'])
        rospy.loginfo(f"\033[90m收到任务:\ntask id:{str(self.task_id)[-4:]}\n业务类型：{self.business_type}\n目标货架：{self.task_info[0]['destination_shelf_area']}\033[0m")
        self.cmd_request(self.task_info[0])

    def run_command(self, command):
        global debug_mode
        # No need to display on the terminal in debug mode
        if debug_mode:
            os.system(f'terminator -e "{command}" &')
        else:
            os.system(f'nohup {command} > /dev/null 2>&1 &')

    def stop_command(self, command_name):
        os.system(f'pkill -f "{command_name}"')


    def task_planning(self):        
        while not rospy.is_shutdown():
            
            if self.on_connect_ok:
                if self.new_mesg == 1:
                    if "cmd_id" in self.cloud_json_received.keys():
                        cmd_id = str(self.cloud_json_received["cmd_id"])
                    else:
                        cmd_id = None
                    if cmd_id == "1001":
                        self.handle_inbound_task()
                    elif cmd_id == "1003":
                        self.emergency_stop_en = self.cloud_json_received["epos"]
                        self.emergency_stop_request()
                    elif cmd_id == "1004":
                        self.tar_task_id = self.cloud_json_received["tar_task_id"]
                        self.set_task_sts = self.cloud_json_received["set_task_sts"]
                    elif cmd_id == "1005":
                        self.camrea_enable = self.cloud_json_received["camrea_enable"]
                        if self.camrea_enable == 1:
                            self.camrea_enable_request(2)
                    
                    if cmd_id == "1010":
                        if self.cloud_json_received['function'] == "lidar":
                            if self.cloud_json_received['status'] == "1":
                                # 启动雷达融合节点
                                self.run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash;\
                                                  roslaunch fusion_pointclouds fusion_pointclouds.launch'")
                            elif self.cloud_json_received['status'] == "0":
                                # 关闭
                                self.stop_command("fusion_pointclouds.launch")
                        elif self.cloud_json_received['function'] == "camera":
                            if self.cloud_json_received['status'] == "1":
                                self.run_command(f"bash -c 'source {WORKSPACE_PATH}/src/lidar_camera_det/sh/start_camera_lidar_det.sh'")
                            elif self.cloud_json_received['status'] == "0":
                                self.stop_command("lidar_camera_det.launch")
                                self.stop_command("camera_node.launch")

                        elif self.cloud_json_received['function'] == "auto_start":
                            if self.cloud_json_received['status'] == "1":
                                self.run_command(f"bash -c 'source {WORKSPACE_PATH}/src/auto_start/sh/start_work_d.sh'")
                                # self.run_command(f"bash -c 'source {WORKSPACE_PATH}/src/auto_start/sh/start_work.sh'")
                            elif self.cloud_json_received['status'] == "0":
                                self.stop_command("run_hybrid_a_star.launch")
                                self.stop_command("vehicle_control.launch")
                                self.stop_command("smach_fork.launch")

                        elif self.cloud_json_received['function'] == "car_model":
                            if self.cloud_json_received['status'] == "1":
                                self.run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash; \
                                                 roslaunch car_ori_display sim.launch'")
                            elif self.cloud_json_received['status'] == "0":
                                self.stop_command("sim.launch")

                        elif self.cloud_json_received['function'] == "location":
                            if self.cloud_json_received['status'] == "1":
                                self.run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash;\
                                                 roslaunch perception location_ndt_gps.launch'")
                            elif self.cloud_json_received['status'] == "0":
                                self.stop_command("location_ndt_gps.launch")

                        elif self.cloud_json_received['function'] == "planning":
                            if self.cloud_json_received['status'] == "1":
                                self.run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash;\
                                                 roslaunch hybrid_a_star run_hybrid_a_star.launch'")
                            elif self.cloud_json_received['status'] == "0":
                                self.stop_command("run_hybrid_a_star.launch")

                        elif self.cloud_json_received['function'] == "control":
                            if self.cloud_json_received['status'] == "1":
                                self.run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash;\
                                                  roslaunch control vehicle_control.launch'")
                            elif self.cloud_json_received['status'] == "0":
                                self.stop_command("vehicle_control.launch")
                                
                        elif self.cloud_json_received['function'] == "smach":
                            if self.cloud_json_received['status'] == "1":
                                self.run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash;\
                                                 roslaunch smach_fork smach_fork.launch'")
                            elif self.cloud_json_received['status'] == "0":
                                self.stop_command("smach_fork.launch")

                        elif self.cloud_json_received['function'] == "mapping1":
                            if self.cloud_json_received['status'] == "1":
                                self.run_command(f"bash -c 'source {WORKSPACE_PATH_MAP}/src/auto_start/sh/start_map_1.sh")
                            elif self.cloud_json_received['status'] == "0":
                                self.stop_command("start_rviz.launch")
                                self.stop_command("rviz")
                                self.stop_command("lidar_start.launch")
                                self.stop_command("rosbag")
                        elif self.cloud_json_received['function'] == "mapping2":
                            if self.cloud_json_received['status'] == "1":
                                self.run_command(f"bash -c 'source {WORKSPACE_PATH_MAP}/src/auto_start/sh/start_map_2.sh'")
                            elif self.cloud_json_received['status'] == "0":
                                self.stop_command("run_lego_loam.launch")
                                self.stop_command("rosbag")

                        elif self.cloud_json_received['function'] == "mapping3":
                            if self.cloud_json_received['status'] == "1":
                                self.run_command(f"bash -c 'source {WORKSPACE_PATH_MAP}/src/auto_start/sh/start_map_3.sh'")
                            elif self.cloud_json_received['status'] == "0":
                                self.stop_command("run_pgm.launch")

                        elif self.cloud_json_received['function'] == "mapping4":
                            if self.cloud_json_received['status'] == "1":
                                self.run_command(f"bash -c 'source {WORKSPACE_PATH_MAP}/src/auto_start/sh/start_map_4.sh'")
                            elif self.cloud_json_received['status'] == "0":
                                self.stop_command("ndt_localizer_rviz.launch")
                                self.stop_command("start_no_rviz.launch")
                                self.stop_command("sim.launch")

                        elif self.cloud_json_received['function'] == "mapping5":
                            if self.cloud_json_received['status'] == "1":
                                self.run_command(f"bash -c 'source {WORKSPACE_PATH_MAP}/src/auto_start/sh/start_map_5.sh'")
                            elif self.cloud_json_received['status'] == "0":
                                self.stop_command("rosbag")

                        elif self.cloud_json_received['function'] == "mapping6":
                            if self.cloud_json_received['status'] == "1":
                                self.run_command(f"bash -c 'source {WORKSPACE_PATH_MAP}/src/auto_start/sh/start_map_6.sh'")
                            elif self.cloud_json_received['status'] == "0":
                                self.stop_command("lidar_gps_fusion.py")

                    self.new_mesg = 0
                else:
                    pass
            else:
                print("Waiting for connect to cloud.")
            time.sleep(1)

    def camrea_enable_request(self, camrea_enable):
        rospy.loginfo("wait_for_service bales_num_in_camera_service")
        rospy.wait_for_service("bales_num_in_camera_service")
        request = BalesNumInCameraRequest()
        request.data = camrea_enable
        rospy.ServiceProxy("bales_num_in_camera_service", BalesNumInCamera).call(request)

    def cmd_request(self, task_info):
        rospy.wait_for_service("cloud_order_service")
        request = cloud_orderRequest()
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "world"
        request.header = header
        task = single_task()
        task.task_id = str(task_info["task_id"])
        task.bale_num_to_handle = task_info["bale_num_to_handle"]
        try:
            task.origin_shelf_job_position.position.x = task_info["origin_shelf_job_position"]["x"]
            task.origin_shelf_job_position.position.y = task_info["origin_shelf_job_position"]["y"]
            task.origin_shelf_job_position.position.z = task_info["origin_shelf_job_position"]["z"]
            task.origin_shelf_job_position.orientation.z = task_info["origin_shelf_job_position"]["yaw"]
            task.destination_shelf_job_position.position.x = task_info["destination_shelf_job_position"]["x"]
            task.destination_shelf_job_position.position.y = task_info["destination_shelf_job_position"]["y"]
            task.destination_shelf_job_position.position.z = task_info["destination_shelf_job_position"]["z"]
            task.destination_shelf_job_position.orientation.z = task_info["destination_shelf_job_position"]["yaw"]
            task.destination_shelf_area = task_info["destination_shelf_area"]
            print(f'task.destination_shelf_area:{task.destination_shelf_area}')
        except KeyError as e:
            rospy.logerr(f"货架{e}坐标信息缺失!")
        request.task_info.append(task)
        try:
            fun = rospy.ServiceProxy("cloud_order_service", cloud_order)
            resp = fun(request)
            request.task_info = []
            return resp.task_progress
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            request.task_info = []

    def emergency_stop_request(self):
        print("wait_for_service emergency_stop_ser")
        rospy.wait_for_service("emergency_stop_ser")
        request = EmergencyStopTaskRequest()
        request.data = self.emergency_stop_en
        fun = rospy.ServiceProxy("emergency_stop_ser", EmergencyStopTask)
        resp = fun(request)
        print(f"急停指令下发状态：{resp}")
        return resp.success

    def check_network_connection(self):
        import socket
        try:
            socket.create_connection((self.broker_address, self.broker_port), timeout=5)
            return True
        except OSError:
            return False

    def start(self):
        rospy.init_node("sub_cloud", anonymous=False)
        signal.signal(signal.SIGINT, self.signal_handler)

        global debug_mode
        global WORKSPACE_PATH
        global WORKSPACE_PATH_MAP

        debug_mode = rospy.get_param('debug_mode', True)

        rp = rospkg.RosPack()
        package_name = "v2n"
        self.package_path = rp.get_path(package_name)
        self.workspace_path = os.path.dirname(os.path.dirname(self.package_path))
        WORKSPACE_PATH = self.workspace_path

        WORKSPACE_PATH_MAP = "/home/nvidia/mapping_ws"
        
        # check the network
        while not self.check_network_connection():
            print("Checking network connection...")
            time.sleep(5)

        add_thread = threading.Thread(target=self.task_planning)
        add_thread.start()

        client = mqtt.Client(self.client_id) # 
        client.on_connect = self.on_connect
        client.on_disconnect = self.on_disconnect
        client.on_message = self.on_message
        client.username_pw_set(self.username, self.password)
        client.connect(self.broker_address, self.broker_port, 60)
        client.loop_start()

        try:
            rospy.spin()
        except KeyboardInterrupt:
            client.loop_stop()
            sys.exit(1)

if __name__ == "__main__":
    manager = TaskManager()
    manager.start()