#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# @Time    : 2024/8/30
# @Author  : Zhang Lianhui

"""
功能描述：i9
根据云平台的指令发布任务请求

2024.3.15:
mqtt车端订阅云端程序重写,增加调度相关的指令解析。
2024.3.27:
将接收到的指令存储在redis，实现路线拼接指令逻辑。
"""


from __future__ import print_function
import rospy
from ai_repo.srv import *
from ai_repo.srv import *

import json
import paho.mqtt.client as mqtt
import threading
import json
import rospy
import time
import signal
import random
import sys
from ai_repo.msg import task


# MQTT代理服务器的账户地址和端口
username = 'tju_me'
password = 'tjuME!@#'
broker_address = ""

task_msg = task()
broker_port = 1883
json_save2redis = {}
# 客户端的名称
randint = random.randint(1,1000)
client_id = f"vehicle_task_planning_client{randint}"

# 定义全局变量
control_json = """

{
  "timestamp": "1724389841",
  "cmd_id": "1000",
  "order_id": "order_001",
  "business_type": "outbound",
  "origin_shelf_area": "A1",
  "destination_shelf_area": "B1",
  "task_info": [
    {
      "task_id": "task_001",
      "bale_num_to_handle": 5,
      "origin_shelf_job_position": {
        "x": 1.5,
        "y": 2.5,
        "z": 0.0,
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 1.57
      },
      "destination_shelf_job_position": {
        "x": 1.5,
        "y": 2.5,
        "z": 0.0,
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 1.57
      }
    },
    {
      "task_id": "task_002",
      "bale_num_to_handle": 3,
      "origin_shelf_position": {
        "x": 3.0,
        "y": 4.0,
        "z": 0.0,
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 3.14
      },
      "destination_shelf_position": {
        "x": 3.0,
        "y": 4.0,
        "z": 0.0,
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 3.14
      }
    }
  ]
}

"""



# 连接成功回调函数
def on_connect(client, userdata, flags, rc):
    SN = "XEIPY30011JA98744"
    if rc == 0:
        print("Connected with result code " + str(rc))
        # 一次订阅多个主题,元组第二个参数是qos
        topics = [
            (f'suntae/agv/{SN}/target_point_route/get',2),
            (f"suntae/agv/{SN}/order_status/get",2),
            (f"suntae/agv/{SN}/car_control/get",2),
            (f"suntae/agv/{SN}/job_task/get",2)
        ]
        # 订阅主题
        # topics = "suntae/agv/{SN}/order_status/get
        client.subscribe(topics)
    else:
        print("Connection failed. Retrying...")
        client.reconnect()
# 连接失败回调函数
def on_disconnect(client, userdata, rc):
    if rc != 0:
        print("Connection lost. Reconnecting...")
        client.reconnect()


# 收到消息的回调函数
def on_message(client, userdata, msg):
    global control_json
    global is_exit
    global new_mesg
    # print("Received message: " + msg.payload.decode())
    control_json = json.loads(msg.payload.decode())
    new_mesg = 1


def signal_handler(sig, frame):
    sys.exit(0)
new_mesg = 0

#  任务发布
def task_planning(r,SN):
    """
    r : redis.Redis()
    SN ： ID of forklift
    """
    global control_json
    global new_mesg
    global json_save2redis

    while True:
        if control_json:
            print("收到云平台json:",control_json)
        if 'cmd_id' in control_json.keys():
            cmd_id = control_json["cmd_id"]
        else:
            cmd_id = None
            control_json = {}
        if cmd_id == 1000:
            timestamp = control_json["timestamp"]
            order_id = control_json["order_id"]
            business_type = control_json["business_type"]
            origin_shelf_area = control_json["origin_shelf_area"]
            destination_shelf_area = control_json["destination_shelf_area"]
            task_info = control_json["task_info"]
            
            for single_task in task_info:
                task_id = single_task["task_id"]
                bale_num_to_handle = single_task["bale_num_to_handle"]
                
                # 检查是否有 origin_shelf_job_position 或 origin_shelf_position
                if "origin_shelf_job_position" in single_task:
                    origin_shelf_job_position = single_task["origin_shelf_job_position"]
                else:
                    origin_shelf_job_position = None
                
                if "destination_shelf_job_position" in single_task:
                    destination_shelf_job_position = single_task["destination_shelf_job_position"]
                else:
                    destination_shelf_job_position = None
            




            # # 通过服务通信向线控程序发送指令
            # tar_poi_route_command2srv(json_save2redis)

            # json_data = json.dumps(json_save2redis)
            # if can_save:
            #     r.set(SN,json_data)
            #     print("save ok")

            # except:
            #     print("fail to save")

        

        control_json = {}
        time.sleep(1)


def tar_poi_route_command2srv(json_save2redis):
    rospy.wait_for_service('TarPoiRouteCommand')
    # 准备请求参数
    request = TarPoiRouteRequest()
    request.order_id = json_save2redis['order_id']
    request.task_id = json_save2redis['task_id']
    request.target_point_list = json_save2redis['target_point_list']
    request.map_list = json_save2redis['map_list']
    request.start_head = json_save2redis['start_head']
    request.end_head = json_save2redis['end_head']
    try:
        fun = rospy.ServiceProxy('TarPoiRouteCommand', TarPoiRoute)
        resp = fun(request)
        print(f"目标点指令下发状态：{resp}")
        return resp.res
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def main():
    rospy.init_node("job_task_publish", anonymous=False)
    signal.signal(signal.SIGINT, signal_handler)
    r = redis.Redis(host='localhost', port=6379, db=0)
    SN = 'XEIPY30011JA98744'

    add_thread = threading.Thread(target=task_planning,args = (r,SN))
    add_thread.start()

    # 创建MQTT客户端
    client = mqtt.Client(client_id)
    # 创建Redis连接

    # 设置连接成功和消息收到的回调函数
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    client.username_pw_set(username, password)
    client.connect(broker_address, broker_port, 60)
    # 开始循环处理网络流量
    client.loop_start()
    try:
        # 持续运行直到手动停止
        while True:
            pass
    except KeyboardInterrupt:
        # 用户按下Ctrl+C停止程序时，进行清理工作
        client.loop_stop()
        # client.disconnect()
        sys.exit(1)

if __name__ == "__main__":
    main()