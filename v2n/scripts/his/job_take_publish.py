#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# @Time    : 2024/2/27
# @Author  : Zhang Lianhui

"""
功能描述：
根据云平台的指令发布任务请求

2024.3.15:
mqtt车端订阅云端程序重写,增加调度相关的指令解析。
2024.3.27:
将接收到的指令存储在redis，实现路线拼接指令逻辑。
"""


from __future__ import print_function
import rospy
from v2n.srv import *
from v2n.srv import *

import redis
import json
import paho.mqtt.client as mqtt
import threading
import json
import rospy
import time
import std_msgs.msg
import signal
import random
import sys
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
# from car_interfaces.msg import V2NInterface
# MQTT代理服务器的账户地址和端口
username = 'tju_me'
password = 'tjuME!@#'
broker_address = "60.28.24.166"

# username  = 'Tianda2024'
# password = 'Tianda2024'
# broker_address = "47.105.56.58"
broker_port = 1883
json_save2redis = {}
# 客户端的名称
randint = random.randint(1,1000)
client_id = f"vehicle_task_planning_client{randint}"

# 定义全局变量
control_json = {}

# init ros topic
# tar_point_route_pub = rospy.Publisher('/v2n/tar_point_route', V2NInterface, queue_size=10)
# order_sts_ctrl_pub = rospy.Publisher('/v2n/order_sts_ctrl', V2NInterface, queue_size=10)
# car_ctrl_pub = rospy.Publisher('/v2n/car_ctrl', V2NInterface, queue_size=10)
# job_task_pub = rospy.Publisher('/v2n/job_task', V2NInterface, queue_size=10)



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

    initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    goal_pose_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    while 1:
        if control_json:
            print("收到云平台json:",control_json)
        if 'commandIdNo' in control_json.keys():
            task_id = control_json["commandIdNo"]
            task = control_json
        else:
            task_id = None
            task = {}
        control_msg = {"task_id": task_id}
        if task_id == 1001:
 
            # 目标点与路线下发
            # try:
            order_id = task["order_id"]
            task_id = task["task_id"]
            target_point_list = str(task["target_point_list"])
            map_list = str(task["map_list"])
            # start_head = task["start_head"]
            # end_head = task["end_head"]
            # site_array = task["site_array"]

            start_head_list = str(task["start_head_list"])
            end_head_list = str(task["end_head_list"])
            can_save = 1

            # 设置位姿信息
            initial_pose = PoseWithCovarianceStamped()
            initial_pose.header.frame_id = "map"  # 或者其他合适的坐标系
            initial_pose.header.stamp = rospy.Time.now()
            initial_pose.pose.pose.position.x = task["target_point_list"][0][0]
            initial_pose.pose.pose.position.y = task["target_point_list"][0][1]
            yaw = task["start_head_list"][0]
            half_yaw = yaw / 2.0

            initial_pose.pose.pose.position.z = 0.0
            initial_pose.pose.pose.orientation.x = 0.0
            initial_pose.pose.pose.orientation.y = 0.0
            initial_pose.pose.pose.orientation.z = math.sin(half_yaw)
            initial_pose.pose.pose.orientation.w = math.cos(half_yaw)
            initial_pose_pub.publish(initial_pose)
            
            # 云端发布的坐标_1 是起点,坐标_2是终点

            # goal位姿
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"  # 或者其他合适的坐标系
            goal_pose.header.stamp = rospy.Time.now()
            goal_pose.pose.position.x = task["target_point_list"][1][0]
            goal_pose.pose.position.y = task["target_point_list"][1][1]
            yaw = task["end_head_list"][0]
            half_yaw = yaw / 2.0
            goal_pose.pose.position.z = 0.0
            goal_pose.pose.orientation.x = 0.0
            goal_pose.pose.orientation.y = 0.0
            goal_pose.pose.orientation.z = math.sin(half_yaw)
            goal_pose.pose.orientation.w = math.cos(half_yaw)
            goal_pose_pub.publish(goal_pose)




            # # 将字典序列化为JSON字符串
            # # 从redis读取，判断之前是否已经有同订单号的任务，如果有，则需要拼接线路。
            # try:
            #     # 从redis读取当前车辆SN，如果不存在则创建空json保存
            #     data_from_redis = json.loads(r.get(SN))
            # except:
            #     r.set(SN,json.dumps({}))
            
            # try:
            #     # 这是旧订单号，判断是否需要更新路线
            #     pre_order_id = data_from_redis["order_id"]
            #     pre_task_id = data_from_redis["task_id"]

            #     if pre_task_id == task_id and pre_order_id == order_id:
            #         print(f"task:{pre_task_id}: 保存失败！因为订单号和任务id和之前的都一样")
            #         can_save = 0
            #     else:
            #         can_save = 1

            #     try:
            #         pre_target_point_list = eval(data_from_redis["target_point_list"])
            #     except:
            #         pre_target_point_list = (data_from_redis["target_point_list"])
            #     pre_target_point = pre_target_point_list[-1]

            #     if can_save:
            #         # 之前的任务id和现在的任务id不一样，但是订单号一样
            #         if pre_target_point == eval(target_point_list)[0]:
            #             # 之前的最后一个点坐标和新的目标点第一个坐标一样，则更新线路
            #             print(f"订单{order_id}线路已经更新")
            #             target_point_list = pre_target_point_list + eval(target_point_list)[1:]
            #             print("拼接后：", target_point_list )
            #         else:
            #             print("订单号相同，任务id相同，但是新目标点列表第一个点和之前订单目标点列表中的最后一个点不一样，无法下单！（请销毁上一个订单，或者检查目标点）")
            #             can_save = 0
            # except Exception as e:
            #     print(e)

            # json_save2redis = {
            #     "order_id":order_id,
            #     "task_id" : task_id,
            #     "target_point_list" : target_point_list,
            #     "map_list" : map_list,
            #     # "site_array":site_array,
            #     "status": -1,
            #     "floors": -1,
            #     "type":-1,
            #     'order_task_status':1,
            #     "start_head" : start_head_list,
            #     "end_head" : end_head_list
            # }

            # # 通过服务通信向线控程序发送指令
            # tar_poi_route_command2srv(json_save2redis)

            # json_data = json.dumps(json_save2redis)
            # if can_save:
            #     r.set(SN,json_data)
            #     print("save ok")

            # except:
            #     print("fail to save")

        elif task_id == 1002:
            """
            订单状态控制
                status:
                0 暂停: 车辆立即停止所有作业,并立即停车
                1 执行: 执行订单任务
                2 销毁: 销毁清除订单,立即停止所有作业,并立即停车
            """
            order_id = task["order_id"]
            print("int(task[order_id])",task["order_id"])
            status  = int(task["status"])
            # task_id = task["task_id"]
            try:
                # 从数据库读取当前车辆的记录
                json_save2redis = json.loads(r.get(SN))
                if json_save2redis["order_id"] == order_id:
                    # 当前从云端接受到的订单id和之前数据库里面保存的订单一样则添加status字段
                    json_save2redis["status"] = status
                    order_sts_ctrl_command2srv(json_save2redis)
                    if status == 2:
                        # 销毁
                        json_save2redis = {}
                    r.set(SN,json.dumps(json_save2redis))

            except:
                json_save2redis["order_id"] = order_id 
                json_save2redis["status"] = status
                json_save2redis["task_id"] = task_id
                json_save2redis["order_task_status"] = 1
                r.set(SN,json.dumps(json_save2redis))
                print(f"ERROR: order_id:{order_id} not there,create now")
        elif task_id == 1003:
            """
            车辆控制
                gear:
                无目标点及路线时才可以更换档位
                只可选 R/D 两种档位
            """
            order_id = task["order_id"]
            task_id = task["task_id"]
            gear = str(task["gear"])
            try:
                json_save2redis = json.loads(r.get(SN))
                if json_save2redis["order_id"] == order_id:
                    json_save2redis["gear"] = gear
                    json_save2redis["task_id"] = task_id
                    gear_command2srv(json_save2redis)
                    r.set(SN,json.dumps(json_save2redis))
                else:
                    print("订单号和当前进行的订单号不同")
            except:
                print(f"ERROR: order_id:{order_id} not there")
            

        elif task_id == 1004:
            """
            作业任务
                type:
                0: 放货 1: 插货
            """
            order_id = task["order_id"]
            task_id = task["task_id"]
            floors= int(task["floors"])
            task_type = bool(task["type"])
            # try:
            json_save2redis = json.loads(r.get(SN))
            if json_save2redis["order_id"] == order_id:
                json_save2redis["floors"] = floors
                json_save2redis["task_id"] = task_id
                json_save2redis["type"] = task_type
                json_save2redis["order_task_status"] = 1
                r.set(SN,json.dumps(json_save2redis))
                task_type2srv(json_save2redis)
            else :
                print("订单号和当前进行的订单号不同")

            # except Exception as e :
            #     print(f"ERROR: order_id:{order_id} not there{e}")

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

def order_sts_ctrl_command2srv(json_save2redis):
    rospy.wait_for_service('OrderStsCtrlCommand')
    request = OrderStsCtrlRequest()
    request.order_id = json_save2redis['order_id']
    request.task_id = json_save2redis['task_id']
    request.status = json_save2redis['status']

    try:
        fun = rospy.ServiceProxy('OrderStsCtrlCommand', OrderStsCtrl)
        resp = fun(request)
        print(f"目标点指令下发状态：{resp}")
        return resp.res
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def gear_command2srv(json_save2redis):
    rospy.wait_for_service('GearCommand')
    request = GearRequest()
    request.order_id = json_save2redis['order_id']
    request.task_id = json_save2redis['task_id']
    request.gear = json_save2redis['gear']

    try:
        fun = rospy.ServiceProxy('GearCommand', Gear)
        resp = fun(request)
        print(f"目标点指令下发状态：{resp}")
        return resp.res
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def task_type2srv(json_save2redis):
    rospy.wait_for_service('TaskTypeCommand')
    request = TaskTypeRequest()
    request.order_id = str(json_save2redis['order_id'])
    request.task_id = str(json_save2redis['task_id'])
    request.type = json_save2redis['type']
    request.floors = json_save2redis['floors']

    try:
        fun = rospy.ServiceProxy('TaskTypeCommand', TaskType)
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