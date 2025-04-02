#!/usr/bin/env python

'''
Author: CYUN && cyun@tju.enu.cn
Date: 2025-01-09 21:04:38
LastEditors: CYUN && cyun@tju.enu.cn
LastEditTime: 2025-01-11 15:31:28
FilePath: /undefined/home/nvidia/clamp_forklift_ws2/src/lidar_obstacle_detect/src/stop_task_client.py
Description: 

Copyright (c) 2025 by Tianjin University, All Rights Reserved. 
'''
import rospy
from car_interfaces.srv import StopTask, StopTaskRequest, StopTaskResponse

def stop_task_client():
    # 初始化 ROS 节点
    rospy.init_node('stop_task_client')

    # 确保服务已经启动
    rospy.wait_for_service('stop_task', timeout=5.0)

    try:
        # 创建服务客户端
        client = rospy.ServiceProxy('stop_task', StopTask)

        # 创建请求
        req = StopTaskRequest()
        req.data = 1  #

        # 调用服务
        res = client(req)

        # 检查服务调用是否成功
        if res.success:
            rospy.loginfo(f"Service call succeeded: {res.message}")
        else:
            rospy.logerr(f"Service call failed: {res.message}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to call service stop_task: {e}")

if __name__ == "__main__":
    stop_task_client()