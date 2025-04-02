#!/usr/bin/env python3

'''
Author: CYUN
Date: 2024-10-07 08:49:51
LastEditors: CYUN && cyun@tju.enu.cn
LastEditTime: 2025-01-13 03:13:08
FilePath: /smach_fork/scripts/task_smach.py
Description: version:3.0

Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
'''

import rospy
import smach
import smach_ros
import signal
import sys
# from smach_sim_tasks5 import create_state_machine
import tkinter as tk
from tkinter import filedialog
import os
import rospkg



def signal_handler(sig, frame):
    print(f"\nReceived {sig} signal, exiting gracefully.")
    sys.exit(0)


def main():
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('autonomous_clamping_state_machine')
    
    choose_fsm = rospy.get_param('choose_fsm', True)

    # add 1.6 cyun 
    if (choose_fsm):
        # 初始寻找位置
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('smach_fork')
        file_path = f'{package_path}/scripts'
        os.chdir(file_path)

        root = tk.Tk()
        root.withdraw()

        file_path = filedialog.askopenfilename()
        if file_path:
            module_name = file_path.split('/')[-1].split('.')[0]
            try:
                import_module = __import__(module_name)
                create_state_machine = getattr(import_module, 'create_state_machine')
            except (ImportError, AttributeError):
                print(f"无法导入模块 {module_name}，请检查文件是否正确。")
                sys.exit(1)
        else:
            print("未选择任何文件，程序退出。")
            sys.exit(1)
    else:
        from smach_sim_tasks7 import create_state_machine


    # sm = smach.StateMachine(outcomes=['succeed', 'aborted','preempted'])
    
    # 创建状态机
    sm = create_state_machine()

    # 可视化
    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()

    # 运行状态机
    outcome = sm.execute()
    
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()