'''
Author: CYUN && cyun@tju.enu.cn
Date: 2025-01-09 20:00:35
LastEditors: CYUN && cyun@tju.enu.cn
LastEditTime: 2025-01-09 20:00:51
FilePath: /undefined/home/cyun/forklift_sim_ws3/src/car_ori_display/scripts/intetnet_test.py
Description: 

Copyright (c) 2025 by Tianjin University, All Rights Reserved. 
'''
import socket
import requests
import time
import socket
import requests
import threading

# 设置全局默认超时时间（可选）
socket.setdefaulttimeout(0.5)  # 全局超时设置为1秒

def resolve_hostname(host, timeout=1):
    """尝试解析主机名并限制时间"""
    class ResolveThread(threading.Thread):
        def __init__(self, host):
            super().__init__()
            self.host = host
            self.result = None
            self.error = None

        def run(self):
            try:
                self.result = socket.gethostbyname(self.host)
            except Exception as e:
                self.error = e

    resolve_thread = ResolveThread(host)
    resolve_thread.start()
    resolve_thread.join(timeout)

    if resolve_thread.is_alive():
        # 线程仍在运行，说明超时了
        return False, "Timeout resolving hostname"
    elif resolve_thread.error:
        # 线程执行过程中发生了错误
        return False, str(resolve_thread.error)
    else:
        return True, resolve_thread.result

def check_internet_connection(host="www.baidu.com", port=80, timeout=1):
    try:
        # 尝试解析主机名，确保 DNS 正常工作
        success, result = resolve_hostname(host, timeout)
        if not success:
            print(f"Failed to resolve hostname: {result}")
            return False
        
        print(f"Resolved {host} to {result}")

        # 尝试建立 TCP 连接，确保可以到达目标服务器
        with socket.create_connection((host, port), timeout=timeout) as sock:
            print(f"Successfully connected to {host}:{port}")

        # 尝试发送 HTTP 请求，确保可以通过 HTTP 获取内容
        response = requests.get(f"http://{host}", timeout=timeout)
        if response.status_code == 200:
            print(f"Successfully received response from {host}")
            return True
        else:
            print(f"Received unexpected status code: {response.status_code}")
            return False

    except (socket.timeout, socket.error):
        print("Failed to connect to host")
        return False
    except requests.RequestException as e:
        print(f"Failed to get response from host: {e}")
        return False

# 测试函数
if __name__ == "__main__":
    while True:
        if check_internet_connection():
            print("Internet is reachable")
        else:
            print("No internet connection")
        time.sleep(0.1)