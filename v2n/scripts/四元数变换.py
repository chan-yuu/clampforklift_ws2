'''
Author: CYUN && cyun@tju.enu.cn
Date: 2024-10-22 16:54:22
LastEditors: CYUN && cyun@tju.enu.cn
LastEditTime: 2024-10-25 15:18:19
FilePath: /src/v2n/scripts/四元数变换.py
Description: 

Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
'''
import math

def quaternion_to_yaw(quaternion):
    # 四元数分量
    x, y, z, w = quaternion
    
    # 计算偏航角（yaw）
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    
    # 将弧度转换为角度
    yaw_degrees = math.degrees(yaw)
    
    return yaw_degrees

# 示例四元数
quaternion = (-0.0080,0.0145,0.7037,0.7103)
yaw_angle = quaternion_to_yaw(quaternion)

print(f"Yaw angle: {yaw_angle:.2f} degrees")