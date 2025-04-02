#!/bin/bash
###
 # @Author: CYUN && cyun@tju.enu.cn
 # @Date: 2024-07-24 14:34:01
 # @LastEditors: CYUN && cyun@tju.enu.cn
 # @LastEditTime: 2024-10-15 00:10:12
 # @FilePath: /undefined/home/cyun/forklift_sim_ws/vcan_delete.sh
 # @Description: 
 # 
 # Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
### 

# 关闭vcan0接口
sudo ip link set down can0
# 删除vcan0接口
sudo ip link delete can0

# 关闭vcan0接口
sudo ip link set down can1
# 删除vcan0接口
sudo ip link delete can1
# 卸载vcan模块
sudo modprobe -r vcan