#!/bin/bash
###
 # @Author: CYUN && cyun@tju.enu.cn
 # @Date: 2024-07-24 14:34:01
 # @LastEditors: CYUN && cyun@tju.enu.cn
 # @LastEditTime: 2024-12-19 16:19:13
 # @FilePath: /undefined/home/cyun/forklift_sim_ws3/src/auto_start/sh/vcan_create.sh
 # @Description: 
 # 
 # Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
### 

sudo apt-get install can-utils -y
pip install python-can can cantools

sudo modprobe vcan
sudo ip link add dev can0 type vcan
sudo ip link set up can0


sudo modprobe vcan
sudo ip link add dev can1 type vcan
sudo ip link set up can1
