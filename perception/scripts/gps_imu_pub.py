#!/usr/bin/python3
# -*- coding: utf-8 -*-

# 作者： cyun

# 功能描述：
# 通过can协议（dbc文件）解析组合惯导数据

import can
import cantools
import rospy
import rosparam
from car_interfaces.msg import GpsImuInterface,GpsImuAddInterface

import math
import os
from datetime import datetime, timedelta
import time
import pyproj
import signal
import threading

state_flag = 0
state_flag1 = 0

state_flag2 =0
state_flag3=0

k_s = 0

class OutOfRangeError(ValueError):
    def __init__(self, message):
        self.message = message
        super().__init__(self.message)


def from_latlon(latitude, longitude, force_zone_number=None):
    
    K0 = 0.9996

    E = 0.00669438
    E2 = E * E
    E3 = E2 * E
    E_P2 = E / (1.0 - E)

    SQRT_E = math.sqrt(1 - E)
    _E = (1 - SQRT_E) / (1 + SQRT_E)
    _E2 = _E * _E
    _E3 = _E2 * _E
    _E4 = _E3 * _E
    _E5 = _E4 * _E

    M1 = (1 - E / 4 - 3 * E2 / 64 - 5 * E3 / 256)
    M2 = (3 * E / 8 + 3 * E2 / 32 + 45 * E3 / 1024)
    M3 = (15 * E2 / 256 + 45 * E3 / 1024)
    M4 = (35 * E3 / 3072)

    P2 = (3. / 2 * _E - 27. / 32 * _E3 + 269. / 512 * _E5)
    P3 = (21. / 16 * _E2 - 55. / 32 * _E4)
    P4 = (151. / 96 * _E3 - 417. / 128 * _E5)
    P5 = (1097. / 512 * _E4)

    R = 6378137

    ZONE_LETTERS = "CDEFGHJKLMNPQRSTUVWXX"

    if not -80.0 <= latitude <= 84.0:
        raise OutOfRangeError('latitude out of range (must be between 80 deg S and 84 deg N)')
    if not -180.0 <= longitude <= 180.0:
        raise OutOfRangeError('longitude out of range (must be between 180 deg W and 180 deg E)')

    lat_rad = math.radians(latitude)
    lat_sin = math.sin(lat_rad)
    lat_cos = math.cos(lat_rad)

    lat_tan = lat_sin / lat_cos
    lat_tan2 = lat_tan * lat_tan
    lat_tan4 = lat_tan2 * lat_tan2

    if force_zone_number is None:
        zone_number = latlon_to_zone_number(latitude, longitude)
    else:
        zone_number = force_zone_number

    zone_letter = latitude_to_zone_letter(latitude)

    lon_rad = math.radians(longitude)
    central_lon = zone_number_to_central_longitude(zone_number)
    central_lon_rad = math.radians(central_lon)

    n = R / math.sqrt(1 - E * lat_sin ** 2)
    c = E_P2 * lat_cos ** 2

    a = lat_cos * (lon_rad - central_lon_rad)
    a2 = a * a
    a3 = a2 * a
    a4 = a3 * a
    a5 = a4 * a
    a6 = a5 * a

    m = R * (M1 * lat_rad -
             M2 * math.sin(2 * lat_rad) +
             M3 * math.sin(4 * lat_rad) -
             M4 * math.sin(6 * lat_rad))

    easting = K0 * n * (a +
                        a3 / 6 * (1 - lat_tan2 + c) +
                        a5 / 120 * (5 - 18 * lat_tan2 + lat_tan4 + 72 * c - 58 * E_P2)) + 500000

    northing = K0 * (m + n * lat_tan * (a2 / 2 +
                                        a4 / 24 * (5 - lat_tan2 + 9 * c + 4 * c ** 2) +
                                        a6 / 720 * (61 - 58 * lat_tan2 + lat_tan4 + 600 * c - 330 * E_P2)))

    if latitude < 0:
        northing += 10000000

    return easting, northing, zone_number, zone_letter


def s84_to_utm(latitude,longitude):
    wgs84 = pyproj.CRS.from_string('EPSG:4326') # 使用WGS84 CRS
    utm = pyproj.CRS.from_string('EPSG:32645') # 使用UTM Zone 50 CRS

    transformer = pyproj.Transformer.from_crs(wgs84, utm, always_xy=True)

    # 经纬度转UTM坐标
    utm_x, utm_y = transformer.transform(longitude, latitude)

    # print("UTM坐标:", "[",utm_x,",",utm_y,"]")
    return utm_x,utm_y


def latitude_to_zone_letter(latitude):
    ZONE_LETTERS = "CDEFGHJKLMNPQRSTUVWXX"
    if -80 <= latitude <= 84:
        return ZONE_LETTERS[int(latitude + 80) >> 3]
    else:
        return None


# 0-360 ->  -180 - 180
def angle_2_angle(angle):
    angle -= 90
    while (angle < -180):
        angle = angle + 360
    while (angle > 180):
        angle = angle - 360
    return -angle


# deg -> rad
def deg_2_rad(angle):
    return angle * math.pi / 180


def UTM_local_change(utm_x, utm_y, dis_local_x, dis_local_y, theta):
    theta_to_ego = math.atan2(dis_local_x, dis_local_y) * 180 / math.pi
    dis_local = math.sqrt(math.pow(dis_local_x, 2) + math.pow(dis_local_y, 2))
    utm_x_change_local = utm_x + dis_local * math.cos(theta + theta_to_ego)
    utm_y_change_local = utm_y + dis_local * math.sin(theta + theta_to_ego)

    return utm_x_change_local, utm_y_change_local

def latlon_to_zone_number(latitude, longitude):
    if 56 <= latitude < 64 and 3 <= longitude < 12:
        return 32

    if 72 <= latitude <= 84 and longitude >= 0:
        if longitude <= 9:
            return 31
        elif longitude <= 21:
            return 33
        elif longitude <= 33:
            return 35
        elif longitude <= 42:
            return 37

    return int((longitude + 180) / 6) + 1


def zone_number_to_central_longitude(zone_number):
    return (zone_number - 1) * 6 - 180 + 3


def gps_week_seconds_to_utc(gpsweek, gpsseconds, leapseconds=18):
    datetimeformat = "%Y-%m-%d %H:%M:%S.%f"
    gpsseconds = gpsseconds / 1000

    epoch = datetime.strptime("1980-01-06 08:00:00.000", datetimeformat)
    # timedelta函数会处理seconds为负数的情况
    elapsed = timedelta(days=(gpsweek * 7), seconds=(gpsseconds - leapseconds))
    beijing_time = datetime.strftime(epoch + elapsed, datetimeformat)
    time_stamp = int((time.mktime(datetime.strptime(beijing_time, "%Y-%m-%d %H:%M:%S.%f").timetuple())))

    return beijing_time, time_stamp

def publish_data():
    global state_flag
    global state_flag1
    global k_s

    #nvidia
    gps_dbc_file = '/home/nvidia/forklift_ws/src/perception/config/gps_imu_data.dbc'
    db = cantools.db.load_file(gps_dbc_file)
    GPS_MSG = GpsImuInterface()

    gps_can_channel = 'can0'

    if os.path.exists(gps_dbc_file):
        print("找到文件")
    else:
        print("未找到文件")
        dir_path = os.path.dirname(os.path.abspath(__file__))
        gps_dbc_file = os.path.join(dir_path, gps_dbc_file)
        if os.path.exists(gps_dbc_file):
            print("找到文件")
        else:
            print("仍未找到文件")
        
    try:
        bus = can.interface.Bus(channel=gps_can_channel, bustype='socketcan')
        print("CAN已连接")
    except:
        print("CAN未连接")
    pub = rospy.Publisher('/world_pose', GpsImuInterface, queue_size=10)

    # pub = rospy.Publisher('/gps_imu', GpsImuInterface, queue_size=10)
    while not rospy.is_shutdown():

        message = bus.recv()
        GPS_MSG.header.stamp = rospy.Time.now()
        if message.arbitration_id == 0x10B:
            decoded_message = db.decode_message(message.arbitration_id, message.data)
            heading = decoded_message["heading"]
            GPS_MSG.AngleHeading = angle_2_angle(heading)  
            GPS_MSG.pitch = decoded_message["pitch"]
            GPS_MSG.roll = decoded_message["roll"]
            ## 暂时：
            GPS_MSG.yaw = GPS_MSG.AngleHeading

        elif message.arbitration_id == 0x20B:
            decoded_message = db.decode_message(message.arbitration_id, message.data)
            GPS_MSG.PosLat = decoded_message["lat"]
            GPS_MSG.PosLon = decoded_message["lon"]
            GPS_MSG.posX = from_latlon(GPS_MSG.PosLat, GPS_MSG.PosLon)[0] - 604000
            GPS_MSG.posY = from_latlon(GPS_MSG.PosLat, GPS_MSG.PosLon)[1] - 4084800
            # GPS_MSG.posX = GPS_MSG.posX + math.sin(GPS_MSG.yaw) # from_latlon(GPS_MSG.PosLat, GPS_MSG.PosLon)[0] - 604000

            ## 暂时使用：
            GPS_MSG.x = from_latlon(GPS_MSG.PosLat, GPS_MSG.PosLon)[0] - 604000
            GPS_MSG.y = from_latlon(GPS_MSG.PosLat, GPS_MSG.PosLon)[1] - 4084800



        elif message.arbitration_id == 0x30B:
            decoded_message = db.decode_message(message.arbitration_id, message.data)
            GPS_MSG.PosAlt = decoded_message["alt"]
            GPS_MSG.posZ = GPS_MSG.PosAlt
            GPS_MSG.nav_flag = str(decoded_message["nav_flag"])
            if GPS_MSG.nav_flag == "RTK_fix": # 名字确定一下
                GPS_MSG.state = True # state bool true
            else:
                GPS_MSG.state = False # state bool false

        elif message.arbitration_id == 0x40B:
            decoded_message = db.decode_message(message.arbitration_id, message.data)
            GPS_MSG.VelE = decoded_message["east_speed"]
            GPS_MSG.VelN = decoded_message["north_speed"]
            GPS_MSG.VelU = decoded_message["up_speed"]
            GPS_MSG.Vel = math.sqrt(GPS_MSG.VelE**2+GPS_MSG.VelN**2)
            #暂时使用：
            GPS_MSG.vel = GPS_MSG.Vel
            # print(1111)
        elif message.arbitration_id == 0X50B:
            decoded_message = db.decode_message(message.arbitration_id, message.data)
            GPS_MSG.x_acc = decoded_message["x_acc"]
            GPS_MSG.y_acc = decoded_message["y_acc"]
            GPS_MSG.acc = math.sqrt((GPS_MSG.x_acc)**2+(GPS_MSG.y_acc)**2)

        elif message.arbitration_id == 0X60B:
            decoded_message = db.decode_message(message.arbitration_id, message.data)
            GPS_MSG.x_gyro = decoded_message["x_gyro"]
            GPS_MSG.y_gyro = decoded_message["y_gyro"]

        elif message.arbitration_id == 0x70B:
            decoded_message = db.decode_message(message.arbitration_id, message.data)
            GPS_MSG.z_acc = decoded_message["z_acc"]
            GPS_MSG.z_gyro = decoded_message["z_gyro"]

        elif message.arbitration_id == 0X09B:
            decoded_message = db.decode_message(message.arbitration_id, message.data)
            GPS_MSG.gps_week = decoded_message["gps_week"]
            GPS_MSG.gps_ms = decoded_message["gps_ms"]

        # elif message.arbitration_id == 0x21B:
        #     decoded_message = db.decode_message(message.arbitration_id, message.data)
        #     GPS_MSG.lat_g = decoded_message["lat"]
        #     GPS_MSG.lon_g = decoded_message["lon"]
        #     GPS_MSG.posX = from_latlon(GPS_MSG.lat_g, GPS_MSG.lon_g)[0]
        #     GPS_MSG.posY = from_latlon(GPS_MSG.lat_g, GPS_MSG.lon_g)[1]
        #     if k_s == 0:
        #         pos_x = GPS_MSG.posX
        #         pos_y = GPS_MSG.posY
        #         k_s = 1
        #     dis = math.sqrt((GPS_MSG.posX-pos_x)**2+(GPS_MSG.posY-pos_y)**2)
        #     # print("locat dis error", dis)
        #     pos_x = GPS_MSG.posX
        #     pos_y = GPS_MSG.posY

        # elif message.arbitration_id == 0x41B:
        #     decoded_message = db.decode_message(message.arbitration_id, message.data)
        #     GPS_MSG.east_speed_g = decoded_message["east_speed"]
        #     GPS_MSG.north_speed_g = decoded_message["north_speed"]            
        #     GPS_MSG.up_speed_g = decoded_message["up_speed"]

        # elif message.arbitration_id == 0x31B:
        #     decoded_message = db.decode_message(message.arbitration_id, message.data)
        #     GPS_MSG.alt_g = decoded_message["alt"]
        #     GPS_MSG.nav_flag_g = str(decoded_message["nav_flag"])

        # elif message.arbitration_id == 0X51B:
        #     decoded_message = db.decode_message(message.arbitration_id, message.data)
        #     GPS_MSG.heading_g = decoded_message["heading"]

        pub.publish(GPS_MSG)
        time.sleep(0.01)


def quit(signal, frame):
    rospy.logwarn("Stopping...")
    rospy.signal_shutdown("Quit signal received")


def main():

    signal.signal(signal.SIGINT, quit)
    rospy.init_node("gps_imu_pub")
    rospy.logwarn("start")
    publish_thread = threading.Thread(target=publish_data)

    publish_thread.start()
    rospy.spin()
    publish_thread.join()
    
if __name__ == "__main__":
    main()
