#!/usr/bin/env python3
#coding=utf-8

import rospy
from car_interfaces.msg import GpsImuInterface
import math
import json
import serial
import rospy
import std_msgs.msg
from car_interfaces.msg import GpsImuInterface, GpsImuAddInterface
import math
from datetime import datetime, timedelta
import time
import pyproj
import signal

def from_latlon(latitude, longitude, force_zone_number=None):

    """This function convert Latitude and Longitude to UTM coordinate

        Parameters
        ----------
        latitude: float
            Latitude between 80 deg S and 84 deg N, e.g. (-80.0 to 84.0)

        longitude: float
            Longitude between 180 deg W and 180 deg E, e.g. (-180.0 to 180.0).

        force_zone number: int
            Zone Number is represented with global map numbers of an UTM Zone
            Numbers Map. You may force conversion including one UTM Zone Number.
            More information see UTMzones [1]_

       .. _[1]: http://www.jaworski.ca/UTMzones.htm
    """
    #__all__ = ['to_latlon', 'from_latlon']

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



# 0-360 ->  -pi - pi
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


if __name__ == "__main__":
    rospy.init_node("gps_imu_pub_node")  
    rospy.logwarn("start")

    pub = rospy.Publisher("gps_imu", GpsImuInterface, queue_size = 10) 
    # rospy.init_node("gps_imu_pub", anonymous=True)
    pub_location = rospy.Publisher('/ztbus/location', std_msgs.msg.String, queue_size=10)

    rate = rospy.Rate(10)
    content = {}

    while not rospy.is_shutdown():
        rospy.loginfo("success")  
        # ,
        content["Lat"] = 38.99708624484#buf_pos_lat # GPS_MSG.PosLat
        content["Lon"] = 117.29806014549#buf_pos_lon # GPS_MSG.PosLon
        content["Head"] = -92.05994637500001#buf_angle_heading # GPS_MSG.AngleHeading
        content["Speed"] = 1#buf_vel # GPS_MSG.Vel
        content["UTM_x"] = from_latlon(38.99708624484, 117.29806014549)[0]
        content["UTM_y"] = from_latlon(38.99708624484, 117.29806014549)[1]

        pub_location.publish(json.dumps(content))

        # pub.publish(msg)

        rate.sleep()

