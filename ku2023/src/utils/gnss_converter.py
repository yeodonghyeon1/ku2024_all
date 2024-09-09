#!/usr/bin/env python
# -*- coding:utf-8 -*-

#######################################################################
# Copyright (C) 2022 EunGi Han(winterbloooom) (winterbloooom@gmail.com)
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>

"""Subscribe GNSS data and publish its converted data in ENU coordinate system

GNSS Converter Node & function
===========================================================================

.. ENU Coordinate system:
    https://en.wikipedia.org/wiki/Axes_conventions#Ground_reference_frames:_ENU_and_NED

.. pymap3d library:
    https://pypi.org/project/pymap3d/

Attributes:
    origin (list): 
        Origin coordinate(latitude, longitude, altitude) data with DD.MMMMMMM format. 
        Get this data from /tricat221/params/coordinates.ymal (e.g. [35.0695517, 128.5788733, 49.0] )
    boat (list):
        Coordinate data list to subscribe and save current location with ENU coordinate system. 
        Raw GNSS data is converted using "origin".

Notes:
    Using "pymap3d" library, we can convert GNSS datas with various formats. 
    In this module, geodetic coordinates(lat, lon, al) -> ENU coordinates(e, n, u)
"""

import pymap3d as pm
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix,Imu
import math
from rtcm_msgs.msg import Message

origin = rospy.get_param("origin") 
boat = [0, 0, 0]
vel_x=0.0
vel_y=0.0
vel_z=0.0
temp_X=0.0
temp_y=0.0
temp_z=0.0

num_vel=0.0000001 #imu가속 값 xy보정 상수값
num_diff_x=0.75
num_diff_y=0.45

def gps_fix_callback(msg):
    """Subscribe geodetic coordinate from GPS and convert in ENU coordinate

    Args:
        msg (NavSatFix): geodetic coordinate including latitude, lonagitude and altitude value
    """
    boat[0], boat[1], boat[2] = enu_convert([msg.latitude, msg.longitude, msg.altitude])
    #위도, 경도, 고도

def imu_ang_vel_callback(msg):
    global vel_x
    global vel_y
    global vel_z
    # vel_x = msg.orientation.x
    # vel_y = msg.orientation.y
    # vel_z = msg.orientation.z
    # vel_x = msg.angular_velocity.x
    # vel_y = msg.angular_velocity.y
    # vel_z = msg.angular_velocity.z
    vel_x=msg.linear_acceleration.x
    vel_y=msg.linear_acceleration.y
    vel_z=msg.linear_acceleration.z

def enu_convert(gnss):
    """Convert in ENU coordinate using pymap3d
    
    Args:
        gnss (list): a list including [latitude, longitude, altitude]

    Returns:
        e (float): ENU East coordinate (meters) from "origin"
        n (float): ENU North coordinate (meters) from "origin"

    Note:
        In i-Tricat221 coordinate system, the x-axis is magnetic North.
        So, return [n, e, u] not [e, n, u]
    """
    # global temp_X
    # global temp_y
    # global temp_z

    # temp_X=gnss[0]+((round(vel_x,4)- num_diff_x)*num_vel)
    # temp_y=gnss[1]+((round(vel_y,4)- num_diff_y)*num_vel)
    # temp_z=gnss[2]+(round(vel_z,4)*num_vel)

    # temp_X=gnss[0]+vel_x*num_vel
    # temp_y=gnss[1]+vel_y*num_vel
    # temp_z=gnss[2]+vel_z*num_vel



    #e, n, u = pm.geodetic2enu(temp_X, temp_y, temp_z, origin[0], origin[1], origin[2])
    e, n, u = pm.geodetic2enu(gnss[0], gnss[1], gnss[2], origin[0], origin[1], origin[2]) #for fake gps


    return n, e, u 


def main():
    
    etk = rospy.get_param("etk_use")
    etk_use = ""
    if etk == True:
        etk_use = "/gps/filtered"
    else:
        etk_use = "/ublox_gps/fix"
    print(etk_use)
    rospy.init_node("gnss_converter", anonymous=True)
    rospy.Subscriber("{}".format(etk_use), NavSatFix, gps_fix_callback, queue_size=1)
    # rospy.Subscriber("/gps/filtered", NavSatFix, gps_fix_callback, queue_size=1)
    pub = rospy.Publisher("enu_position", Point, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz
    enu_position = Point()
    # new_fake_gps=NavSatFix()

    while not rospy.is_shutdown():
        enu_position.x = boat[0]
        enu_position.y = boat[1]
        
        # new_fake_gps.latitude=temp_X
        # new_fake_gps.longitude=temp_y
        # new_fake_gps.altitude=temp_z

        pub.publish(enu_position)
        # pub1.publish(new_fake_gps) #for fake gps

        rate.sleep()


if __name__ == "__main__":
    main()
