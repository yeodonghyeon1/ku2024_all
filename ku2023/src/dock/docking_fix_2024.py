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

"""Script for autonomous(docking) misson

Notes:
    * State
        * 0: Obstacle Avidance
            * 도킹 시작 지점에 도착하기 전까지는 장애물을 회피하는 자율운항 모드
        * 1: Go to Station 1
            * 도킹 시작 지점 -> station1 위치 이동
        * 2: Go to Station 2
            * station1 -> station2 위치 이동
        * 3: Go to Station 3
            * station2 -> station3 위치 이동
        * 4: Rotating Heading Mode
            * 일정 시간동안 정지한 뒤 스테이션 방향으로 선수각을 제자리 회전함
            * 제어 방식 2가지
                * 단순히 현 vs 목표'차이각' 사용
                * State 6의 도킹 모드에서 사용하는 제어법(스테이션 방향 벡터 위 추종점 이용): 주변 파동으로 제자리 정지/회전이 힘들 수 있기 때문
        * 5: Detecting Target
            * 정지한 상태에서 전면 카메라로 일정 횟수만큼 검출을 반복 시행하고, 지정한 횟수 이상일 때 타겟을 발견했다고 판단함
            * 제어 방식 2가지(위와 동일)s
        * 6: Docking Mode
            * 타겟을 검출했으며, 스테이션 안쪽으로 도킹을 실시함s
            * 도킹 방식 2가지
                * 각 스테이션을 일일히 방문하지 않고, station2(중앙)의 멀리서 한 번에 보고 타겟이 왼/오/중앙 어디에 있는지 파악 후 바로 도킹
                    * state 5에서 바로 시작해야 함
                    * 제어 방식 2번을 선택해야 함
                * 각 스테이션을 차례대로 방문해서 지금 보고 있는 마크의 스테이션으로 도킹
            * 제어 방식 2가지
                * 프레임 중앙 세로선으로 도형의 중앙점이 일치해야 함
                * 스테이션 전방 위치(배가 이동해온 좌표)에서부터 스테이션 방향으로 반직선을 그리고, 그 위로 보트의 추종점을 찍어나감
"""

import math
import os
import sys

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, UInt16
from visualization_msgs.msg import MarkerArray

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import control.control_tools as control
import dock.dock_visualize_fix as dock_visualize
import dock.mark_detect_fix as mark_detect
import utils.gnss_converter as gc
import utils.obstacle_avoidance as oa
from ku2023.msg import ObstacleList
from utils.tools import *


class Docking:

    def __init__(self):
        # subscribers
        self.diff = [0, 0]
        self.heading_sub = rospy.Subscriber("/heading", Float64, self.heading_callback, queue_size=1)
        self.enu_pos_sub = rospy.Subscriber("/enu_position", Point, self.boat_position_callback, queue_size=1)
        self.obstacle_sub = rospy.Subscriber("/obstacles", ObstacleList, self.obstacle_callback, queue_size=1)
        self.cam_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.cam_callback)
        self.bridge = CvBridge()

        # publishers
        self.servo_pub = rospy.Publisher("/servo", UInt16, queue_size=0)
        self.visual_rviz_pub = rospy.Publisher("/visual_rviz", MarkerArray, queue_size=0)
        self.thrusterL_pub = rospy.Publisher("/thrusterL", UInt16, queue_size=0)
        self.thrusterR_pub = rospy.Publisher("/thrusterR", UInt16, queue_size=0)
        # target info
        self.target_shape = rospy.get_param("target_shape")
        self.target_color = rospy.get_param("target_color")
        target_color_range = rospy.get_param("color_range")[self.target_color]

        # ranges, limits
        self.angle_range = rospy.get_param("angle_range")  # 배열! [min, max]
        self.servo_range = rospy.get_param("servo_range")  # 배열! [min, max]
        self.servo_middle = (self.servo_range[0] + self.servo_range[1]) / 2
        self.arrival_range = rospy.get_param("arrival_range")  # 도착여부 판단할 범위
        self.color_range = np.array(
            [
                [
                    target_color_range["color1_lower"],
                    target_color_range["color2_lower"],
                    target_color_range["color3_lower"],
                ],
                [
                    target_color_range["color1_upper"],
                    target_color_range["color2_upper"],
                    target_color_range["color3_upper"],
                ],
            ]
        )
        self.ref_dir_range = rospy.get_param("ref_dir_range")  # 좌우로 얼마나 각도 허용할 건가
        self.arrival_target_area = rospy.get_param("arrival_target_area")  # 도착이라 판단할 타겟 도형의 넓이
        self.mark_detect_area = rospy.get_param("mark_detect_area")  # 도형이 검출될 최소 넓이
        self.target_detect_area = rospy.get_param("target_detect_area")  # 타겟이라고 인정할 최소 넓이
        self.station_dir = rospy.get_param("station_dir")

        # coordinates
        self.enterence_x, self.enterence_y, _ = gc.enu_convert(rospy.get_param("docking_enterence"))
        self.station1_x, self.station1_y, _ = gc.enu_convert(rospy.get_param("station1"))
        self.station2_x, self.station2_y, _ = gc.enu_convert(rospy.get_param("station2"))
        self.station3_x, self.station3_y, _ = gc.enu_convert(rospy.get_param("station3"))
        self.docking_end_x, self.docking_end_y, _ = gc.enu_convert(rospy.get_param("docking_end"))

        self.boat_x, self.boat_y = 0, 0
        self.waypoints = [
            [self.enterence_x, self.enterence_y],
            [self.station1_x, self.station1_y],
            [self.station2_x, self.station2_y],
            [self.station3_x, self.station3_y],
            [self.docking_end_x, self.docking_end_y]
        ]
        self.station_vec_ends = control.calc_station_vec_end(station_dir=self.station_dir, stations=self.waypoints[1:])
        self.trajectory = []

        #self.diff = [-1.7, -0.4]
        # data
        self.psi = 0  # 자북과 선수 사이 각
        self.psi_goal = 0
        self.psi_desire = 0
        self.raw_img = np.zeros((480, 640, 3), dtype=np.uint8)  # row, col, channel
        self.hsv_img = np.zeros((480, 640), dtype=np.uint8)
        self.shape_img = np.zeros((480, 640, 3), dtype=np.uint8)
        self.obstacles = []
        self.mark_area = 0
        self.distance_to_point = 0  # 특정 지점으로부터 배까지의 거리

        # count
        ## (state 4에서) '이 시간동안(횟수)' 정지(약한 후진)하고 그 뒤에 헤딩 돌릴 것.
        self.stop_time = rospy.get_param("stop_time")
        ## (state 4에서) 몇 번 정지 신호를 보냈는가?
        self.stop_cnt = 2
        ## (state 5에서) '이만큼(횟수)' 기다리며 얼마나 발견하나 횟수를 셈
        self.target_detect_time = rospy.get_param("target_detect_time")
        ## (state 5에서) 그만큼 중 얼마나 기다렸는가
        self.mark_check_cnt = 0
        ## (state 5에서) 그만큼 기다리는 동안 '얼마나' 발견해야 발견이라 하겠는가
        self.target_detect_cnt = rospy.get_param("target_detect_cnt")
        ## (state 5에서) 그만큼 기다리는 동안 '몇 번' 타겟을 발견했는가
        self.detected_cnt = 0

        # constants
        self.angle_alpha = rospy.get_param("angle_alpha")
        self.pixel_alpha = rospy.get_param("pixel_alpha")

        # ON/OFF
        self.draw_contour = rospy.get_param("draw_contour")

        # speed
        # self.thruster_auto = rospy.get_param("thruster_auto")
        # self.thruster_station = rospy.get_param("thruster_station")
        # self.thruster_left = rospy.get_param("thruster_left")
        # self.thruster_right = rospy.get_param("thruster_right")
        # self.thruster_stop = rospy.get_param("thruster_stop")
        # self.thruster_back = rospy.get_param("thruster_back")

        # other settings
        self.filter_queue_size = rospy.get_param("filter_queue_size")
        self.span_angle = rospy.get_param("span_angle")
        self.ob_angle_range = rospy.get_param("ob_angle_range")
        self.ob_dist_range = rospy.get_param("ob_dist_range")
        self.find_time = rospy.get_param("find_time") #(edit) 파라미터에 추가하기

        self.thruster_speed_L= 1500
        self.thruster_speed_R= 1500
        # current status
        self.state = 0
        # 0: 장애물 회피
        # 1: 스테이션1로 이동 중
        # 2: 스테이션2로 이동 중
        # 3: 스테이션3로 이동 중
        # 4: 헤딩 맞추는 중
        # 5: 타겟 스캔 중
        # 6: 스테이션 진입 중
        # 7: 끝. 정지
        self.target = []  # [area, center_col(pixel)]. 딕셔너리로 선언하는 쪽이 더 쉬울 듯
        self.target_found = False
        self.next_to_visit = 4  # 다음에 방문해야 할 스테이션 번호. state 시작을 1로할거면 1로
        self.imu_fix = 0
        # controller
        cv2.namedWindow("controller")
        # cv2.createTrackbar("color1 min", "controller", self.color_range[0][0], 180, lambda x: x)
        # cv2.createTrackbar("color1 max", "controller", self.color_range[1][0], 180, lambda x: x)
        # cv2.createTrackbar("color2 min", "controller", self.color_range[0][1], 255, lambda x: x)
        # cv2.createTrackbar("color2 max", "controller", self.color_range[1][1], 255, lambda x: x)
        # cv2.createTrackbar("color3 min", "controller", self.color_range[0][2], 255, lambda x: x)
        # cv2.createTrackbar("color3 max", "controller", self.color_range[1][2], 255, lambda x: x)
        # cv2.createTrackbar(
        #     "mark_detect_area", "controller", self.mark_detect_area, 3000, lambda x: x
        # )
        # cv2.createTrackbar(
        #     "target_detect_area", "controller", self.target_detect_area, 3000, lambda x: x
        # )
        # cv2.createTrackbar(
        #     "arrival_target_area", "controller", self.arrival_target_area, 10000, lambda x: x
        # )

    def heading_callback(self, msg):
        # self.psi = msg.data  # [degree]
        self.psi = -(msg.data - self.imu_fix)
        
    def boat_position_callback(self, msg):
        self.boat_x = msg.x + self.diff[0]
        self.boat_y = msg.y + self.diff[1]

    def obstacle_callback(self, msg):
        self.obstacles = msg.obstacle
        # [msg.obstacle.begin.x, msg.obstacle.begin.y, msg.obstacle.end.x, msg.obstacle.end.y]

    def cam_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if img.size == (640 * 480 * 3):
            self.raw_img = img
            self.color_check = cv2.GaussianBlur(np.uint8([[self.raw_img[240, 320]]]), (5, 5), 0)
            # self.color_check = cv2.cvtColor(self.color_check, cv2.COLOR_BGR2HSV)

            check_img = img
            check_img = cv2.circle(check_img, (320, 240), 3, (255,0,0), 2)
            cv2.imshow("img", check_img)
            # [120  98 189]
            #191, 115, 116
            #188 115 115
            pass

    def get_trackbar_pos(self):
        """get trackbar poses and set each values"""
        # self.color_range[0][0] = cv2.getTrackbarPos("color1 min", "controller")
        # self.color_range[1][0] = cv2.getTrackbarPos("color1 max", "controller")
        # self.color_range[0][1] = cv2.getTrackbarPos("color2 min", "controller")
        # self.color_range[1][1] = cv2.getTrackbarPos("color2 max", "controller")
        # self.color_range[0][2] = cv2.getTrackbarPos("color3 min", "controller")
        # self.color_range[1][2] = cv2.getTrackbarPos("color3 max", "controller")
        # self.mark_detect_area = cv2.getTrackbarPos("mark_detect_area", "controller")
        # self.target_detect_area = cv2.getTrackbarPos("target_detect_area", "controller")
        # self.arrival_target_area = cv2.getTrackbarPos("arrival_target_area", "controller")

    def is_all_connected(self):
        """make sure all subscribers(nodes) are connected to this node

        Returns:
            bool : True(if all connected) / False(not ALL connected yet)
        """
        not_connected = ""
        # if self.heading_sub.get_num_connections() == 0:
        #     not_connected += "\theadingCalculator"

        # if self.enu_pos_sub.get_num_connections() == 0:
        #     not_connected += "\tgnssConverter"

        # if self.obstacle_sub.get_num_connections() == 0:
        #     not_connected += "\tlidarConverter"

        if not self.raw_img.size == (640 * 480 * 3):
            not_connected += "\tCamera"

        if len(not_connected) == 0:
            return True
        else:
            print("\nWaiting >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
            print(not_connected)
            print("\n")
            return False

    def calc_distance(self, point):
        self.distance_to_point = math.hypot(self.boat_x - point[0], self.boat_y - point[1])

        return self.distance_to_point <= self.arrival_range
    def calc_distance2(self, point):
        self.distance_to_point = math.hypot(self.boat_x - point[0], self.boat_y - point[1])

        return self.distance_to_point <= self.arrival_range


    def check_state(self):
        change_state = False #현재 보트의 도킹 순서
        if self.state == 0: #도킹 포인트 지점으로 이동
            # 변경지점 도착 여부 판단1111
            change_state = self.calc_distance(self.waypoints[0])
        elif self.state == 3:
            if self.mark_check_cnt >= self.target_detect_time:
                change_state = True
                self.mark_check_cnt = 0
                self.detected_cnt = 0
            else:
                change_state = False
        elif self.state == 4: # 도형 쪽으로 헤딩값 수정
            change_state = self.check_heading()
        elif self.state == 5:
            if self.mark_check_cnt >= self.target_detect_time:
                change_state = True
                self.mark_check_cnt = 0
                self.detected_cnt = 0
            else:
                change_state = False
                
        elif self.state == 6:
            # 도킹 완료했는지 확인
            change_state = self.check_docked()

        if change_state:
            print("")
            print("{:=^70}".format(" Change State "))
            print("")
            if self.state == 0:  # 도킹 위치로 도달했을 때
                self.next_to_visit = 4 # 바로 5로 변경
                self.state = 4
                self.stop_cnt = 0
            elif self.state == 5:
                if self.target_found:
                    self.state += 1
                else:
                    self.state = self.next_to_visit
                    pass
            else:
                self.state += 1

            return True
        else:
            return False

    def check_heading(self):
        """선수각이 스테이션 방향을 향하고 있는지(허용 범위 내로 들어왔는지) 체크"""
        angle_to_station = self.station_dir - self.psi
        angle_to_station = rearrange_angle(angle_to_station)

        return abs(angle_to_station) <= self.ref_dir_range

    def check_target(self, return_target=False):
        """표지를 인식하고 타겟이면 타겟 정보를 반환

        Args:
            return_target (bool): target 정보를 리턴할 것인지(True), Bool 정보를 리턴할 것인지(False)

        Returns:
            True/False (bool): 타겟을 찾음(True), 찾지 못함(False)
            target (list): 타겟을 찾았고, [넓이, 중앙지점] 정보를 담고 있음
        """
        # self.show_window()
        preprocessed = mark_detect.preprocess_image(self.raw_img, blur=True , brightness=False, hsv=False)

        
        if self.target_color == 'black2':
          self.hsv_img = mark_detect.select_color(~preprocessed, self.color_range)  # 원하는 색만 필터링
        else:
            self.hsv_img = mark_detect.select_color(preprocessed, self.color_range)  # 원하는 색만 필터링


        
        #self.hsv_img = mark_detect.select_color(preprocessed, self.color_range)  # 원하는 색만 필터링

        target, self.shape_img, self.mark_area = mark_detect.detect_target(
            self.hsv_img,
            self.target_shape,
            self.mark_detect_area,
            self.target_detect_area,
            self.draw_contour,
        )  # target = [area, center_col] 형태로 타겟의 정보를 받음

        if return_target == True:
            return target
        else:
            return False if len(target) == 0 else True
        
    def check_target_state_zero(self, return_target=False):
        """표지를 인식하고 타겟이면 타겟 정보를 반환

        Args:
            return_target (bool): target 정보를 리턴할 것인지(True), Bool 정보를 리턴할 것인지(False)

        Returns:
            True/False (bool): 타겟을 찾음(True), 찾지 못함(False)
            target (list): 타겟을 찾았고, [넓이, 중앙지점] 정보를 담고 있음
        """

        preprocessed = mark_detect.preprocess_image(self.raw_img, blur=True , brightness=False, hsv=False)


        self.hsv_img = preprocessed
        target, self.shape_img, self.mark_area = mark_detect.detect_target_state_zero(
            self.hsv_img,
            self.target_shape,
            self.mark_detect_area,
            self.target_detect_area,
            self.draw_contour,
        )  # target = [area, center_col] 형태로 타겟의 정보를 받음
        cv2.imshow("shape_img", self.shape_img)
        if return_target == True:
            return target
        else:
            return False if len(target) == 0 else True
        
    def check_docked(self):
        """스테이션에 도크되었는지 확인

        도킹 모드(6번)에서 마크를 탐지했을 때, 탐지가 되었다면 마크의 넓이를 기준으로 판단.
        탐지가 되지 않았다면 도크되지 않았다고 판단.

        Returns:
            True/False: 도킹 끝(True), 아직 안 끝남(False)
        """
        if self.calc_distance2(self.waypoints[4]):# (edit) clac_distance2 수정 필요 coordinates.yaml 수정필요 param 수정 필요
            return True
        else:
            return False



        if len(self.target) != 0:
            return self.target[0] >= self.arrival_target_area
        else:
            return False

    def calc_psi_goal(self):
        """다음 목표지점으로 가기 위한 선수 회전 각도를 계산"""
        psi_goal = (
            math.degrees(
                math.atan2(
                    self.waypoints[self.state][1] - self.boat_y,
                    self.waypoints[self.state][0] - self.boat_x,
                )
            )
            - self.psi
        )
        self.psi_goal = rearrange_angle(psi_goal)

    def print_status(self, error_angle, u_servo, u_thruster):
        state_str = [
            "Avoiding Obstacles",
            "Going to Station #1",
            "Going to Station #2",
            "Going to Station #3",
            "Rotating Heading",
            "Detecting Target",
            "Docking",
            "End",
        ]
        print("")
        print("({:>4.2f}, {:>4.2f})".format(self.boat_x, self.boat_y))
        print("State: # {} - {}  angle = {}".format(str(self.state), state_str[self.state] , error_angle))
        print("")

        if self.state == 6:
            error_angle_dir_str = "( Right )" if error_angle > 0 else "(  Left )"
            print("Mark Area    : {:>7,.0f} / {:>7,.0f}".format(self.mark_area, self.mark_detect_area))
            print(
                "Target Area  : {:>7,.0f} / {:>7,.0f} ({:>5})".format(
                    self.target[0] if len(self.target) != 0 else 0,
                    self.target_detect_area,
                    "Found" if len(self.target) != 0 else "None",
                )
            )
            print(
                "Arrival Area : {:>7,.0f} / {:>7,.0f}".format(
                    self.target[0] if len(self.target) != 0 else 0,
                    self.arrival_target_area,
                )
            )
            print("")
            print("mid - {:>6} = {:>11} {:->4} {:>11}".format("target", "error_pixel", ">", "error_angle"))
            print(
                "320 - {:>6,.0f} = {:>11,.0f} {:>4} {:>11.2f} {:>9}".format(
                    self.target[1] if len(self.target) != 0 else 0,
                    320 - self.target[1] if len(self.target) != 0 else 0,
                    "",
                    error_angle,
                    error_angle_dir_str,
                )
            )
            print("")

        if self.state in [0, 1, 2, 3]:
            psi_goal_dir_str = "[   | * ]" if self.psi_goal > 0 else "[ * |   ]"
            error_angle_dir_str = "( Right )" if error_angle > 0 else "(  Left )"
            if u_servo > self.servo_middle:
                servo_value_str = str("<" * int(((self.servo_middle - u_servo) // 5)))  # go left
            else:
                servo_value_str = str(">" * int(((self.servo_middle - u_servo) // 5)))   # go right
            print("{:^9}   {:^8} - {:^8} = {:^8} {:->9} {:^5}".format("goal", "desire", "psi", "error", ">", "servo"))
            print(
                "{:>9}   {:>8.2f} - {:>8.2f} = {:>8.2f} {:>9} {:>5} ( {:^5} )".format(
                    psi_goal_dir_str,
                    self.psi_desire,
                    self.psi,
                    error_angle,
                    error_angle_dir_str,
                    u_servo,
                    servo_value_str,
                )
            )
            print("")
            print("{:<9} : {:6.2f} m".format("distance", self.distance_to_point))

        elif self.state == 4:
            error_angle_dir_str = "( Right )" if error_angle > 0 else "(  Left )"

            if u_servo > self.servo_middle:
                servo_value_str = "<" * ((self.servo_middle - u_servo) // 5)  # go left
            else:
                servo_value_str = ">" * ((self.servo_middle - u_servo) // 5)  # go right

            if self.stop_cnt >= self.stop_time:
                print("Rotating Heading >>>>")
            else:
                print("Stopping Boat >>>>>>> {:>2d} / {:>2d}".format(self.stop_cnt, self.stop_time))
            print("")
            print("{:^8} - {:^8} = {:^8} {:->9} {:^5}".format("desire", "psi", "error", ">", "servo"))
            print(
                "{:>8.2f} - {:>8.2f} = {:>8.2f} {:>9} {:>5} ( {:^5} )".format(
                    self.psi_desire,
                    self.psi,
                    error_angle,
                    error_angle_dir_str,
                    u_servo,
                    servo_value_str,
                )
            )

        elif self.state == 5:
            print("Target Shape : {} | Color : {}".format(self.target_shape, self.target_color))
            print("Waiting..... : {:>4d} / {:>4d}".format(self.mark_check_cnt, self.target_detect_time))
            print("Target Cnt   : {:>4d} / {:>4d}".format(self.detected_cnt, self.target_detect_cnt))
            print("")
            print("Mark Area    : {:>7,.0f} / {:>7,.0f}".format(self.mark_area, self.mark_detect_area))

        print("")
        print("ThrusterL  : {}".format(self.thruster_speed_L))
        print("ThrusterR  : {}".format(self.thruster_speed_R))

        print("")
        try:
            print("color : {}".format(self.color_check))
        except:
            pass
        print("\n\n\n\n")

        print("-" * 70)

    def show_window(self):
        self.get_trackbar_pos()
        cv2.moveWindow("controller", 0, 0)
        if self.state in [5, 6]:
            raw_img = cv2.resize(self.raw_img, dsize=(0, 0), fx=0.5, fy=0.5)  # 카메라 데이터 원본
            hsv_img = cv2.resize(self.hsv_img, dsize=(0, 0), fx=0.5, fy=0.5)  # 색 추출 결과
            hsv_img = cv2.cvtColor(hsv_img, cv2.COLOR_GRAY2BGR)
            col1 = np.vstack([raw_img, hsv_img])
            col2 = cv2.resize(self.shape_img, dsize=(0, 0), fx=0.9, fy=1.0)  # 타겟 검출 결과
            show_img = np.hstack([col1, col2])
            cv2.imshow("controller", show_img)
        else:
            raw_img = cv2.resize(self.raw_img, dsize=(0, 0), fx=0.5, fy=0.5)
            cv2.imshow("controller", raw_img)


def main():
    rospy.init_node("Docking", anonymous=True)
    docking = Docking()
    print_cnt = 0
    rate = rospy.Rate(10)
    error_angle = 0
    u_thruster = True
    docking.thruster_speed_L= 1500
    docking.thruster_speed_R= 1500
    start_straight = 0
    imu_fix = True
    in_line = False
    #docking_state = 5
    while not docking.is_all_connected():
        rospy.sleep(0.2)
    print("\n<<<<<<<<<<<<<<<<<<< All Connected !")
        
    while not rospy.is_shutdown():
        docking.trajectory.append([docking.boat_x, docking.boat_y])  # 이동 경로 추가
        docking.show_window()
        # print(docking.state)
        change_state = docking.check_state()  # 현재 상태 점검 및 변경
        # 일부 변수 초기화
        inrange_obstacles = []
        danger_angles = []
        forward_point = []
        if imu_fix:
            docking.imu_fix = docking.psi_goal
            imu_fix= False
        # 특정 지점으로 이동해야 하는 경우, psi_goal 계산
        if docking.state in [0]:
            docking.calc_psi_goal()

        # 도킹 완료됨. 정지 및 끝내기
        if docking.state == 7:
            # docking.servo_pub.publish(docking.servo_middle)
            # docking.thruster_pub.publish(1500)
            docking.thrusterL_pub.publish(1500)
            docking.thrusterR_pub.publish(1500)
            u_thruster = False
            print(">>>>>>>>>>>>>> Finished <<<<<<<<<<<<<<")
            return

        # 시작점으로 이동하며 장애물 회피
        elif docking.state == 0:
            # 장애물 탐지. 범위 내에 있는 장애물을 필터링하고, 장애물이 있는 각도 리스트를 만듦
            inrange_obstacles, danger_angles = oa.ob_filtering(
                obstacles=docking.obstacles,
                dist_to_goal=docking.distance_to_point,
                angle_to_goal=docking.psi_goal,
                span_angle=docking.span_angle,
                angle_range=docking.ob_angle_range,
                distance_range=docking.ob_dist_range,
            )
            # print("Obstacle : {:2d} / {:2d}".format(len(inrange_obstacles), len(docking.obstacles)))

            # 목표각과 현 헤딩 사이 상대적 각도 계산. 선박고정좌표계로 '가야 할 각도'에 해당
            error_angle = oa.calc_desire_angle(
                danger_angles=danger_angles,
                angle_to_goal=docking.psi_goal,
                angle_range=docking.ob_angle_range,
            )
            # 선속 결정
            u_thruster = True

            docking.mark_check_cnt += 1  # 탐색 횟수 1회 증가
            detected = docking.check_target_state_zero()  # 타겟이 탐지 되었는가?

            # if detected:
            #     docking.detected_cnt += 1  # 타겟 탐지 횟수 1회 증가

            # # 지정된 횟수만큼 탐색 실시해봄
            # if docking.mark_check_cnt >= docking.target_detect_time:
            #     # 타겟 마크가 충분히 많이 검출됨
            #     if docking.detected_cnt >= docking.target_detect_cnt:
            #         docking.target = docking.check_target(return_target=True)  # 타겟 정보
            #         docking.target_found = True  # 타겟 발견 플래그
            #     else:
            #         docking.target = []  # 타겟 정보 초기화(못 찾음)
            #         docking.target_found = False  # 타겟 미발견 플래그
            #         rospy.sleep(0.2)
            #         docking.thrusterL_pub.publish(1550)
            #         docking.thrusterR_pub.publish(1550)
            #         rospy.sleep(1)

        #     # 아직 충분히 탐색하기 전
        #     else:
        #         docking.target_found = False  # 타겟 미발견 플래그

        # 헤딩 돌리기: 우선 일정 시간동안 정지한 후 헤딩 회전
        elif docking.state == 4:
            # 정지 종료, 헤딩 돌리기
            if docking.stop_cnt >= docking.stop_time:
                u_thruster = True
                error_angle = docking.station_dir - docking.psi
                #u_thruster = docking.thruster_rotate
            # 정지 중
            else:
                docking.stop_cnt += 1  # 몇 번 루프를 돌 동안 정지
                rospy.sleep(1)
                u_thruster = False

        # 표지 판단
        elif docking.state == 5:
            docking.mark_check_cnt += 1  # 탐색 횟수 1회 증가
            detected = docking.check_target()  # 타겟이 탐지 되었는가?

            if detected:
                docking.detected_cnt += 1  # 타겟 탐지 횟수 1회 증가

            # 지정된 횟수만큼 탐색 실시해봄
            if docking.mark_check_cnt >= docking.target_detect_time:
                # 타겟 마크가 충분히 많이 검출됨
                if docking.detected_cnt >= docking.target_detect_cnt:
                    docking.target = docking.check_target(return_target=True)  # 타겟 정보
                    docking.target_found = True  # 타겟 발견 플래그
                # 타겟 마크가 충분히 검출되지 않아 미검출로 판단
                else:
                    docking.target = []  # 타겟 정보 초기화(못 찾음)
                    docking.target_found = False  # 타겟 미발견 플래그
                    rospy.sleep(0.2)
                    docking.thrusterL_pub.publish(1550)
                    docking.thrusterR_pub.publish(1550)
                    rospy.sleep(1)

            # 아직 충분히 탐색하기 전
            else:
                docking.target_found = False  # 타겟 미발견 플래그

            # 에러각 계산 방식 (1)
            error_angle = docking.station_dir - docking.psi

            error_angle = rearrange_angle(error_angle)
            u_thruster = False

        # 스테이션 진입
        elif docking.state == 6:
            # 타겟 검출 for 도킹 완료 여부 판단 / 중앙 지점 추종
            docking.target = docking.check_target(return_target=True)
            u_thruster = True

            """
                Version1
                
                화면을 4분할로 나눠서 첫번째 화면에 이미지가 들어가면 좌회전 네번째 화면에 이미지가 들어가면 우회전 하도록 설정

            """
            try:
                a, b = docking.hsv_img.shape
                two_divisions = b / 2
                four_divisions = two_divisions / 2
                first_divisions = two_divisions - four_divisions
                last_divisions = two_divisions + four_divisions
                if in_line == False:
                    if len(docking.target) == 0:
                        #start_straight += 1
                        #docking.stop_cnt += 1  # 몇 번 루프를 돌 동안 정지
                        if start_straight < docking.find_time:
                            error_angle = docking.station_dir - docking.psi
                            error_angle = rearrange_angle(error_angle)
                            #앵글 각도 맞추기
                            print(start_straight)

                        else:
                            print("line진입 직진")
                            error_angle = 0
                            u_thruster = True
                            in_line = True                    
                    elif first_divisions > docking.target[2]:
                        #좌회전 선회
                        error_angle = -6.0
                    elif last_divisions < docking.target[2]:
                        #우회전 선회
                        error_angle = 6.0
                    else:
                        #직진 들어왔다고 판단 직진 쓰러스트
                        error_angle = 0
                        #직진
                else:
                    pass
            except:
                pass

            error_angle = rearrange_angle(error_angle)
        if u_thruster == True:
            # if error_angle>-0.9 and error_angle <-0.5: #go to light little
            #     thruster_speed_L= 1500
            #     thruster_speed_R= 1550
            # elif error_angle<0.9 and error_angle>0.5: #go to left little
            #     thruster_speed_L= 1550
            #     thruster_speed_R= 1500
            if error_angle < -5.0:
                #우회전
                docking.thruster_speed_L= 1650
                docking.thruster_speed_R= 1400
                pass
            elif error_angle > 5.0:
                #좌회전
                docking.thruster_speed_L= 1400
                docking.thruster_speed_R= 1650
            else:
                docking.thruster_speed_L=1550
                docking.thruster_speed_R=1550
                pass
        else:
            #정지
            docking.thruster_speed_L=1500
            docking.thruster_speed_R=1500
            #1초 정도 정지 후 
            # 선속 결정
            #u_thruster = docking.thruster_station

        docking.thrusterR_pub.publish(docking.thruster_speed_L)
        docking.thrusterL_pub.publish(docking.thruster_speed_R)

        # 각 모드에서 계산한 error_angle을 바탕으로 월드좌표계로 '가야 할 각도'를 계산함
        docking.psi_desire = rearrange_angle(docking.psi + error_angle)

        # 각 모드에서 계산한 결과로 서보모터 제어값을 결정함
        u_servo = control.degree_to_servo(
            error_angle=error_angle,
            angle_alpha=docking.angle_alpha,
            angle_range=docking.angle_range,
            servo_range=docking.servo_range,
        )

        # 터미널 프린트 주기 설정 및 현 상황 출력
        if print_cnt > 1:
            docking.print_status(error_angle, u_servo, u_thruster)
            print_cnt = 0
        else:
            print_cnt += 1

        # 시각화
        all_markers = dock_visualize.visualize(
            dc=docking,
            inrange_obstacles=inrange_obstacles,
            danger_angels=danger_angles,
        )
        docking.visual_rviz_pub.publish(all_markers)

        if cv2.waitKey(1) == 27:
            cv2.destroyAllWindows()
            break


def docking_part():
    docking = Docking()
    print_cnt = 0
    error_angle = 0
    u_thruster = True
    docking.thruster_speed_L= 1500
    docking.thruster_speed_R= 1500
    start_straight = 0
    in_line = False
    #docking_state = 5
    while not rospy.is_shutdown():
        docking.trajectory.append([docking.boat_x, docking.boat_y])  # 이동 경로 추가
        docking.show_window()
        change_state = docking.check_state()  # 현재 상태 점검 및 변경
        # 일부 변수 초기화
        inrange_obstacles = []
        danger_angles = []
        forward_point = []

        # 특정 지점으로 이동해야 하는 경우, psi_goal 계산
        if docking.state in [0]:
            docking.calc_psi_goal()

        # 도킹 완료됨. 정지 및 끝내기
        if docking.state == 7:
            docking.thrusterL_pub.publish(1500)
            docking.thrusterR_pub.publish(1500)
            u_thruster = False
            print(">>>>>>>>>>>>>> Finished <<<<<<<<<<<<<<")
            return

        # 시작점으로 이동하며 장애물 회피
        elif docking.state == 0:
            # 장애물 탐지. 범위 내에 있는 장애물을 필터링하고, 장애물이 있는 각도 리스트를 만듦
            inrange_obstacles, danger_angles = oa.ob_filtering(
                obstacles=docking.obstacles,
                dist_to_goal=docking.distance_to_point,
                angle_to_goal=docking.psi_goal,
                span_angle=docking.span_angle,
                angle_range=docking.ob_angle_range,
                distance_range=docking.ob_dist_range,
            )
            # print("Obstacle : {:2d} / {:2d}".format(len(inrange_obstacles), len(docking.obstacles)))

            # 목표각과 현 헤딩 사이 상대적 각도 계산. 선박고정좌표계로 '가야 할 각도'에 해당
            error_angle = oa.calc_desire_angle(
                danger_angles=danger_angles,
                angle_to_goal=docking.psi_goal,
                angle_range=docking.ob_angle_range,
            )
            # 선속 결정
            u_thruster = True

        # 헤딩 돌리기: 우선 일정 시간동안 정지한 후 헤딩 회전
        elif docking.state == 4:
            # 정지 종료, 헤딩 돌리기
            if docking.stop_cnt >= docking.stop_time:
                u_thruster = True
                error_angle = docking.station_dir - docking.psi
            # 정지 중
            else:
                docking.stop_cnt += 1  # 몇 번 루프를 돌 동안 정지
                rospy.sleep(1)
                u_thruster = False

        # 표지 판단
        elif docking.state == 5:
            docking.mark_check_cnt += 1  # 탐색 횟수 1회 증가
            detected = docking.check_target()  # 타겟이 탐지 되었는가?

            if detected:
                docking.detected_cnt += 1  # 타겟 탐지 횟수 1회 증가

            # 지정된 횟수만큼 탐색 실시해봄
            if docking.mark_check_cnt >= docking.target_detect_time:
                # 타겟 마크가 충분히 많이 검출됨
                if docking.detected_cnt >= docking.target_detect_cnt:
                    docking.target = docking.check_target(return_target=True)  # 타겟 정보
                    docking.target_found = True  # 타겟 발견 플래그
                # 타겟 마크가 충분히 검출되지 않아 미검출로 판단
                else:
                    docking.target = []  # 타겟 정보 초기화(못 찾음)
                    docking.target_found = False  # 타겟 미발견 플래그
                    rospy.sleep(0.2)
                    docking.thrusterL_pub.publish(1550)
                    docking.thrusterR_pub.publish(1550)
                    rospy.sleep(1)

            # 아직 충분히 탐색하기 전
            else:
                docking.target_found = False  # 타겟 미발견 플래그

            # 에러각 계산 방식 (1)
            error_angle = docking.station_dir - docking.psi
            error_angle = rearrange_angle(error_angle)
            u_thruster = False

        # 스테이션 진입
        elif docking.state == 6:
            # 타겟 검출 for 도킹 완료 여부 판단 / 중앙 지점 추종
            docking.target = docking.check_target(return_target=True)
            u_thruster = True
        
            """
                Version1
                
                화면을 4분할로 나눠서 첫번째 화면에 이미지가 들어가면 좌회전 네번째 화면에 이미지가 들어가면 우회전 하도록 설정

            """
            try:
                a, b = docking.hsv_img.shape
                two_divisions = b / 2
                four_divisions = two_divisions / 2
                first_divisions = two_divisions - four_divisions
                last_divisions = two_divisions + four_divisions
                if in_line == False:
                    if len(docking.target) == 0:
                            #start_straight += 1
                            #docking.stop_cnt += 1  # 몇 번 루프를 돌 동안 정지
                            if start_straight < docking.find_time:
                                error_angle = docking.station_dir - docking.psi
                                error_angle = rearrange_angle(error_angle)
                                #앵글 각도 맞추기
    
                            else:
                                error_angle = 0
                                u_thruster = True
                                in_line = True                    
                    elif first_divisions > docking.target[2]:
                        #좌회전 선회
                        error_angle = -6.0
                    elif last_divisions < docking.target[2]:
                        #우회전 선회
                        error_angle = 6.0
                    else:
                        #직진 들어왔다고 판단 직진 쓰러스트
                        error_angle = 0
                        #직진
                else:
                    pass
            except:
                pass
    
            error_angle = rearrange_angle(error_angle)
        if u_thruster == True:
            # if error_angle>-0.9 and error_angle <-0.5: #go to light little
            #     thruster_speed_L= 1500
            #     thruster_speed_R= 1550
            # elif error_angle<0.9 and error_angle>0.5: #go to left little
            #     thruster_speed_L= 1550
            #     thruster_speed_R= 1500
            if error_angle < -5.0:
                #우회전
                docking.thruster_speed_L= 1650
                docking.thruster_speed_R= 1400
                pass
            elif error_angle > 5.0:
                #좌회전
                docking.thruster_speed_L= 1400
                docking.thruster_speed_R= 1650
            else:
                docking.thruster_speed_L=1550
                docking.thruster_speed_R=1550
                pass
        else:
            #정지
            docking.thruster_speed_L=1500
            docking.thruster_speed_R=1500
            #1초 정도 정지 후 
            # 선속 결정
            #u_thruster = docking.thruster_station

        docking.thrusterR_pub.publish(docking.thruster_speed_L)
        docking.thrusterL_pub.publish(docking.thruster_speed_R)

        # 각 모드에서 계산한 error_angle을 바탕으로 월드좌표계로 '가야 할 각도'를 계산함
        docking.psi_desire = rearrange_angle(docking.psi + error_angle)

        # 각 모드에서 계산한 결과로 서보모터 제어값을 결정함
        u_servo = control.degree_to_servo(
            error_angle=error_angle,
            angle_alpha=docking.angle_alpha,
            angle_range=docking.angle_range,
            servo_range=docking.servo_range,
        )

        # 터미널 프린트 주기 설정 및 현 상황 출력
        if print_cnt > 1:
            docking.print_status(error_angle, u_servo, u_thruster)
            print_cnt = 0
        else:
            print_cnt += 1

        # 시각화
        all_markers = dock_visualize.visualize(
            dc=docking,
            inrange_obstacles=inrange_obstacles,
            danger_angels=danger_angles,
        )
        docking.visual_rviz_pub.publish(all_markers)

        if cv2.waitKey(1) == 27:
            cv2.destroyAllWindows()
            break

if __name__ == "__main__":
    main()
