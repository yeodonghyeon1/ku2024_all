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
"""Script for autonomous(obstacle avoidance) misson"""

import math
import os
import sys
import rospy

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import autonomous_visualize as av
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan,Imu
from std_msgs.msg import Float64, UInt16
from visualization_msgs.msg import MarkerArray

import control.control_tools as control
import datatypes.point_class
import utils.gnss_converter as gc
import utils.obstacle_avoidance as oa
from ku2023.msg import ObstacleList
from utils.tools import *
import copy
import numpy




class Autonomous:
    def __init__(self):
        # coordinates
        # make waypoint
        self.waypoint_idx = 1

        self.remained_waypoints = {}  # 남은 waypoints. key는 waypoint 순서, value는 [x, y] 좌표
        self.gnss_waypoint = rospy.get_param("waypoints")  # GPS 형식의 전체 waypoints
        for idx, waypoint in enumerate(self.gnss_waypoint):
            n, e, _ = gc.enu_convert(waypoint)  # ENU 좌표계로 변환
            self.remained_waypoints[idx + 1] = [n, e]  # 축이 반대이므로 순서 바꿔 할당.
        self.waypoints = copy.deepcopy(self.remained_waypoints)
        self.boat_y, self.boat_x = 0, 0  # 배의 좌표
        self.goal_x = self.remained_waypoints[self.waypoint_idx][0]  # 다음 목표 x좌표
        self.goal_y = self.remained_waypoints[self.waypoint_idx][1]  # 다음 목표 y좌표
        self.trajectory = []  # 지금껏 이동한 궤적
        self.diff = [0, 0]
    
        # locations, coordinates
        # self.boat_x, self.boat_y = 0, 0  # 현재 보트 위치
        # self.goal_x, self.goal_y, _ = gc.enu_convert(rospy.get_param("autonomous_goal"))  # 목표점 위치
        # self.trajectory = []  # 지금까지 이동한 궤적
        boundary = rospy.get_param("boundary1")  # 경기장 꼭짓점
        self.boundary = []
        for p in boundary:
            self.boundary.append(list(gc.enu_convert(p)))
        # self.diff = [-1.7, -0.4]  # 현재 보트 위치 GPS 보정

        # directions
        self.heading_queue = []  # 헤딩을 필터링할 이동평균필터 큐
        self.psi = 0  # 자북과 heading의 각도(자북 우측 +, 좌측 -) [degree]
        self.psi_goal = 0  # 의미 변화함: 나로부터 goal이 떨어진 각도. (+)면 오른쪽, (-)면 왼쪽에 있음
        self.psi_desire = 0  # 이번에 가야 할 각도

        # obstacles
        self.ob_dist_range = rospy.get_param("ob_dist_range")  # 라이다로 장애물을 탐지할 최대거리
        self.ob_angle_range = rospy.get_param("ob_angle_range")  # 장애물 탐지 각도
        self.span_angle = rospy.get_param("span_angle")  # 장애물 양쪽에 더해줄 각도 여유분. 충돌 방지용
        self.input_points = []  # lidar raw data
        self.obstacles = []  # 장애물 전체
        self.inrange_obstacles = []  # 탐지 범위 내 장애물
        self.danger_angles = []  # 장애물이 존재하는 각도 리스트

        # distances
        self.goal_range = rospy.get_param("goal_range")  # 도착이라 판단할 거리(반지름)
        self.distance_to_goal = 100000  # 배~목적지 거리. max 연산이므로 큰 값을 초기 할당

        # control
        self.rotate_angle_range = rospy.get_param("rotate_angle_range")  # 회전할 각도 범위
        self.servo_range = rospy.get_param("servo_range")  # 서보모터 최대/최소값
        self.servo_middle = (self.servo_range[0] + self.servo_range[1]) / 2  # 서보모터 중앙값(전진)
        self.filter_queue = []  # 서보모터값을 필터링할 이동평균필터 큐
        # self.thruster_speed = rospy.get_param("thruster_speed")  # 쓰러스터 PWM 고정값 (선속)

        # other fixed values
        self.filter_queue_size = rospy.get_param("filter_queue_size")  # 이동평균필터 큐사이즈
        self.angle_alpha = rospy.get_param("angle_alpha")  # angle_to_servo 함수에서 사용할 상수

        # visualize
        self.show_raw_pcd = rospy.get_param("show_raw_pcd")  # 라이다 raw 데이터 보이기
        self.print_cnt = 0  # 출력 속도를 조정할 카운터.

        # subscribers
        self.heading_sub = rospy.Subscriber("/heading", Float64, self.heading_callback, queue_size=1)
        self.enu_pos_sub = rospy.Subscriber("/enu_position", Point, self.boat_position_callback, queue_size=1)
        self.obstacle_sub = rospy.Subscriber("/obstacles", ObstacleList, self.obstacle_callback, queue_size=1)
        
        #####edit
        self.imu_sub = rospy.Subscriber("/imu/data", Imu, self.IMU_callback, queue_size=1)
        #####

        if self.show_raw_pcd:
            self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)

        # publishers
        self.thrusterL_pub = rospy.Publisher("/thrusterL", UInt16, queue_size=0)
        self.thrusterR_pub = rospy.Publisher("/thrusterR", UInt16, queue_size=0)
        self.visual_rviz_pub = rospy.Publisher("/visual_rviz", MarkerArray, queue_size=0)
        self.imu_fix_pub = rospy.Publisher("/imu_fix", Float64, queue_size=1)

        # pre-setting
        self.arrival_check()  # 다음 목표까지 남은 거리

        # robot parameter
        self.max_speed = 2.23 # [m/s]
        self.min_speed = 0.1  # [m/s]
        self.max_yaw_rate = 60.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 2.5  # [m/ss]
        self.max_delta_yaw_rate = 60.0 * math.pi / 180.0  # [rad/ss]
        self.v_resolution = 0.05  # [m/s]  인식할수있는 최소 단위라고 보면 되나
        self.yaw_rate_resolution = 1* math.pi / 180.0  # [rad/s]  #4   8 
        self.dt = 0.1  # [s] Time tick for motion prediction 움직임 예측을 위한 시간단위, 움직임은 0.1단위로 이뤄짐
        self.predict_time = 1.5 # [s]
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked
        self.robot_type = 0  #    circle = 0  rectangle = 1
        self.v_x_gain = 0.5 # 0.45~1.0

    def scan_callback(self, msg): ##/scan 토픽 안에 range_min, range_max값이 있음??...?
        if not self.show_raw_pcd:
            return

        self.input_points = []
        phi = msg.angle_min  # 각 점의 각도 계산 위해 계속 누적해갈 각도
        for r in msg.ranges:
            if msg.range_min/2 <= r <= msg.range_max/2:
                p = datatypes.point_class.Point.polar_to_cartesian(r, phi)
                self.input_points.append(p)
            phi += msg.angle_increment

    def IMU_callback(self, imu):
        self.imu = imu

    def heading_callback(self, msg):
        """IMU 지자기 센서로 측정한 자북과 heading 사이각 콜백함수

        Args:
            msg (Float64) : heading. 0 = magnetic north, (+) = 0~180 deg to right, (-) = 0 ~ -180 deg to left

        Notes:
            * IMU의 예민성으로 인하여 heading 값에 noise가 있음. 따라서 이동평균필터를 적용함.
        """
        # self.psi = moving_avg_filter(
        #     self.heading_queue, self.filter_queue_size, msg.data
        # )  # [deg]
        self.psi = -msg.data

    def boat_position_callback(self, msg):
        """GPS로 측정한 배의 ENU 변환 좌표 콜백함수

        Args:
            msg (Point) : position of boat

        Note:
            * ENU좌표계로 변환되어 입력을 받는데, ENU좌표계와 x, y축이 반대임
            * 따라서 Point.x, Point.y가 각각 y, x가 됨
        """
        self.boat_x = msg.x + self.diff[0]
        self.boat_y = msg.y + self.diff[1]

    def obstacle_callback(self, msg):
        """lidar_converter에서 받아온 장애물 정보 저장

        Args:
            msg (ObstacleList) : list of obstacles(Obstacle object)

        Note:
            * 개당 [msg.obstacle.begin.x, msg.obstacle.begin.y, msg.obstacle.end.x, msg.obstacle.end.y]
        """
        self.obstacles = msg.obstacle

    def is_all_connected(self):
        """make sure all subscribers(nodes) are connected to this node

        Returns:
            bool : True(if all connected) / False(not ALL connected yet)

        Notes:
            * 방법1: `if self.heading_sub.get_num_connections() == 0` - 노드 연결 여부만 체크
            * 방법2: `rospy.wait_for_message("/heading", Float64)` - 값 들어올 때까지 무한정 대기
        """
        ## 방법1
        # not_connected = ""  # 아직 연결되지 않은 센서 목록
        # if self.heading_sub.get_num_connections() == 0:
        #     not_connected += "headingCalculator\t"
        # if self.enu_pos_sub.get_num_connections() == 0:
        #     not_connected += "gnssConverter\t"
        # if self.obstacle_sub.get_num_connections() == 0:
        #     not_connected += "lidarConverter\t"

        # if len(not_connected) == 0:
        #     return True  # 전부 연결됨
        # else:
        #     print("\nWaiting >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
        #     print(not_connected)
        #     print("\n")
        #     return False  # 아직 다 연결되지는 않음

        ## 방법2
        rospy.wait_for_message("/heading", Float64)
        print("\n{:><70}".format("heading_calculator Connected "))
        rospy.wait_for_message("/enu_position", Point)
        print("\n{:><70}".format("gnss_converter Connected "))
        rospy.wait_for_message("/obstacles", ObstacleList)
        print("\n{:><70}".format("lidar_converter Connected "))
        if self.show_raw_pcd:
            rospy.wait_for_message("/scan", LaserScan)
            print("\n{:><70}".format("LiDAR Connected "))
        return True

    def arrival_check(self):
        """calculate distance from boat to the next goal of current state

        Returns:
            bool : True (= arrived to the goal) / False
        """
        self.distance_to_goal = math.hypot(self.boat_x - self.goal_x, self.boat_y - self.goal_y)
        return self.distance_to_goal <= self.goal_range

    def print_status(self, error_angle, u_servo): 
        """print current state

        Args:
            error_angle (float) : angle between psi_desire and psi (heading to desire angle)
            u_servo (int) : servo moter publish value
        """
        print("")
        print("({:>4.2f}, {:>4.2f})".format(self.boat_x, self.boat_y))
        print("Obstacle  : {:2d} / {:2d}".format(len(self.inrange_obstacles), len(self.obstacles)))

        psi_goal_dir_str = "[   | * ]" if self.psi_goal > 0 else "[ * |   ]"
        error_angle_dir_str = "( Right )" if error_angle > 0 else "(  Left )"
        if u_servo > self.servo_middle:
            servo_value_str = "<" * ((self.servo_middle - u_servo) // 5)  # go left
        else:
            servo_value_str = ">" * ((self.servo_middle - u_servo) // 5)  # go right

        print("")
        print("{:^9}   {:^6} - {:^6} = {:^6} {:->9} {:^5}".format("goal", "desire", "psi", "error", ">", "servo"))
        print(
            "{:>9}   {:>6.2f} - {:>6.2f} = {:>6.2f} {:>9} {:>5} ( {:^5} )".format(
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
        print("{:<9} : {:6.2f} m".format("distance", self.distance_to_goal))
        print("")
        print("-" * 70)

    def set_next_goal(self):
        del self.remained_waypoints[self.waypoint_idx]
        self.waypoint_idx += 1

        if len(self.gnss_waypoint) + 1 == self.waypoint_idx:
            return

        self.goal_x = self.remained_waypoints[self.waypoint_idx][0]
        self.goal_y = self.remained_waypoints[self.waypoint_idx][1]


    def dwa_control(self,x, goal, ob):
        """
        Dynamic Window Approach control
        """
        x=numpy.array(x)
        dw = self.calc_dynamic_window(x)

        u, trajectory = self.calc_control_and_trajectory(x, dw, goal, ob)

        return u, trajectory
    
    def calc_dynamic_window(self,x ):
        """
        calculation dynamic window based on current state x
        """
        # Dynamic window from robot specification
        Vs = [self.min_speed, self.max_speed,
            -self.max_yaw_rate, self.max_yaw_rate]
        # Dynamic window from motion model  위치, 방향, 속도 , 회전속도
        #현 위치에서 맥시범으로 가속을 할때를 가정
        #여기선 0.1초 동안 최대로 가속을 할때를 가정으로 4개의 dynamic window 생성 
        
        Vd = [x[3] - self.max_accel * self.dt,  # 0.0 - 0.2*0.1 
            x[3] + self.max_accel * self.dt,
            x[4] - self.max_delta_yaw_rate * self.dt,  #0.0 - 40.0 * math.pi / 180.0(=0.697777)*0.1
            x[4] + self.max_delta_yaw_rate * self.dt]
        #vd =[-0.02 , 0.02 , -0.06978 , 0.06978 ]
        #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
            max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
        
        #기존의 위치에서 최소속도로 이동하는 값이랑, 최대속력으로이동하는 값이랑 비교해서 값이 큰 값을 dw로 선정
        return dw
    
    def calc_obstacle_cost(self,trajectory, ob):
        """
        calc obstacle cost inf: collision
        """
        #print("타입: ", type(ob))  #타입:  <class 'numpy.ndarray'>
        # print("장애물정보: ",ob)
        try:
            ox = ob[:, 0]
            oy = ob[:, 1]
            dx = ox[:, None]#-28.798 , 장애물 지나고 16
            dy = oy[:, None] #-3.1512  장애물 지나고 0.25
            r = numpy.hypot(dx, dy) #두점 빗변 계산(거리)

            ox = ob[:, 0]
            oy = ob[:, 1]
            r = numpy.hypot(ox, oy) #두점 빗변 계산(거리)

            # print("최근 x,y값: ", trajectory[-1])
            # print("ox와oy값: ",ox,oy)
            # print("dx,dy값: ", dx,dy)
            # print("변수 r값",r ) #13~~14

            if numpy.array(r <= self.robot_radius).any():  #장애물과의 거리보다 로봇의 반지름이 크면 부딫혔다는 얘기
                return float("Inf")
            min_r = numpy.min(r)
            return 1.0 / min_r  # OK
            
        except:
            return 0

    def calc_control_and_trajectory(self,x, dw, goal, ob):
        """
        calculation final inumpyut with dynamic window
        """
        x_init = x[:]
        min_cost = float("inf")
        best_u = [0.0, 0.0] 
        best_trajectory = numpy.array([x])

        for v in numpy.arange(dw[0], dw[1], self.v_resolution): 
            for y in numpy.arange(dw[2], dw[3], self.yaw_rate_resolution): 

                trajectory = self.predict_trajectory(x_init,v,y) 
             
                to_goal_cost = self.to_goal_cost_gain * self.calc_to_goal_cost(trajectory, goal) 
                speed_cost = self.speed_cost_gain * (self.max_speed - trajectory[-1, 3])
                ob_cost = self.obstacle_cost_gain * self.calc_obstacle_cost(trajectory, ob)

                final_cost = to_goal_cost + speed_cost + ob_cost

                if min_cost >= final_cost:  
                    min_cost = final_cost
                    best_u = [v, y]
                    best_trajectory = trajectory


                    # 로봇 끼임 방지?
                    # if abs(best_u[0]) < self.robot_stuck_flag_cons and abs(x[3]) < self.robot_stuck_flag_cons:  #이동할려는 속도가 0.001보다 작다면 이빠이 회전
                    #     # to ensure the robot do not get stuck in
                    #     # best v=0 m/s (in front of an obstacle) and
                    #     # best omega=0 rad/s (heading to the goal with
                    #     # angle difference of 0)
                    #     best_u[1] = -self.max_delta_yaw_rate

        return best_u, best_trajectory    

    def predict_trajectory(self,x_init, v, y): #while로 해당시간동안 motion을 반복해서 x(위치정보)갱신하고, 누적된걸trajetory로 반환
        """
        predict trajectory with an inumpyut
        """

        x = numpy.array(x_init)
        trajectory = numpy.array(x)
        time = 0
        while time <= self.predict_time:  #선박의 상태를 계속 갱신
            x = self.motion(x, [v, y], self.dt)
            trajectory = numpy.vstack((trajectory, x))
            time += self.dt

        return trajectory
    
    def motion(self,x, u, dt): #u에 해당하는 선가속과 각가속을 하고난뒤의 위치갱신
        """
        motion model
        """
        x[2] += u[1] * dt
        x[0] += u[0] * math.cos(x[2]) * dt
        x[1] += u[0] * math.sin(x[2]) * dt
        x[3] = u[0]
        x[4] = u[1]

        return x
    
    def calc_to_goal_cost(self,trajectory, goal):
        """
            calc to goal cost with angle difference
        """
        # print("***********************")
        # print(goal)

        dx = goal[0] - trajectory[-1, 0]  
        dy = goal[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - trajectory[-1, 2]
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

        return cost



def main():
    rospy.init_node("autonomous", anonymous=False)
    start_time = rospy.get_time()
    auto = Autonomous()
    rate = rospy.Rate(10)
    util_n =37 ## 5~7
    fix_imu=0.0
    imu_fix = True

    while not auto.is_all_connected():
        rospy.sleep(0.2)
    print("\n{:<>70}".format(" All Connected !"))
    while not rospy.is_shutdown():
        if len(auto.remained_waypoints) == 0:# 마지막 목적지까지 도착함
            auto.thrusterL_pub.publish(1500)
            auto.thrusterR_pub.publish(1500)
            auto.thrusterL_pub.publish(1500)
            auto.thrusterR_pub.publish(1500)
            rospy.sleep(1.5)
            print("-" * 20)
            print("Finished!")
            return
        else:
            arrived = auto.arrival_check()  # 현 시점에서 목표까지 남은 거리 재계산
            if arrived:  # current goal in and change goal
                auto.thrusterL_pub.publish(1450)
                auto.thrusterR_pub.publish(1450)
                auto.thrusterL_pub.publish(1450)
                auto.thrusterR_pub.publish(1450)
                rospy.sleep(1.5)
                auto.thrusterL_pub.publish(1500)
                auto.thrusterR_pub.publish(1500)
                auto.thrusterL_pub.publish(1500)
                auto.thrusterR_pub.publish(1500)
                rospy.sleep(2.0)
                auto.set_next_goal() 

            else:
                auto.trajectory.append([auto.boat_x, auto.boat_y])  # 이동 경로 추가
                ###move and add the next goal
                # 현재 heading에서 목표로 갈 때 돌려야 할 각도 업데이트
                auto.psi_goal = math.degrees(math.atan2(auto.goal_y - auto.boat_y, auto.goal_x - auto.boat_x)) - auto.psi
                auto.psi_goal = rearrange_angle(auto.psi_goal)

                if imu_fix:
                    fix_imu = auto.psi_goal
                    imu_fix= False

                # 장애물 탐지. 범위 내에 있는 장애물을 필터링하고, 장애물이 있는 각도 리스트를 만듦
                auto.inrange_obstacles, auto.danger_angles = oa.ob_filtering(
                    obstacles=auto.obstacles,
                    dist_to_goal=auto.distance_to_goal,
                    angle_to_goal=auto.psi_goal,
                    span_angle=auto.span_angle,
                    angle_range=auto.ob_angle_range,
                    distance_range=auto.ob_dist_range,
                )

                # 목표각과 현 헤딩 사이 상대적 각도 계산. 선박고정좌표계로 '가야 할 각도'에 해당
                error_angle = oa.calc_desire_angle(
                    danger_angles=auto.danger_angles,
                    angle_to_goal=auto.psi_goal,
                    angle_range=auto.ob_angle_range,
                )

                # 월드좌표계로 '가야 할 각도'를 계산함
                auto.psi_desire = rearrange_angle(auto.psi + error_angle)

                # degree 단위를 servo moter 단위로 변경
                u_servo = control.degree_to_servo(
                    error_angle=error_angle,
                    angle_alpha=auto.angle_alpha,
                    angle_range=auto.rotate_angle_range,
                    servo_range=auto.servo_range,
                )
                # u_servo = moving_avg_filter(auto.filter_queue, auto.filter_queue_size, u_servo)

                #-----------------edit----------------------------------------------------
                #x init
                #need imu-> vel_x, vel_z
                x= numpy.array([auto.boat_x,auto.boat_y,auto.psi,
                                auto.imu.linear_acceleration.x,
                                auto.imu.angular_velocity.z]) #x, y, yaw, 
                goal = numpy.array([auto.goal_x,auto.goal_y])
                u,trajectory = auto.dwa_control(x,goal,auto.inrange_obstacles)

                # Lin_Vel_X
                if u[0]>= 0.0:
                    thruster_speed_L  = auto.v_x_gain * u[0]
                    thruster_speed_R  = auto.v_x_gain * u[0]
                else:
                    thruster_speed_L  = auto.thrust_factor * auto.v_x_gain * u[0]
                    thruster_speed_R  = auto.thrust_factor * auto.v_x_gain * u[0]

                # Ang_Vel_Z
                # thruster_speed_L -= auto.w_z_gain * (auto.half_beam * u[1])
                # thruster_speed_L += auto.w_z_gain * (auto.half_beam * u[1])

                # need the value to 1200~1800 maybe use map
                #original value 

                thruster_speed_L = (thruster_speed_L * 250) + 800
                thruster_speed_R = (thruster_speed_R * 250) + 800
                print(thruster_speed_L)
                print(thruster_speed_R)    
                auto.thrusterL_pub.publish(thruster_speed_L)
                auto.thrusterR_pub.publish(thruster_speed_R)

                # 현 상태 출력 및 시각화
                print("")
                print("{:<9} : {:<6.3f}".format("Run time", rospy.get_time() - start_time))  # 작동 시간
                auto.print_status(error_angle, u_servo)
                auto.imu_fix_pub.publish(fix_imu)
                all_markers = av.visualize(auto)
                auto.visual_rviz_pub.publish(all_markers)
            rate.sleep()


if __name__ == "__main__":
    main()
