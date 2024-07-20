import sys
import serial
import pynmea2
import csv
import time
import traceback
import numpy as np
import keyboard
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWebEngineWidgets import *
import os
import subprocess
import threading
import paramiko
import pyqtgraph as pg
from queue import Queue
from myunitree_robot_go1 import myunitree
from rplidar import RPLidar
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

import robot_resorce_rc

# GUI 이미지 넣기 위한 코드
# terminal: pyrcc5 -o robot_resorce_rc.py robot_resorce.qrc

PORT_NAME = '/dev/ttyUSB0'
DMAX = 1000  # 최대 거리 설정 (mm)

# 라즈베리 파이의 SSH 접속 정보
raspberry_pi_ip = '192.168.208.188'  # 라즈베리 파이의 IP 주소로 변경하세요
username = 'pi'  # 라즈베리 파이의 사용자 이름
password = '48324832jh!'  # 라즈베리 파이의 비밀번호
python_script_path = '/home/pi/Desktop/raspProjects/servoDcMotorJetsonTest.py'  # 라즈베리 파이에서 서보 모터 제어 스크립트 경로


class MainTread(QThread):
    def __init__(self, parent):
        super().__init__(parent)
        self.parent = parent

    def run(self):
        try:
            while True:
                time.sleep(0.01)
                self.parent.sendCmd()
        except Exception as e:
            print("MainTread에서 예외 발생:")
            traceback.print_exc()


class LidarThread(QThread):
    def __init__(self, lidar, data_queue):
        super().__init__()
        self.lidar = lidar
        self.data_queue = data_queue

    def run(self):
        for scan in self.lidar.iter_scans():
            self.data_queue.put(scan)


class GPSThread(QThread):
    gps_data_signal = pyqtSignal(float, float)  # Signal to emit GPS data

    def __init__(self, port, api_key, logging_interval=1000):
        super().__init__()
        self.port = port
        self.api_key = api_key
        self.logging_interval = logging_interval
        self.is_logging = False
        self.init_csv()

        self.min_accuracy = 5  # 최소 정확도 설정
        self.history_size = 5  # 이동 평균 적용을 위한 히스토리 크기
        self.lat_history = []
        self.lon_history = []

    def run(self):
        try:
            self.ser = serial.Serial(self.port, baudrate=115200, timeout=5)
            while True:
                line = self.ser.readline().decode('utf-8')
                if line.startswith('$GPGGA'):
                    msg = pynmea2.parse(line)
                    lat = msg.latitude
                    lon = msg.longitude
                    if msg.lat_dir == 'S':
                        lat = -lat
                    if msg.lon_dir == 'W':
                        lon = -lon

                    # 이동 평균 적용
                    avg_lat, avg_lon = self.apply_moving_average(lat, lon)

                    self.gps_data_signal.emit(avg_lat, avg_lon)
                    self.save_to_csv(lat, lon, 'gps_data_always.csv')
                    if self.is_logging:
                        self.save_to_csv(lat, lon, 'gps_data_logging.csv')
                time.sleep(self.logging_interval / 1000.0)
        except Exception as e:
            print(f"GPS Thread Error: {e}")

    def apply_moving_average(self, lat, lon):
        self.lat_history.append(lat)
        self.lon_history.append(lon)

        if len(self.lat_history) > self.history_size:
            self.lat_history.pop(0)
            self.lon_history.pop(0)

        avg_lat = sum(self.lat_history) / len(self.lat_history)
        avg_lon = sum(self.lon_history) / len(self.lon_history)

        return avg_lat, avg_lon

    def init_csv(self):
        with open('gps_data_always.csv', 'w', newline='') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(['Latitude', 'Longitude'])
        with open('gps_data_logging.csv', 'w', newline='') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(['Latitude', 'Longitude'])

    def save_to_csv(self, lat, lon, filename):
        with open(filename, 'a', newline='') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow([lat, lon])

    def start_logging(self):
        self.is_logging = True

    def stop_logging(self):
        self.is_logging = False

    def set_logging_interval(self, interval):
        self.logging_interval = interval


class CustomWebEnginePage(QWebEnginePage):
    def javaScriptConsoleMessage(self, level, message, line_number, source_id):
        print(f"콘솔 메시지: {message} (줄 {line_number}): {source_id}")


def send_command(command):
    try:
        # SSH 클라이언트 생성
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect(raspberry_pi_ip, username=username, password=password)

        # 원격으로 파이썬 스크립트 실행
        stdin, stdout, stderr = client.exec_command(f'python3 {python_script_path} {command}')

        # 명령 실행 결과 출력
        print("Current Status : ", command)

        # SSH 연결 종료
        client.close()

    except Exception as e:
        print(f"An error occurred: {e}")


class MyWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi(r'./gui_test.ui', self)  # Ui 연결
        self.myunitree_go1 = myunitree()  # myunitree  class 불러와서 명명
        # ----- 변수 초기화 ------------------------------------------
        self.velocity_0_Front_value = 0
        self.velocity_0_Back_value = 0
        self.velocity_1_Left_value = 0
        self.velocity_1_Right_value = 0
        self.yawspeed_value_L = 0
        self.yawspeed_value_R = 0
        self.move_velocity_0_value = 0
        self.move_velocity_1_value = 0

        # 키보드 상태 트래킹
        self.pressed_keys = {
            'w': False,
            's': False,
            'a': False,
            'd': False
        }

        # 장애물 회피용 변수 초기화
        self.obstacle_detected = {
            'Front': False,
            'Back': False,
            'Left': False,
            'Right': False,
            'Front-Right': False,
            'Front-Left': False,
            'Back-Right': False,
            'Back-Left': False
        }

        self.prev_velocity_0_Front_value = 0
        self.prev_velocity_0_Back_value = 0
        self.prev_velocity_1_Left_value = 0
        self.prev_velocity_1_Right_value = 0

        # ------ 버튼 -----------------------------------------------------
        self.connect_btn.clicked.connect(self.udp_connect)  # 통신 연결 버튼
        self.disconnect_btn.clicked.connect(self.udp_disconnect)
        # 컨트롤러 버튼
        self.Stop_btn.clicked.connect(self.Click_Stop_Btn)
        self.UP_btn.clicked.connect(self.Click_UP_Btn)
        self.Down_btn.clicked.connect(self.Click_Down_Btn)
        self.Damping_btn.clicked.connect(self.Click_Damping_Btn)
        self.Recovery_btn.clicked.connect(self.Click_Recovery_Btn)

        # 키보드 핫키 설정
        keyboard.on_press_key("w", lambda _: self.set_key('w', True, self.Front_btn,
                                                          "background-color: rgb(114, 137, 218);"))
        keyboard.on_release_key("w", lambda _: self.set_key('w', False, self.Front_btn,
                                                            "background-color: rgb(255, 255, 255);"))
        keyboard.on_press_key("s", lambda _: self.set_key('s', True, self.Back_btn,
                                                          "background-color: rgb(114, 137, 218);"))
        keyboard.on_release_key("s", lambda _: self.set_key('s', False, self.Back_btn,
                                                            "background-color: rgb(255, 255, 255);"))
        keyboard.on_press_key("a", lambda _: self.set_key('a', True, self.Left_btn,
                                                          "background-color: rgb(114, 137, 218);"))
        keyboard.on_release_key("a", lambda _: self.set_key('a', False, self.Left_btn,
                                                            "background-color: rgb(255, 255, 255);"))
        keyboard.on_press_key("d", lambda _: self.set_key('d', True, self.Right_btn,
                                                          "background-color: rgb(114, 137, 218);"))
        keyboard.on_release_key("d", lambda _: self.set_key('d', False, self.Right_btn,
                                                            "background-color: rgb(255, 255, 255);"))

        keyboard.on_press_key("q", self.press_TurnL_key_callback)
        keyboard.on_release_key("q", self.release_TurnL_key_callback)
        keyboard.on_press_key("e", self.press_TurnR_key_callback)
        keyboard.on_release_key("e", self.release_TurnR_key_callback)

        # ------ 값 입력 ----------------------------------------------------
        self.input_vel_0.valueChanged.connect(self.vel_0_value_changed)
        self.input_vel_1.valueChanged.connect(self.vel_1_value_changed)
        self.input_yawspeed.valueChanged.connect(self.yawspeed_value_changed)

        # ------ Label -----------------------------------------------------
        self.SOC_label = self.findChild(QLabel, "SOC_label")
        self.Mode_label = self.findChild(QLabel, "mode_label")
        self.GaitType_label = self.findChild(QLabel, "gaittype_label")
        self.State_Connect_label = self.findChild(QLabel, "state_connect_label")
        self.Move_State_label = self.findChild(QLabel, "operation_state_label")
        self.obstacle_label = self.findChild(QLabel, "obstacle_label")
        self.lidar_connect_label = self.findChild(QLabel, "lidar_connect_label")
        self.map_status_label = self.findChild(QLabel, "map_status_label")
        self.latitude_label = self.findChild(QLabel, "latitude_label")
        self.longitude_label = self.findChild(QLabel, "longitude_label")
        # ------ ComboBox ---------------------------------------------------
        self.Mode_ComboBox = self.findChild(QComboBox, "mode_comboBox")
        self.Mode_ComboBox.currentIndexChanged.connect(self.Change_mode_combobox)
        self.GaitType_ComboBox = self.findChild(QComboBox, "gaittype_comboBox")
        self.GaitType_ComboBox.currentIndexChanged.connect(self.Change_gaittype_comboBox)

        # ----- Lidar -----------------------------------------------
        # Lidar Setup
        try:
            self.data_queue = Queue()
            self.lidar = RPLidar(PORT_NAME)
            self.slam_view = self.findChild(pg.PlotWidget, "slam_view")

            self.slam_figure = Figure()
            self.slam_canvas = FigureCanvas(self.slam_figure)
            self.slam_layout = QVBoxLayout(self.slam_view)
            self.slam_layout.addWidget(self.slam_canvas)

            if self.check_lidar_connection():
                self.lidar.start_motor()
                self.start_lidar_thread()  # LiDAR 스레드 시작
                self.timer = QTimer()
                self.timer.timeout.connect(self.process_lidar_data)
                self.timer.start(100)  # 100ms마다 데이터 처리
            else:
                print("LiDAR not connected: Thread will not start.")
        except Exception as e:
            print(f"Failed to initialize LiDAR: {e}")
            self.lidar = None
            self.lidar_connect_label.setText("Disconnect")
            self.lidar_connect_label.setStyleSheet("color: rgb(237,66,69);")

        # ----------------------GPS-----------------------------------
        self.api_key = "AIzaSyBUBAhu3jl8NIC54-BXqEggLmJo-YNCIcw"
        self.port = '/dev/ttyTHS1'  # GPS 모듈의 시리얼 포트로 변경

        self.gps_thread = GPSThread(self.port, self.api_key)
        self.gps_thread.gps_data_signal.connect(self.update_gps_data)
        self.gps_thread.start()

        # QWebEngineView 설정
        self.map_view_widget = self.findChild(QWidget, "map_view")
        self.map_view = QWebEngineView()
        self.map_view.setPage(CustomWebEnginePage(self.map_view))

        layout = QVBoxLayout(self.map_view_widget)
        layout.addWidget(self.map_view)

        self.load_map()

        # 버튼 클릭 이벤트 연결
        self.map_expand_btn.clicked.connect(lambda: self.execute_js("map.setZoom(map.getZoom() + 1);"))
        self.map_reduce_btn.clicked.connect(lambda: self.execute_js("map.setZoom(map.getZoom() - 1);"))
        self.map_record_start_btn.clicked.connect(self.start_logging)
        self.map_record_stop_btn.clicked.connect(self.stop_logging)

        # 위치 기록 주기 변경 이벤트 연결
        self.input_map_period.valueChanged.connect(self.update_logging_interval)

        # 서보 모터 제어 관련 요소 초기화
        self.seed_bucket_open_btn = self.findChild(QPushButton, 'seed_bucket_open_btn')
        self.seed_bucket_open_btn.clicked.connect(self.start_servo)
        self.seed_bucket_close_btn = self.findChild(QPushButton, 'seed_bucket_close_btn')
        self.seed_bucket_close_btn.clicked.connect(self.stop_servo)

        # DC 모터 제어 관련 요소 초기화
        self.seed_spreader_on_btn = self.findChild(QPushButton, 'seed_spreader_on_btn')
        self.seed_spreader_on_btn.clicked.connect(self.dc_motor_on)
        self.seed_spreader_off_btn = self.findChild(QPushButton, 'seed_spreader_off_btn')
        self.seed_spreader_off_btn.clicked.connect(self.dc_motor_off)
        self.seed_spreader_lowspeed_btn = self.findChild(QPushButton, 'seed_spreader_lowspeed_btn')
        self.seed_spreader_lowspeed_btn.clicked.connect(self.set_low_speed)
        self.seed_spreader_normalspeed_btn = self.findChild(QPushButton, 'seed_spreader_normalspeed_btn')
        self.seed_spreader_normalspeed_btn.clicked.connect(self.set_middle_speed)
        self.seed_spreader_highspeed_btn = self.findChild(QPushButton, 'seed_spreader_highspeed_btn')
        self.seed_spreader_highspeed_btn.clicked.connect(self.set_high_speed)

    # ------ SendCmd -------------------------------------
    def sendCmd(self):
        self.myunitree_go1.sendCmd()

        self.data_SOC = self.myunitree_go1.hstate_bms_SOC
        self.data_mode = self.myunitree_go1.hstate_mode
        self.data_gaitType = self.myunitree_go1.hstate_gaitType
        self.data_velocity = self.myunitree_go1.hstate_velocity

        self.update_label()

    # ------데이터 입력 이벤트------------
    def vel_0_value_changed(self, value):
        self.velocity_0_Front_value = value
        self.velocity_0_Back_value = -value

    def vel_1_value_changed(self, value):
        self.velocity_1_Left_value = value
        self.velocity_1_Right_value = -value

    def yawspeed_value_changed(self, value):
        self.yawspeed_value_L = value
        self.yawspeed_value_R = -value

    # ------버튼 클릭 이벤트--------------
    def Click_Stop_Btn(self):
        if self.myunitree_go1.connect_flag:
            self.myunitree_go1.Robot_force_Stop()

    def Click_UP_Btn(self):
        if self.myunitree_go1.connect_flag:
            self.myunitree_go1.Change_Mode_to_STAND_UP()

    def Click_Down_Btn(self):
        if self.myunitree_go1.connect_flag:
            self.myunitree_go1.Change_Mode_to_STAND_DOWN()

    def Click_Damping_Btn(self):
        if self.myunitree_go1.connect_flag:
            self.myunitree_go1.Change_Mode_to_Damping()

    def Click_Recovery_Btn(self):
        if self.myunitree_go1.connect_flag:
            self.myunitree_go1.Change_Mode_to_Recovery_Stand()

    def set_key(self, key, value, button, style):
        self.pressed_keys[key] = value
        button.setStyleSheet(style if value else "background-color: rgb(255, 255, 255);")
        if self.myunitree_go1.connect_flag:
            self.update_movement()

    def update_movement(self):
        key_input_vel0 = 0
        key_input_vel1 = 0
        if self.pressed_keys['w']:
            key_input_vel0 = self.velocity_0_Front_value
        if self.pressed_keys['s']:
            key_input_vel0 = self.velocity_0_Back_value
        if self.pressed_keys['a']:
            key_input_vel1 = self.velocity_1_Left_value
        if self.pressed_keys['d']:
            key_input_vel1 = self.velocity_1_Right_value

        # 장애물이 감지된 방향의 속도를 0으로 설정
        if self.obstacle_detected['Front']:
            if key_input_vel0 > 0:
                key_input_vel0 = 0
        if self.obstacle_detected['Back']:
            if key_input_vel0 < 0:
                key_input_vel0 = 0
        if self.obstacle_detected['Left']:
            if key_input_vel1 > 0:
                key_input_vel1 = 0
        if self.obstacle_detected['Right']:
            if key_input_vel1 < 0:
                key_input_vel1 = 0
        if self.obstacle_detected['Front-Right']:
            if key_input_vel0 > 0 and key_input_vel1 < 0:
                key_input_vel0 = 0
                key_input_vel1 = 0
        if self.obstacle_detected['Front-Left']:
            if key_input_vel0 > 0 and key_input_vel1 > 0:
                key_input_vel0 = 0
                key_input_vel1 = 0
        if self.obstacle_detected['Back-Right']:
            if key_input_vel0 < 0 and key_input_vel1 < 0:
                key_input_vel0 = 0
                key_input_vel1 = 0
        if self.obstacle_detected['Back-Left']:
            if key_input_vel0 < 0 and key_input_vel1 > 0:
                key_input_vel0 = 0
                key_input_vel1 = 0

        # 현재 움직임 상태 업데이트
        self.move_velocity_0_value = key_input_vel0
        self.move_velocity_1_value = key_input_vel1
        self.myunitree_go1.Move_mult(self.move_velocity_0_value, self.move_velocity_1_value)

    def press_TurnL_key_callback(self, event):
        self.Turn_L_btn.setStyleSheet("background-color: rgb(235, 69, 158);")
        if self.myunitree_go1.connect_flag:
            self.myunitree_go1.Turn_RL(self.yawspeed_value_L)

    def press_TurnR_key_callback(self, event):
        self.Turn_R_btn.setStyleSheet("background-color: rgb(235, 69, 158);")
        if self.myunitree_go1.connect_flag:
            self.myunitree_go1.Turn_RL(self.yawspeed_value_R)

    def release_TurnL_key_callback(self, event):
        self.Turn_L_btn.setStyleSheet("background:rgb(153, 170, 181);" "color:rgb(255, 255, 255);")
        if self.myunitree_go1.connect_flag:
            self.myunitree_go1.Turn_Stop()

    def release_TurnR_key_callback(self, event):
        self.Turn_R_btn.setStyleSheet("background:rgb(153, 170, 181);" "color:rgb(255, 255, 255);")
        if self.myunitree_go1.connect_flag:
            self.myunitree_go1.Turn_Stop()

    # ------ 콤보 박스 메소드 --------------
    def Change_mode_combobox(self, index):
        selected_item = self.Mode_ComboBox.currentText()
        print(f"Selected Mode: {selected_item}")

        if selected_item == "IDLE (0)":
            self.myunitree_go1.Change_Mode_to_IDLE()
        elif selected_item == "Force Stand (1)":
            self.myunitree_go1.Change_Mode_to_Force_Stand()
        elif selected_item == "Vel Walk (2)":
            self.myunitree_go1.Change_Mode_to_VEL_WALK()
        elif selected_item == "Stand Down (5)":
            self.myunitree_go1.Change_Mode_to_STAND_DOWN()
        elif selected_item == "Stand Up (6)":
            self.myunitree_go1.Change_Mode_to_STAND_UP()

    def Change_gaittype_comboBox(self, index):
        selected_item = self.GaitType_ComboBox.currentText()
        print(f"Selected GaitType: {selected_item}")

        if selected_item == "IDLE (0)":
            self.myunitree_go1.Change_GaitType_to_IDLE()
        elif selected_item == "Trot (1)":
            self.myunitree_go1.Change_GaitType_to_Trot()
        elif selected_item == "Climb Stair (2)":
            self.myunitree_go1.Change_GaitType_to_CLIMB_STAIR()
        elif selected_item == "Trot Obstacle (3)":
            self.myunitree_go1.Change_GaitType_to_TROT_OBSTACLE()

    # ---------------------------------------------------------------------
    def udp_connect(self):
        try:
            self.myunitree_go1.connect()
            h1 = MainTread(self)
            h1.start()
        except Exception as e:
            print("udp_connect에서 예외 발생:")
            traceback.print_exc()

    def udp_disconnect(self):
        try:
            self.myunitree_go1.disconnect()
            h1 = MainTread(self)
            h1.start()
        except Exception as e:
            print("udp_disconnect에서 예외 발생:")
            traceback.print_exc()

    def update_label(self):
        self.SOC_label.setText("{:.1f}".format(self.data_SOC))
        self.Mode_label.setText("{:.1f}".format(self.data_mode))
        self.GaitType_label.setText("{:.1f}".format(self.data_gaitType))

        if self.myunitree_go1.connect_flag:
            self.State_Connect_label.setText("Connect")
            self.State_Connect_label.setStyleSheet("color: rgb(87,242,135);")
        else:
            self.State_Connect_label.setText("Disconnect")
            self.State_Connect_label.setStyleSheet("color: rgb(237,66,69);")

        if (abs(self.data_velocity[0]) < 0.05
                and abs(self.data_velocity[1]) < 0.05):
            self.Move_State_label.setText("STOP")
            self.Move_State_label.setStyleSheet("color: rgb(237,66,69);")
        else:
            self.Move_State_label.setText("Moving..")
            self.Move_State_label.setStyleSheet("color: rgb(254,231,92);")

    def update_line(self, scan):
        self.slam_figure.clear()
        polar_ax = self.slam_figure.add_subplot(111, projection='polar')
        polar_ax.set_theta_zero_location('N')
        polar_ax.set_theta_direction(-1)
        polar_ax.set_rmax(DMAX)
        polar_ax.grid(True)

        offsets = np.array([(np.radians(meas[1]), meas[2]) for meas in scan])
        colors = np.array(['red' if meas[2] < 500 else 'grey' for meas in scan])
        polar_ax.scatter(offsets[:, 0], offsets[:, 1], s=5, color=colors, lw=0)

        self.detect_obstacles(scan)
        self.slam_canvas.draw()

    def detect_obstacles(self, scan):
        close_points = np.array([(meas[1], meas[2]) for meas in scan if meas[2] < 500])

        if close_points.size == 0:
            self.obstacle_detected = {
                'Front': False,
                'Back': False,
                'Left': False,
                'Right': False,
                'Front-Right': False,
                'Front-Left': False,
                'Back-Right': False,
                'Back-Left': False
            }
            self.obstacle_distances = {
                'Front': None,
                'Back': None,
                'Left': None,
                'Right': None,
                'Front-Right': None,
                'Front-Left': None,
                'Back-Right': None,
                'Back-Left': None
            }
            self.obstacle_label.setText("0")
            self.update_obstacle_colors()
            self.update_obstacle_distances()
            return

        angles = close_points[:, 0]
        distances = close_points[:, 1]

        clusters = []
        current_cluster = [close_points[0]]

        for point in close_points[1:]:
            if np.abs(point[0] - current_cluster[-1][0]) < 15:
                current_cluster.append(point)
            else:
                if len(current_cluster) >= 10:
                    clusters.append(np.array(current_cluster))
                current_cluster = [point]

        if len(current_cluster) >= 5:
            clusters.append(np.array(current_cluster))

        self.obstacle_detected = {
            'Front': False,
            'Back': False,
            'Left': False,
            'Right': False,
            'Front-Right': False,
            'Front-Left': False,
            'Back-Right': False,
            'Back-Left': False
        }

        self.obstacle_distances = {
            'Front': None,
            'Back': None,
            'Left': None,
            'Right': None,
            'Front-Right': None,
            'Front-Left': None,
            'Back-Right': None,
            'Back-Left': None
        }

        for cluster in clusters:
            avg_angle = np.mean(cluster[:, 0])
            avg_distance = np.mean(cluster[:, 1])
            direction = self.determine_direction(avg_angle)
            self.obstacle_detected[direction] = True
            self.obstacle_distances[direction] = avg_distance

        self.obstacle_label.setText(f"{len(clusters)}개")
        self.update_obstacle_colors()
        self.update_obstacle_distances()

    def update_obstacle_colors(self):
        color_map = {
            'Front': self.obstacle_front_frame,
            'Back': self.obstacle_back_frame,
            'Left': self.obstacle_left_frame,
            'Right': self.obstacle_right_frame,
            'Front-Right': self.obstacle_front_right_frame,
            'Front-Left': self.obstacle_front_left_frame,
            'Back-Right': self.obstacle_back_right_frame,
            'Back-Left': self.obstacle_back_left_frame
        }

        for direction, frame in color_map.items():
            if self.obstacle_detected[direction]:
                frame.setStyleSheet("background-color: rgb(237,66,69);")
            else:
                frame.setStyleSheet("background-color: rgb(87, 242, 135);")

    def update_obstacle_distances(self):
        distance_map = {
            'Front': self.obstacle_front_label,
            'Back': self.obstacle_back_label,
            'Left': self.obstacle_left_label,
            'Right': self.obstacle_right_label,
            'Front-Right': self.obstacle_front_right_label,
            'Front-Left': self.obstacle_front_left_label,
            'Back-Right': self.obstacle_back_right_label,
            'Back-Left': self.obstacle_back_left_label
        }

        for direction, label in distance_map.items():
            if self.obstacle_distances[direction] is not None:
                label.setText(f"{self.obstacle_distances[direction]:.1f} mm")
            else:
                label.setText(" - ")

    def determine_direction(self, angle):
        if 337.5 <= angle or angle < 22.5:
            return "Front"
        elif 22.5 <= angle < 67.5:
            return "Front-Right"
        elif 67.5 <= angle < 112.5:
            return "Right"
        elif 112.5 <= angle < 157.5:
            return "Back-Right"
        elif 157.5 <= angle < 202.5:
            return "Back"
        elif 202.5 <= angle < 247.5:
            return "Back-Left"
        elif 247.5 <= angle < 292.5:
            return "Left"
        elif 292.5 <= angle < 337.5:
            return "Front-Left"

    def check_lidar_connection(self):
        try:
            info = self.lidar.get_info()
            print(f"Lidar Info: {info}")
            self.lidar_connect_label.setText("Connect")
            self.lidar_connect_label.setStyleSheet("color: rgb(87,242,135);")
            return True
        except Exception as e:
            print(f"Failed to connect to Lidar: {e}")
            self.lidar_connect_label.setText("Disconnect")
            self.lidar_connect_label.setStyleSheet("color: rgb(237,66,69);")
            return False

    def process_lidar_data(self):
        if not self.data_queue.empty():
            scan = self.data_queue.get()
            self.update_line(scan)

    def start_lidar_thread(self):
        if self.lidar is not None:
            self.lidar_thread = LidarThread(self.lidar, self.data_queue)
            self.lidar_thread.start()

    def closeEvent(self, event):
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()
        super().closeEvent(event)

    def start_servo(self):
        """
        서보 모터를 시작하는 메서드.
        """
        send_command('open')
        # print("Seed spreader is << OPENED >>")

    def stop_servo(self):
        """
        서보 모터를 중지하는 메서드.
        """
        send_command('close')
        # print("Seed spreader is << CLOSED >>")

    def dc_motor_on(self):
        """
        DC 모터를 켜는 메서드.
        """
        send_command('on')
        # print("Seed spreader is << ON >>")

    def dc_motor_off(self):
        """
        DC 모터를 끄는 메서드.
        """
        send_command('off')
        # print("Seed spreader is << OFF >>")

    def set_low_speed(self):
        """
        DC 모터를 저속으로 설정하는 메서드.
        """
        send_command('low')
        # print("Seed spreader is << LOW SPEED >>")

    def set_middle_speed(self):
        """
        DC 모터를 중속으로 설정하는 메서드.
        """
        send_command('mid')
        # print("Seed spreader is << MIDDLE SPEED >>")

    def set_high_speed(self):
        """
        DC 모터를 고속으로 설정하는 메서드.
        """
        send_command('high')
        # print("Seed spreader is << HIGH SPEED >>")

    def update_gps_data(self, lat, lon):
        self.add_marker(lat, lon)
        self.latitude_label.setText(f"{lat:.3f}")
        self.longitude_label.setText(f"{lon:.3f}")

    def load_map(self):
        html_content = self.get_map_html()
        with open('map.html', 'w') as f:
            f.write(html_content)
        self.map_view.setUrl(QUrl(f"http://localhost:8001/map.html"))

    def get_map_html(self):
        return f"""
        <!DOCTYPE html>
        <html>
        <head>
            <title>Google Maps</title>
            <style>
                body, html {{
                    height: 100%;
                    margin: 0;
                    padding: 0;
                }}
                #map {{
                    height: 100%;
                }}
            </style>
            <script>
                let map;
                let polyline;
                let path = [];
                let currentMarker;

                function initMap() {{
                    map = new google.maps.Map(document.getElementById('map'), {{
                        center: {{lat: 0, lng: 0}},
                        zoom: 2
                    }});

                    polyline = new google.maps.Polyline({{
                        path: path,
                        geodesic: true,
                        strokeColor: '#FF0000',
                        strokeOpacity: 1.0,
                        strokeWeight: 2
                    }});
                    polyline.setMap(map);
                }}

                function addMarker(lat, lng) {{
                    var position = new google.maps.LatLng(lat, lng);

                    if (currentMarker) {{
                        currentMarker.setMap(null);
                    }}

                    currentMarker = new google.maps.Marker({{
                        position: position,
                        map: map
                    }});

                    path.push(position);
                    polyline.setPath(path);
                    map.setCenter(position);
                }}
            </script>
        </head>
        <body>
            <div id="map"></div>
            <script src="https://maps.googleapis.com/maps/api/js?key={self.api_key}&callback=initMap" async defer></script>
        </body>
        </html>
        """

    @pyqtSlot()
    def add_marker(self, lat, lon):
        self.execute_js(f"addMarker({lat}, {lon});")

    def execute_js(self, script):
        self.map_view.page().runJavaScript(script)

    def start_logging(self):
        self.gps_thread.start_logging()
        self.map_status_label.setText("데이터 기록 시작")

    def stop_logging(self):
        self.gps_thread.stop_logging()
        self.map_status_label.setText("데이터 기록 정지")

    def update_logging_interval(self):
        interval = self.input_map_period.value() * 1000
        self.gps_thread.set_logging_interval(interval)


if __name__ == '__main__':
    def run_local_server():
        os.chdir(os.path.dirname(os.path.abspath(__file__)))
        subprocess.run(['python3', '-m', 'http.server', '8001'])


    server_thread = threading.Thread(target=run_local_server)
    server_thread.daemon = True
    server_thread.start()
    sys.argv.append('--no-sandbox')

    app = QApplication(sys.argv)
    window = MyWindow()
    window.show()
    app.exec_()
