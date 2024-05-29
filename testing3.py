# 대각선 장애물 감지와 대처 방식 알고리즘 설계 코드

import sys
import time
import traceback
import numpy as np
import keyboard
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import pyqtgraph as pg
from myunitree_robot_test import myunitree
from rplidar import RPLidar
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


PORT_NAME = 'COM3'
DMAX = 1000  # 최대 거리 설정 (mm)


class Tread1(QThread):
    def __init__(self, parent):
        super().__init__(parent)
        self.parent = parent

    def run(self):
        try:
            while True:
                time.sleep(0.01)
                self.parent.sendCmd()
        except Exception as e:
            print("Tread1에서 예외 발생:")
            traceback.print_exc()


class LidarThread(QThread):
    update_signal = pyqtSignal(list)

    def __init__(self, lidar):
        super().__init__()
        self.lidar = lidar

    def run(self):
        for scan in self.lidar.iter_scans():
            self.update_signal.emit(scan)


class MyWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi(r'./sample2.ui', self)  # Ui 연결
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
            'Right': False
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

        # 키보드 핫키 설정
        keyboard.on_press_key("w", lambda _: self.set_key('w', True, self.Front_btn,
                                                          "background-color: rgb(172, 206, 255);"))
        keyboard.on_release_key("w", lambda _: self.set_key('w', False, self.Front_btn,
                                                            "background-color: rgb(255, 255, 255);"))
        keyboard.on_press_key("s", lambda _: self.set_key('s', True, self.Back_btn,
                                                          "background-color: rgb(172, 206, 255);"))
        keyboard.on_release_key("s", lambda _: self.set_key('s', False, self.Back_btn,
                                                            "background-color: rgb(255, 255, 255);"))
        keyboard.on_press_key("a", lambda _: self.set_key('a', True, self.Left_btn,
                                                          "background-color: rgb(172, 206, 255);"))
        keyboard.on_release_key("a", lambda _: self.set_key('a', False, self.Left_btn,
                                                            "background-color: rgb(255, 255, 255);"))
        keyboard.on_press_key("d", lambda _: self.set_key('d', True, self.Right_btn,
                                                          "background-color: rgb(172, 206, 255);"))
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
        # ------ ComboBox ---------------------------------------------------
        self.Mode_ComboBox = self.findChild(QComboBox, "mode_comboBox")
        self.Mode_ComboBox.currentIndexChanged.connect(self.Change_mode_combobox)
        self.GaitType_ComboBox = self.findChild(QComboBox, "gaittype_comboBox")
        self.GaitType_ComboBox.currentIndexChanged.connect(self.Change_gaittype_comboBox)

        # ----- Lidar -----------------------------------------------
        # Lidar Setup
        try:
            self.lidar = RPLidar(PORT_NAME)
            self.slam_view = self.findChild(pg.PlotWidget, "slam_view")

            self.slam_figure = Figure()
            self.slam_canvas = FigureCanvas(self.slam_figure)
            self.slam_layout = QVBoxLayout(self.slam_view)
            self.slam_layout.addWidget(self.slam_canvas)

            if self.check_lidar_connection():
                self.lidar.start_motor()
                self.start_lidar_thread()  # Lidar 스레드 시작
            else:
                print("Lidar not connected: Thread will not start.")
        except Exception as e:
            print(f"Failed to initialize Lidar: {e}")
            self.lidar = None

    # ------ SendCmd -------------------------------------
    def sendCmd(self):
        self.myunitree_go1.sendCmd()

        self.data_SOC = self.myunitree_go1.hstate_bms_SOC
        self.data_mode = self.myunitree_go1.hstate_mode
        self.data_gaitType = self.myunitree_go1.hstate_gaitType
        # self.data_position_hstate = self.myunitree_go1.hstate_position

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

        # 현재 움직임 상태 업데이트
        self.move_velocity_0_value = key_input_vel0
        self.move_velocity_1_value = key_input_vel1
        self.myunitree_go1.Move_mult(self.move_velocity_0_value, self.move_velocity_1_value)

    def press_TurnL_key_callback(self, event):
        self.Turn_L_btn.setStyleSheet("background-color: rgb(206, 206, 206);")
        if self.myunitree_go1.connect_flag:
            self.myunitree_go1.Turn_RL(self.yawspeed_value_L)

    def press_TurnR_key_callback(self, event):
        self.Turn_R_btn.setStyleSheet("background-color: rgb(206, 206, 206);")
        if self.myunitree_go1.connect_flag:
            self.myunitree_go1.Turn_RL(self.yawspeed_value_R)

    def release_TurnL_key_callback(self, event):
        self.Turn_L_btn.setStyleSheet("background:rgb(112, 112, 112);" "color:rgb(255, 255, 255);")
        if self.myunitree_go1.connect_flag:
            self.myunitree_go1.Turn_Stop()

    def release_TurnR_key_callback(self, event):
        self.Turn_R_btn.setStyleSheet("background:rgb(112, 112, 112);" "color:rgb(255, 255, 255);")
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
            h1 = Tread1(self)
            h1.start()
        except Exception as e:
            print("udp_connect에서 예외 발생:")
            traceback.print_exc()

    def udp_disconnect(self):
        try:
            self.myunitree_go1.disconnect()
            h1 = Tread1(self)
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
            self.State_Connect_label.setStyleSheet("color: blue;")
        else:
            self.State_Connect_label.setText("Disconnect")
            self.State_Connect_label.setStyleSheet("color: red;")

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
        # 거리 500mm 이하의 측정값 필터링
        close_points = np.array([(meas[1], meas[2]) for meas in scan if meas[2] < 500])

        if close_points.size == 0:
            self.obstacle_detected = {
                'Front': False,
                'Back': False,
                'Left': False,
                'Right': False
            }
            return

        angles = close_points[:, 0]
        distances = close_points[:, 1]

        # 클러스터링 알고리즘 적용
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

        for cluster in clusters:
            avg_angle = np.mean(cluster[:, 0])
            avg_distance = np.mean(cluster[:, 1])
            direction = self.determine_direction(avg_angle)
            print(f"장애물 감지: 방향 {direction}, 평균 거리 {avg_distance}mm")
            self.obstacle_detected[direction] = True

    def determine_direction(self, angle):
        if 20 <= angle <= 160:
            return "Right"
        elif 160 < angle <= 200:
            return "Back"
        elif 200 < angle <= 340:
            return "Left"
        else:
            return "Front"

    def check_lidar_connection(self):
        try:
            info = self.lidar.get_info()
            print(f"Lidar Info: {info}")
            return True
        except Exception as e:
            print(f"Failed to connect to Lidar: {e}")
            return False

    def start_lidar_thread(self):
        if self.lidar is not None:
            self.lidar_thread = LidarThread(self.lidar)
            self.lidar_thread.update_signal.connect(self.update_line)
            self.lidar_thread.start()

    def closeEvent(self, event):
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()
        super().closeEvent(event)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MyWindow()
    window.show()
    app.exec_()