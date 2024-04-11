import sys
import time
import traceback
import numpy as np
import math
import os
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
        self.myunitree_b1 = myunitree()  # myunitree  class 불러와서 명명
        # ----- 변수 초기화 ------------------------------------------
        self.velocity_0_Front_value = 0
        self.velocity_0_Back_value = 0
        self.velocity_1_Left_value = 0
        self.velocity_1_Right_value = 0
        self.yawspeed_value_L = 0
        self.yawspeed_value_R = 0
        self.move_vel_0 = 0
        self.move_vel_1 = 0
        self.AutoMode_flag = False


        # ------ 버튼 -----------------------------------------------------
        self.connect_btn.clicked.connect(self.udp_connect)      # 통신 연결 버튼
        self.disconnect_btn.clicked.connect(self.udp_disconnect)
        # 컨트롤러 버튼
        self.N_btn.pressed.connect(self.Click_Front_Btn)
        self.N_btn.released.connect(self.Release_Front_Btn)
        self.S_btn.pressed.connect(self.Click_Back_Btn)
        self.S_btn.released.connect(self.Release_Back_Btn)
        self.W_btn.pressed.connect(self.Click_Left_Btn)
        self.W_btn.released.connect(self.Release_Left_Btn)
        self.E_btn.pressed.connect(self.Click_Right_Btn)
        self.E_btn.released.connect(self.Release_Right_Btn)

        self.Stop_btn.clicked.connect(self.Click_Stop_Btn)

        self.L_btn.pressed.connect(self.Click_Turn_L_Btn)
        self.L_btn.released.connect(self.Release_Turn_L_Btn)
        self.R_btn.pressed.connect(self.Click_Turn_R_Btn)
        self.R_btn.released.connect(self.Release_Turn_R_Btn)

        # self.auto_start_position_btn.pressed.connect(self.click_auto_start_Position)
        # self.auto_end_position_btn.pressed.connect(self.click_auto_end_Position)

        self.Front_btn_pressed_state = False
        self.Back_btn_pressed_state = False
        self.Left_btn_pressed_state = False
        self.Right_btn_pressed_state = False
        self.Turn_L_btn_pressed_state = False
        self.Turn_R_btn_pressed_state = False
        # ------ 값 입력 ----------------------------------------------------
        self.input_vel_0.valueChanged.connect(self.vel_0_value_changed)
        self.input_vel_1.valueChanged.connect(self.vel_1_value_changed)

        # ------ Label -----------------------------------------------------
        self.SOC_label = self.findChild(QLabel, "SOC_label")
        self.Mode_label = self.findChild(QLabel, "mode_label")
        self.GaitType_label = self.findChild(QLabel, "gaittype_label")
        self.State_Position_0_label = self.findChild(QLabel, "state_position_0_label")
        self.State_Position_1_label = self.findChild(QLabel, "state_position_1_label")
        self.State_Connect_label = self.findChild(QLabel, "state_connect_label")
        # ------ ComboBox ---------------------------------------------------
        self.Mode_ComboBox = self.findChild(QComboBox, "mode_comboBox")
        self.Mode_ComboBox.currentIndexChanged.connect(self.Change_mode_combobox)
        self.GaitType_ComboBox = self.findChild(QComboBox, "gaittype_comboBox")
        self.GaitType_ComboBox.currentIndexChanged.connect(self.Change_gaittype_comboBox)


        # ----- Lidar -----------------------------------------------
        # Lidar Setup
        self.lidar = RPLidar(PORT_NAME)
        self.lidar.start_motor()

        self.slam_view = self.findChild(pg.PlotWidget, "slam_view")

        self.slam_figure = Figure()
        self.slam_canvas = FigureCanvas(self.slam_figure)
        self.slam_layout = QVBoxLayout(self.slam_view)
        self.slam_layout.addWidget(self.slam_canvas)

        self.start_lidar_thread()


    # ------ SendCmd -------------------------------------
    def sendCmd(self):
        self.myunitree_b1.sendCmd()

        self.highstate_textBrowser.append(self.myunitree_b1.highstate_info)

        self.data_SOC = self.myunitree_b1.hstate_bms_SOC
        self.data_mode = self.myunitree_b1.hstate_mode
        self.data_gaitType = self.myunitree_b1.hstate_gaitType
        self.data_yawspeed = self.myunitree_b1.hstate_yawspeed

        self.data_position_hstate = self.myunitree_b1.hstate_position

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
    def Click_Front_Btn(self):
        self.Front_btn_pressed_state = True
        self.N_btn.setStyleSheet("background-color: rgb(172, 206, 255);")
        self.myunitree_b1.Move_Front(self.velocity_0_Front_value)
    def Click_Back_Btn(self):
        self.Back_btn_pressed_state = True
        self.S_btn.setStyleSheet("background-color: rgb(172, 206, 255);")
        self.myunitree_b1.Move_Back(self.velocity_0_Back_value)
    def Click_Left_Btn(self):
        self.Left_btn_pressed_state = True
        self.W_btn.setStyleSheet("background-color: rgb(172, 206, 255);")
        self.myunitree_b1.Move_Left(self.velocity_1_Left_value)
    def Click_Right_Btn(self):
        self.Right_btn_pressed_state = True
        self.E_btn.setStyleSheet("background-color: rgb(172, 206, 255);")
        self.myunitree_b1.Move_Right(self.velocity_1_Right_value)
    def Click_Stop_Btn(self):
        self.myunitree_b1.Robot_force_Stop()

    def Click_Turn_L_Btn(self):
        self.Turn_L_btn_pressed_state = True
        self.L_btn.setStyleSheet("background-color: rgb(206, 206, 206);")
        self.myunitree_b1.Turn_Left(self.yawspeed_value_L)
    def Click_Turn_R_Btn(self):
        self.Turn_R_btn_pressed_state = True
        self.R_btn.setStyleSheet("background-color: rgb(206, 206, 206);")
        self.myunitree_b1.Turn_Right(self.yawspeed_value_R)


    def Release_Front_Btn(self):
        self.Front_btn_pressed_state = False
        self.N_btn.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.myunitree_b1.Robot_Stop()
    def Release_Back_Btn(self):
        self.Back_btn_pressed_state = False
        self.S_btn.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.myunitree_b1.Robot_Stop()
    def Release_Left_Btn(self):
        self.Left_btn_pressed_state = False
        self.W_btn.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.myunitree_b1.Robot_Stop()
    def Release_Right_Btn(self):
        self.Right_btn_pressed_state = False
        self.E_btn.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.myunitree_b1.Robot_Stop()
    def Release_Turn_L_Btn(self):
        self.Turn_L_btn_pressed_state = False
        self.L_btn.setStyleSheet("background:rgb(112, 112, 112);"
                                 "color:rgb(255, 255, 255);")
        self.myunitree_b1.Robot_Stop()
    def Release_Turn_R_Btn(self):
        self.Turn_R_btn_pressed_state = False
        self.R_btn.setStyleSheet("background:rgb(112, 112, 112);"
                                 "color:rgb(255, 255, 255);")
        self.myunitree_b1.Robot_Stop()

    # ------ 콤보 박스 메소드 --------------
    def Change_mode_combobox(self, index):
        selected_item = self.Mode_ComboBox.currentText()
        print(f"Selected Mode: {selected_item}")

        if selected_item == "IDLE (0)":
            self.myunitree_b1.Change_Mode_to_IDLE()
        elif selected_item == "Force Stand (1)":
            self.myunitree_b1.Change_Mode_to_Force_Stand()
        # elif selected_item == "Vel Walk (2)":
        # self.myunitree_b1.Change_Mode_to_VEL_WALK()
        elif selected_item == "Stand Down (5)":
            self.myunitree_b1.Change_Mode_to_STAND_DOWN()
        elif selected_item == "Stand Up (6)":
            self.myunitree_b1.Change_Mode_to_STAND_UP()

    def Change_gaittype_comboBox(self, index):
        selected_item = self.GaitType_ComboBox.currentText()
        print(f"Selected GaitType: {selected_item}")

        if selected_item == "IDLE (0)":
            self.myunitree_b1.Change_GaitType_to_IDLE()
        elif selected_item == "Trot (1)":
            self.myunitree_b1.Change_GaitType_to_Trot()
        elif selected_item == "Climb Stair (2)":
            self.myunitree_b1.Change_GaitType_to_CLIMB_STAIR()
        elif selected_item == "Trot Obstacle (3)":
            self.myunitree_b1.Change_GaitType_to_TROT_OBSTACLE()

    # ---------------------------------------------------------------------
    def udp_connect(self):
        try:
            self.myunitree_b1.connect()
            h1 = Tread1(self)
            h1.start()
            self.plot_timer = QTimer(self)
            self.plot_timer.timeout.connect(self.update_position_state_plot)
            self.plot_timer.start(200)
        except Exception as e:
            print("udp_connect에서 예외 발생:")
            traceback.print_exc()
    def udp_disconnect(self):
        try:
            self.myunitree_b1.disconnect()
            h1 = Tread1(self)
            h1.start()
        except Exception as e:
            print("udp_disconnect에서 예외 발생:")
            traceback.print_exc()

    def update_label(self):
        self.SOC_label.setText("{:.1f}".format(self.data_SOC))
        self.Mode_label.setText("{:.1f}".format(self.data_mode))
        self.GaitType_label.setText("{:.1f}".format(self.data_gaitType))
        self.State_Position_0_label.setText("{:.1f}".format(self.data_position_hstate[0]))
        self.State_Position_1_label.setText("{:.1f}".format(-self.data_position_hstate[1]))
        self.BQ_NTC_label.setText("{:.1f}".format(self.data_BQ_NTC[0])) # 8.0
        self.MCU_NTC_label.setText("{:.1f}".format(self.data_MCU_NTC[0])) # 12.0

        if self.myunitree_b1.connect_flag:
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
        colors = np.array(['red' if meas[2] < 300 else 'grey' for meas in scan])
        polar_ax.scatter(offsets[:, 0], offsets[:, 1], s=5, color=colors, lw=0)

        self.detect_obstacles(scan)
        self.slam_canvas.draw()

    def detect_obstacles(self, scan):
        close_points = [meas for meas in scan if meas[2] < 300]
        clusters = []
        current_cluster = []

        for point in close_points:
            if not current_cluster or abs(point[1] - current_cluster[-1][1]) < 15:
                current_cluster.append(point)
            else:
                if len(current_cluster) >= 5:
                    clusters.append(current_cluster)
                current_cluster = [point]
        if len(current_cluster) >= 5:
            clusters.append(current_cluster)

        for cluster in clusters:
            avg_angle = np.mean([meas[1] for meas in cluster])
            avg_distance = np.mean([meas[2] for meas in cluster])
            direction = self.determine_direction(avg_angle)
            print(f"장애물 감지: 방향 {direction}, 평균 거리 {avg_distance}mm")

            if direction == "앞쪽":
                self.myunitree_b1.update_obstacle_state("front", True)
            elif direction == "오른쪽":
                self.myunitree_b1.update_obstacle_state("right", True)
            elif direction == "뒤쪽":
                self.myunitree_b1.update_obstacle_state("back", True)
            elif direction == "왼쪽":
                self.myunitree_b1.update_obstacle_state("left", True)

    def determine_direction(self, angle):
        if 20 <= angle <= 160:
            return "오른쪽"
        elif 160 < angle <= 200:
            return "뒤쪽"
        elif 200 < angle <= 340:
            return "왼쪽"
        else:
            return "앞쪽"

    def start_lidar_thread(self):
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