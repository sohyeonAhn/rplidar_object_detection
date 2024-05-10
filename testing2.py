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

        self.AutoMode_flag = False

        # 키보드 상태 트래킹
        self.pressed_keys = {
            'w': False,
            's': False,
            'a': False,
            'd': False
        }

        # ------ 버튼 -----------------------------------------------------
        self.connect_btn.clicked.connect(self.udp_connect)      # 통신 연결 버튼
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
        keyboard.on_press_key("a",  lambda _: self.set_key('a', True, self.Left_btn,
                                                           "background-color: rgb(172, 206, 255);"))
        keyboard.on_release_key("a", lambda _: self.set_key('a', False, self.Left_btn,
                                                            "background-color: rgb(255, 255, 255);"))
        keyboard.on_press_key("d", lambda _: self.set_key('d', True, self.Right_btn,
                                                          "background-color: rgb(172, 206, 255);"))
        keyboard.on_release_key("d", lambda _: self.set_key('d', False, self.Right_btn,
                                                            "background-color: rgb(255, 255, 255);"))

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
        self.data_position_hstate = self.myunitree_go1.hstate_position

        self.data_velocity = self.myunitree_go1.hstate_velocity
        print(f"State Velocity: {self.data_velocity}")

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
        self.myunitree_go1.Robot_force_Stop()

    def set_key(self, key, value, button, style):
        self.pressed_keys[key] = value
        button.setStyleSheet(style if value else "background-color: rgb(255, 255, 255);")
        self.update_movement()

    def update_movement(self):
        key_input_vel0 = 0
        key_input_vel1 = 0
        if self.pressed_keys['w']:
            key_input_vel0 = self.velocity_0_Front_value
        if self.pressed_keys['s']:
            key_input_vel0 = -self.velocity_0_Back_value
        if self.pressed_keys['a']:
            key_input_vel1 = -self.velocity_1_Left_value
        if self.pressed_keys['d']:
            key_input_vel1 = self.velocity_1_Right_value

        # 현재 움직임 상태 업데이트
        self.move_velocity_0_value = key_input_vel0
        self.move_velocity_1_value = key_input_vel1
        self.myunitree_go1.Move_mult(self.move_velocity_0_value, self.move_velocity_1_value)


    # ------ 콤보 박스 메소드 --------------
    def Change_mode_combobox(self, index):
        selected_item = self.Mode_ComboBox.currentText()
        print(f"Selected Mode: {selected_item}")

        if selected_item == "IDLE (0)":
            self.myunitree_go1.Change_Mode_to_IDLE()
        elif selected_item == "Force Stand (1)":
            self.myunitree_go1.Change_Mode_to_Force_Stand()
        # elif selected_item == "Vel Walk (2)":
        # self.myunitree_go1.Change_Mode_to_VEL_WALK()
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
            self.plot_timer = QTimer(self)
            self.plot_timer.timeout.connect(self.update_position_state_plot)
            self.plot_timer.start(200)
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
        self.State_Position_0_label.setText("{:.1f}".format(self.data_position_hstate[0]))
        self.State_Position_1_label.setText("{:.1f}".format(-self.data_position_hstate[1]))
        self.BQ_NTC_label.setText("{:.1f}".format(self.data_BQ_NTC[0])) # 8.0
        self.MCU_NTC_label.setText("{:.1f}".format(self.data_MCU_NTC[0])) # 12.0

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
                self.myunitree_go1.update_obstacle_state("front", True)
                self.myunitree_go1.Move_Stop()
            elif direction == "오른쪽":
                self.myunitree_go1.update_obstacle_state("right", True)
            elif direction == "뒤쪽":
                self.myunitree_go1.update_obstacle_state("back", True)
            elif direction == "왼쪽":
                self.myunitree_go1.update_obstacle_state("left", True)

    def determine_direction(self, angle):
        if 20 <= angle <= 160:
            return "오른쪽"
        elif 160 < angle <= 200:
            return "뒤쪽"
        elif 200 < angle <= 340:
            return "왼쪽"
        else:
            return "앞쪽"

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

