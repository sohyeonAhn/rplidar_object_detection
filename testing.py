# PyQt5와 관련 라이브러리를 임포트합니다.
import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5 import uic
import pyqtgraph as pg
from rplidar import RPLidar
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

PORT_NAME = 'COM3'
DMAX = 1000  # 최대 거리 설정 (mm)


class LidarThread(QThread):
    update_signal = pyqtSignal(list)  # 새로운 데이터가 있을 때 신호를 보내기 위한 시그널

    def __init__(self, lidar):
        super().__init__()
        self.lidar = lidar

    def run(self):
        for scan in self.lidar.iter_scans():
            self.update_signal.emit(scan)


class MyWindow(QMainWindow):
    def __init__(self, parent=None):
        super(MyWindow, self).__init__(parent)
        uic.loadUi('./sample2.ui', self)

        self.lidar = RPLidar(PORT_NAME)
        self.lidar.start_motor()

        self.slam_view = self.findChild(pg.PlotWidget, "slam_view")

        self.slam_figure = Figure()
        self.slam_canvas = FigureCanvas(self.slam_figure)
        self.slam_layout = QVBoxLayout(self.slam_view)
        self.slam_layout.addWidget(self.slam_canvas)

        self.start_lidar_thread()

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
