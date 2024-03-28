#!/usr/bin/env python3
'''Animates distances and measurement quality'''
from rplidar import RPLidar
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation

# LiDAR 센서에 연결할 포트 설정
PORT_NAME = 'COM3'
# 거리 측정의 최대 범위를 설정
DMAX = 1000  # 1000mm(1m)로 설정
# 측정된 반사 강도의 최소값을 설정
IMIN = 0
# 측정된 반사 강도의 최대값을 설정
IMAX = 50

def update_line(num, iterator, line):
    scan = next(iterator)
    print(scan)
    # 각도와 거리 데이터 추출 및 변환
    offsets = np.array([(np.radians(meas[1]), meas[2]) for meas in scan])
    line.set_offsets(offsets)

    # 거리에 따라 색상 결정
    colors = np.array(['red' if meas[2] < 200 else 'grey' for meas in scan])
    line.set_color(colors)  # 점의 색상을 업데이트
    return line,

def run():
    lidar = RPLidar(PORT_NAME, baudrate=1000000)
    fig = plt.figure()
    ax = plt.subplot(111, projection='polar')
    # 0도를 위쪽으로 설정하고, 각도 방향을 시계 방향으로 설정
    ax.set_theta_zero_location('N')
    ax.set_theta_direction(-1)

    line = ax.scatter([0, 0], [0, 0], s=5, lw=0)
    ax.set_rmax(DMAX)
    ax.grid(True)

    iterator = lidar.iter_scans()
    ani = animation.FuncAnimation(fig, update_line, fargs=(iterator, line), interval=100)
    plt.show()

    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()

if __name__ == '__main__':
    run()
