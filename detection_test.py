#!/usr/bin/env python3
'''Animates distances and measurment quality'''
from rplidar import RPLidar
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import time


# LiDAR 센서에 연결할 포트 설정
PORT_NAME = 'COM3'
# 거리 측정의 최대 범위를 설정
DMAX = 1000 #4000mm(4m)로 설정
# 측정된 반사 강도의 최소값을 설정
IMIN = 0
# 측정된 반사 강도의 최대값을 설정
IMAX = 50

def update_line(num, iterator, line):
    scan = next(iterator)
    offsets = np.array([(np.radians(meas[1]), meas[2]) for meas in scan])
    # 모든 점의 반사 강도를 기본적으로 IMIN으로 설정합니다.
    colors = []  # 색상을 저장할 리스트

    for meas in scan:
        # 거리가 200mm 이하면 빨간색, 그렇지 않으면 회색조로 설정
        if meas[2] <= 200:
            colors.append('red')  # 빨간색
        else:
            colors.append('grey')  # 기본색 (회색조)

    line.set_offsets(offsets)
    line.set_color(colors)  # 각 점의 색상을 설정

    return line,

def run():
    lidar = RPLidar(PORT_NAME, baudrate=1000000)
    fig = plt.figure()
    ax = plt.subplot(111, projection='polar')
    # 0도를 맨 위로 설정
    ax.set_theta_zero_location('N')
    # 각도가 시계 방향으로 증가하도록 설정
    ax.set_theta_direction(-1)

    line = ax.scatter([0, 0], [0, 0], s=5, c=[IMIN, IMAX], cmap=plt.cm.Greys_r, lw=0)
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