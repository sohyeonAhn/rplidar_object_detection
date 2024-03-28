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
DMAX = 4000 #4000mm(4m)로 설정
# 측정된 반사 강도의 최소값을 설정
IMIN = 0
# 측정된 반사 강도의 최대값을 설정
IMAX = 50

def update_line(num, iterator, line): # 애니메이션의 각 프레임을 업데이트

    # scan 변수는 iterator로부터 다음 스캔 데이터를 가져옵니다.
    scan = next(iterator)
    print(scan)
    # scan 데이터에서 각도(라디안)와 거리를 추출하여 offsets 배열을 생성
    offsets = np.array([(np.radians(meas[1]), meas[2]) for meas in scan])
    # scatter plot의 위치를 업데이트
    line.set_offsets(offsets)
    # scan 데이터에서 반사 강도를 추출하여 intens 배열을 생성
    intens = np.array([meas[0] for meas in scan])
    # scatter plot의 색상을 반사 강도에 따라 업데이트
    line.set_array(intens)
    return line,

def run():
    """
    Main function to set up the LiDAR sensor and run the animation.
    """
    # RPLidar 객체를 생성하고, 포트 이름과 바우드레이트를 설정합니다.
    lidar = RPLidar(PORT_NAME, baudrate=1000000)
    # matplotlib figure를 생성합니다.
    fig = plt.figure()
    # 극좌표계를 사용하는 subplot을 생성합니다.
    ax = plt.subplot(111, projection='polar')
    # 초기 scatter plot을 생성합니다. 위치와 색상은 임시값으로 설정됩니다.
    line = ax.scatter([0, 0], [0, 0], s=5, c=[IMIN, IMAX], cmap=plt.cm.Greys_r, lw=0)
    # plot의 최대 반지름을 DMAX로 설정합니다.
    ax.set_rmax(DMAX)
    # 격자를 표시합니다.
    ax.grid(True)

    # LiDAR 센서에서 스캔 데이터를 반복적으로 읽는 iterator를 생성합니다.
    iterator = lidar.iter_scans()
    # matplotlib 애니메이션을 생성하고, update_line 함수를 사용하여 각 프레임을 업데이트합니다.
    ani = animation.FuncAnimation(fig, update_line, fargs=(iterator, line), interval=100)
    # 그래프를 표시합니다.
    plt.show()

    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()

if __name__ == '__main__':
    run()
