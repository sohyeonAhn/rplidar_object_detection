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
    offsets = np.array([(np.radians(meas[1]), meas[2]) for meas in scan])
    line.set_offsets(offsets)

    colors = np.array(['red' if meas[2] < 300 else 'grey' for meas in scan])
    line.set_color(colors)

    close_points = [meas for meas in scan if meas[2] < 300]

    clusters = []
    current_cluster = []

    for point in close_points:
        if not current_cluster:
            current_cluster.append(point)
        else:
            if abs(point[1] - current_cluster[-1][1]) < 15:
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
        direction = determine_direction(avg_angle)
        print(f"장애물 감지: 방향 {direction}, 평균 거리 {avg_distance}mm")

def determine_direction(angle):
    if 20 <= angle <= 160:
        return "오른쪽"
    elif 160 < angle <= 200:
        return "뒤쪽"
    elif 200 < angle <= 340:
        return "왼쪽"
    else:  # 340 < angle <= 360 or 0 <= angle < 20
        return "앞쪽"

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
