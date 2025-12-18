#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import math
import matplotlib.pyplot as plt
import threading

xs, ys = [], []
lock = threading.Lock()

def scan_callback(msg):
    global xs, ys
    _xs, _ys = [], []
    angle = msg.angle_min

    for r in msg.ranges:
        if msg.range_min < r < msg.range_max:
            _xs.append(r * math.cos(angle))
            _ys.append(r * math.sin(angle))
        angle += msg.angle_increment

    with lock:
        xs = _xs
        ys = _ys

def main():
    rospy.init_node("lidar_plotter", anonymous=True)
    rospy.Subscriber("/scan", LaserScan, scan_callback)

    plt.ion()
    fig, ax = plt.subplots()
    sc = ax.scatter([], [], s=5)

    ax.set_aspect('equal')
    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)
    ax.set_title("2D LiDAR Map")
    ax.grid(True)

    rate = rospy.Rate(10)  # 10 Hz refresh

    while not rospy.is_shutdown():
        with lock:
            sc.set_offsets(list(zip(xs, ys)))

        plt.draw()
        plt.pause(0.001)
        rate.sleep()

if __name__ == "__main__":
    main()
