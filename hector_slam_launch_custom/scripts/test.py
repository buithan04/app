#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import math

saved = False

def scan_callback(msg):
    global saved
    if saved:
        return

    points = []
    angle = msg.angle_min

    for r in msg.ranges:
        if r > msg.range_min and r < msg.range_max:
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            points.append((x, y))

        if len(points) >= 100:
            break

        angle += msg.angle_increment

    with open("100_scan_points.txt", "w") as f:
        for p in points:
            f.write(f"{p[0]} {p[1]}\n")

    print("✅ Đã lưu 100 point đầu tiên vào 100_scan_points.txt")
    saved = True
    rospy.signal_shutdown("Done")

rospy.init_node("save_100_scan_points")
rospy.Subscriber("/scan", LaserScan, scan_callback)
rospy.spin()
