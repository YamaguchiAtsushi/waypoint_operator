#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import csv

# CSVファイルの初期化
csv_file = '/home/ubuntu/catkin_ws/src/waypoint_operator/csv/waypoints.csv'
with open(csv_file, 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["x", "y", "z"])

def waypoint_callback(data):
    # ウェイポイントのログ表示
    rospy.loginfo("Received waypoint: (%f, %f, %f)", data.pose.position.x, data.pose.position.y, data.pose.position.z)
    
    # ウェイポイントをCSVファイルに書き込む
    with open(csv_file, 'a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([data.pose.position.x, data.pose.position.y, data.pose.position.z])

def waypoint_listener():
    rospy.init_node('waypoint_subscriber', anonymous=True)
    rospy.Subscriber("/robot_parent/move_base_simple/goal", PoseStamped, waypoint_callback)
    rospy.spin()

if __name__ == '__main__':
    waypoint_listener()