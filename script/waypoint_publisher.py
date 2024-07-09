#!/usr/bin/env python3

import rospy
import csv
from geometry_msgs.msg import PoseStamped



def read_waypoints(csv_file):
    waypoints = []
    with open(csv_file, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # ヘッダー行をスキップ
        for row in reader:
            x, y, z = map(float, row)
            waypoints.append((x, y, z))
    return waypoints

def publish_waypoints(waypoints, topic_name):
    pub = rospy.Publisher(topic_name, PoseStamped, queue_size=10)
    rospy.init_node('waypoint_publisher', anonymous=True)
    rate = rospy.Rate(1)  # 1Hz

    for waypoint in waypoints:
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = waypoint[0]
        goal.pose.position.y = waypoint[1]
        goal.pose.position.z = waypoint[2]
        goal.pose.orientation.w = 1.0

        rospy.loginfo("Publishing waypoint: (%f, %f, %f)", waypoint[0], waypoint[1], waypoint[2])
        pub.publish(goal)
        rate.sleep()

if __name__ == '__main__':
    try:
        csv_file = 'waypoints.csv'  # CSVファイルのパス
        topic_name = '/move_base_simple/goal'  # パブリッシュするトピック名
        waypoints = read_waypoints(csv_file)
        publish_waypoints(waypoints, topic_name)
    except rospy.ROSInterruptException:
        pass