#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import quaternion_from_euler
import csv
import os

class PoseMarkerPublisher:
    def __init__(self):
        rospy.init_node('pose_marker_publisher')
        self.pub = rospy.Publisher('/visualization_marker', Marker, queue_size=100)
        self.file_path = "labeled_poses.txt"

        rospy.sleep(1)  # wait for RViz to subscribe

        self.publish_all_markers()
        rospy.loginfo("Published all pose markers.")
        rospy.spin()

    def publish_all_markers(self):
        if not os.path.exists(self.file_path):
            rospy.logerr(f"File {self.file_path} not found.")
            return

        with open(self.file_path, 'r') as f:
            reader = csv.reader(f)
            for idx, row in enumerate(reader):
                if len(row) != 4:
                    rospy.logwarn(f"Skipping malformed line: {row}")
                    continue
                label, x, y, yaw = row
                x, y, yaw = float(x), float(y), float(yaw)

                self.publish_text_marker(label, x, y, idx)
                self.publish_arrow_marker(x, y, yaw, idx + 1000)

    def publish_text_marker(self, label, x, y, marker_id):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "pose_labels"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 1.0  # elevate label
        marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0

        marker.text = label
        self.pub.publish(marker)

    def publish_arrow_marker(self, x, y, yaw, marker_id):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "pose_arrows"
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0

        q = quaternion_from_euler(0, 0, yaw)
        marker.pose.orientation = Quaternion(*q)

        marker.scale.x = 0.5  # arrow length
        marker.scale.y = 0.05  # arrow width
        marker.scale.z = 0.05

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.pub.publish(marker)

if __name__ == '__main__':
    try:
        PoseMarkerPublisher()
    except rospy.ROSInterruptException:
        pass
