#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def create_polygon_marker(polygon_points, frame_id="map", marker_id=0):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "polygon"
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.1  # Line width

    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    marker.pose.orientation.w = 1.0

    for pt in polygon_points:
        p = Point()
        p.x = pt[0]
        p.y = pt[1]
        p.z = 0.0
        marker.points.append(p)

    # Close the polygon by repeating the first point
    if polygon_points[0] != polygon_points[-1]:
        p = Point()
        p.x = polygon_points[0][0]
        p.y = polygon_points[0][1]
        p.z = 0.0
        marker.points.append(p)

    return marker

def main():
    rospy.init_node("polygon_marker_publisher")
    pub = rospy.Publisher("polygon_marker", Marker, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    # Example polygon (square)
    polygon_2d = [
        [11.193, -1.094], [10.238, -0.737], [8.89, -0.217], [7.997, 0.197],
        [6.572, 0.782], [6.974, 1.902], [7.437, 2.899], [7.735, 3.7177],
        [7.990, 4.348], [8.992, 4.038], [9.683, 3.728], [10.291, 3.483],
        [10.7503, 3.335], [11.463, 2.992], [12.358, 2.655], [12.614, 2.547],
        [12.32, 1.7], [12.486, 2.181], [12.078, 1.105], [11.95, 0.548],
        [11.796, 0.342], [11.756, 0.189], [11.538, -0.454]
    ]

    while not rospy.is_shutdown():
        marker = create_polygon_marker(polygon_2d)
        pub.publish(marker)
        rate.sleep()

if __name__ == "__main__":
    main()
