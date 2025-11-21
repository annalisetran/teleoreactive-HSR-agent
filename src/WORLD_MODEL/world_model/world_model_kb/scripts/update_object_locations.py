#!/usr/bin/env python3

import rospy
from dbase import DBase
from unsw_vision_msgs.msg import PersonDetection, DetectionList, BoundingBox
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2 
import message_filters
from itertools import islice
"""
This node is for updating object locations in the database.

how it works: it checks the database for all objects that are
within the current scene. For each database object, check if 
it is visible using the pointcloud. if it is visible (ie the pointcloud
distance where the object would be is >= the predicted distance of the 
object), remove the object.

Note: Tracker node does not add objects if they are too close to original position.
Must check detectionlist to make sure not removing objects that have been detected.

How to do it: look at top left and bottom right corner of pointcloud.
get vector to those. trace vectors out to max range of pointcloud camera.
This is bbounding box of pointcloud
"""

class UpdateObjectLocations:
    
    def __init__(self) -> None:
        self.detections_topic = rospy.get_param('~detections_topic')
        self.pointcloud_topic = rospy.get_param('~pointcloud_topic')
        self.camera_range = rospy.get_param('~camera_range')
        self.detections_sub = message_filters.Subscriber(self.detections_topic, DetectionList)
        self.pointcloud_sub = message_filters.Subscriber(self.pointcloud_topic, PointCloud2)

        self.synchronized_sub = message_filters.ApproximateTimeSynchronizer(
            [self.pointcloud_sub, self.detections_sub], queue_size=10, slop=0.2
        )
        self.synchronized_sub.registerCallback(self.track_objects)

    def track_objects(self, pointcloud: PointCloud2, detections: DetectionList):
        rospy.loginfo(f'updating objects')
        points = pc2.read_points(pointcloud)
        # might not work
        firstPoint = next(points)
        lastPoint = next(islice(points, pointcloud.height * pointcloud.width - 2, None))
        rospy.loginfo(f'lastpoint: {lastPoint}, firstPoint: {firstPoint}, points: {points}')
        # get camera values from robot docs, hardcode / put in launch file

if __name__ == "__main__":
    try:
        rospy.init_node('scene_manager_node')
        u = UpdateObjectLocations()
        rospy.spin()
    except (KeyboardInterrupt, rospy.ROSInterruptException) as e:
            rospy.logfatal("Error occurred! Stopping the update object locations node...")
            rospy.sleep(1)
            print("Update_object_locations_node terminated")