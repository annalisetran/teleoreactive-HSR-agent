#!/usr/bin/env python3

import rospy
import math
import json

import smach
import smach_ros
import actionlib

import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import PoseStamped, PointStamped, Point, Pose
from std_msgs.msg import String, Int8, Bool, Float64
from unsw_action_msg.msg import UNSWActionMsg, UNSWActionResult
from unsw_vision_msgs.msg import DetectionList
from unsw_vision_msgs.srv import FindObjectFrame
from visualization_msgs.msg import Marker

from action_server.msg import ManipViewPoseAction, ManipViewPoseGoal, ManipViewPoseResult

import os
import sys
module_path = os.environ.get("UNSW_WS")
sys.path.append(module_path + "/WORLD_MODEL/world_model/world_model_kb/scripts")
print(sys.path)

object_category_dict = {
    "banana": "fruit",
    "apple": "fruit",
    "cornflakes": "food",
    "mayonaise": "food"
}

table_pose = Pose()
table_pose.position.x = 0.990
table_pose.position.y = -0.186
table_pose.position.z = 0.000
table_pose.orientation.x = 0.000
table_pose.orientation.y = 0.000
table_pose.orientation.z = 0.496
table_pose.orientation.w = 0.868

table_arm_lift_joint = 0.44
table_arm_flex_joint = -0.48
table_head_pan_joint = -1.60
table_head_tilt_joint = -0.56

class DefineShelves():
    def __init__(self):
        self.plane_marker_dict = {}
        rospy.Subscriber('/detected_plane', Marker, self.plane_detection_callback, queue_size=1)
        self.wait = 0.0
        # self.client = actionlib.SimpleActionClient("manip_view_pose", ManipViewPoseAction)

    def plane_detection_callback(self, msg: Marker):
        self.plane_marker_dict[msg.id] = {"marker": msg, "categories": []}
    
    # TODO: Go through the three different poses, find all planes + detection list for each, define shelves using these two
    def execute(self):
        rospy.loginfo("Define shelves executing")

        # TODO: Define first cabinet viewing pose
        # self.goal = ManipViewPoseGoal()
        # self.goal.arm_lift_joint = table_arm_lift_joint
        # self.goal.arm_flex_joint = table_arm_flex_joint
        # self.goal.head_pan_joint = table_head_pan_joint
        # self.goal.head_tilt_joint = table_head_tilt_joint

        # self.client.send_goal(self.goal)
        # self.client.wait_for_result()

        # rospy.sleep(self.wait)
        detection_list_init = rospy.wait_for_message('/unsw_vision/detections/objects', DetectionList)
        plane_dict_init = self.plane_marker_dict.copy()


        # TODO: Define second cabinet viewing pose
        # self.goal = ManipViewPoseGoal()
        # self.goal.arm_lift_joint = table_arm_lift_joint
        # self.goal.arm_flex_joint = table_arm_flex_joint
        # self.goal.head_pan_joint = table_head_pan_joint
        # self.goal.head_tilt_joint = table_head_tilt_joint

        # self.client.send_goal(self.goal)
        # self.client.wait_for_result()

        # rospy.sleep(self.wait)
        detection_list = rospy.wait_for_message('/unsw_vision/detections/objects', DetectionList)
        plane_dict = self.plane_marker_dict.copy()

        plane_list = sorted(
            list(plane_dict_init.values()) + list(plane_dict.values()),
            key=lambda m: m.marker.pose.position.z
        )
        
        shelf_dict = {index: value for index, value in enumerate(plane_list)}

        sorted_markers = sorted(shelf_dict.items(), key=lambda x: x[1].marker.pose.position.z)

        detections = list(detection_list_init.objects) + list(detection_list.objects)

        for object_detection in detections:
            object_z = object_detection.position.z
            best_shelf = None
            best_z = 0

            for i, marker in sorted_markers:
                marker_z = marker.marker.pose.position.z
                if marker_z < object_z and marker_z > best_z:
                    best_z = marker_z
                    best_shelf = i
            
            if best_shelf is not None:
                obj_class = object_detection.object_class
                if obj_class in object_category_dict:
                    category = object_category_dict[obj_class]
                    if category not in shelf_dict[best_shelf]['categories']:
                        shelf_dict[best_shelf]['categories'].append(category)

        current_cabinet = shelf_dict
        # create set of all the categories currently on shelves
        shelf_category_list = []
        for shelf in current_cabinet.values():
            shelf_category_list.extend(shelf.categories)
        categories_set = set(shelf_category_list)

def main(args):
    rospy.init_node('DefineShelves', anonymous=True)
    define_shelves = DefineShelves()
    define_shelves.execute()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)