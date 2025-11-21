#!/usr/bin/env python

from db import Db
from robot_state_api import RobotStateApi
from object_def_api import ObjectDefApi
from object_api import ObjectApi
from scene_api import SceneApi
from object_class_api import ObjectClassApi

from spatial_visual_system.msg import Detection

import rospy
import json

class SvSInterface:
    __debug = False
    def __init__(self):
        rospy.init_node('world_model_kb')

        self.__getParams()

        rospy.loginfo("Setting up svs_interface...")
        # Setup connection to DB
        self.__db = Db(self.__db_config_path)
        rospy.loginfo("Connected to DB!")

        self.__robot_state = RobotStateApi()
        self.__obj_def = ObjectDefApi()
        self.__obj = ObjectApi()
        self.__scene_api = SceneApi()
        self.__obj_class_api = ObjectClassApi()
        rospy.loginfo("\tConstructed all DB API's")

        self.__detection_sub = rospy.Subscriber("detections", 
                Detection, self.__detection_cb)

        self.__newest_scene_id = -1
        self.__curr_db_scene_id = -1
        rospy.loginfo("svs_interface ready to go!")

    def __getParams(self):
        self.__db_config_path = rospy.get_param("~database_config_path")

    def __detection_cb(self, detection_msg):
        if self.__debug:
            rospy.loginfo("Inserting " + str(detection_msg))
        scene_id = detection_msg.scene_id
        if (scene_id > self.__newest_scene_id or self.__curr_db_scene_id is None):
            self.create_new_scene(scene_id)

        data_json = json.loads(detection_msg.json)
        data_json.update({'scene_id': self.__curr_db_scene_id})

        inserted_id = self.__scene_api.insert_scene_object(
                self.__db.con_pool, json.dumps([data_json]), self.__curr_db_scene_id)

        if inserted_id == -1:
            rospy.logwarn("Failed to insert scene object into the DB: " +
                    str(data_json))

    def create_new_scene(self, scene_id):
        if self.__debug:
            rospy.loginfo("Creating new scene " + str(scene_id))
        robot_state_id = self.create_robot_state()
        if robot_state_id == -1:
            rospy.logwarn("Failed to insert new robot state into the DB:")

        inserted_scene_id = self.__scene_api.insert_scene(
                self.__db.con_pool, robot_state_id)

        if inserted_scene_id == -1:
            rospy.logwarn("Failed to insert new scene (id " + str(scene_id) +
                    ") into the DB")
        else:
            self.__newest_scene_id = scene_id
            self.__curr_db_scene_id = inserted_scene_id
        

    def create_robot_state(self):
        robot_state = {
            "base_pose_x": 0,
            "base_pose_y": 0,
            "base_pose_z": 0,
            "base_pose_w": 0,
            "base_r": 0,
            "base_p": 0,
            "base_yw": 0,
            "head_pose_x": 0,
            "head_pose_y": 1,
            "head_pose_z": 2,
            "head_pose_w": 3,
            "head_r": 0.2,
            "head_p": 0.1,
            "head_yw": 0.1,
            "driveable_state": 1,
            "arm_state": 0,
            "shoulder_state": 0,
            "gripper_state": 0,
            "frame_id": "odom",
            "holding_object_id": None}
        return self.__robot_state.insert_robot_state(
                self.__db.con_pool, json.dumps([robot_state]))


if __name__ == "__main__":
    svs_interface = SvSInterface()
    rospy.spin()
