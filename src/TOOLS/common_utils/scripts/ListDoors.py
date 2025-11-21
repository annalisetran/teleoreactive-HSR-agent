#! /usr/bin/env python3
import os
import rospy
from utility.msg import DBDoor, DoorList
import json
import sys
sys.path.remove('/home/robocupathome/.local/lib/python3.8/site-packages')
sys.path.append('/home/robocupathome/workspace/robocup_ws/src/world_model/world_model_store/scripts')
from db import Db
from object_api import ObjectApi



class ListDoors:
    def __init__(self) -> None:
        self.doors_pub = rospy.Publisher('door_list', DoorList, queue_size=10)
        self.db = Db("/home/robocupathome/workspace/robocup_ws/src/world_model/world_model_store/scripts/database.ini")
        self.obj = ObjectApi()
    
    def publish_doors(self):
        doorListJSON = self.obj.get_objects_of_class_list(self.db.con_pool, "door")
        doorListMsg = DoorList()
        doorListMsg.doors = []
        for door in doorListJSON:
            newDoorMsg = DBDoor()
            newDoorMsg.name = door["name"]
            newDoorMsg.pose.position.x = door["pose_x"]
            newDoorMsg.pose.position.y = door["pose_y"]
            newDoorMsg.pose.position.z = door["pose_z"]
            newDoorMsg.pose.orientation.x = door["orient_x"]
            newDoorMsg.pose.orientation.y = door["orient_y"]
            newDoorMsg.pose.orientation.z = door["orient_z"]
            newDoorMsg.pose.orientation.w = door["orient_w"]
            doorListMsg.doors.append(newDoorMsg)
        # print("publishing doors: ",str(doorListJSON))
        self.doors_pub.publish(doorListMsg)


if __name__ == "__main__":
    rospy.init_node('ListDoors')
    doorListNode = ListDoors()
    while not rospy.is_shutdown():
        doorListNode.publish_doors()
        rospy.sleep(2)

