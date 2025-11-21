#!/usr/bin/env python3

import rospy    
from tmc_msgs.msg import Voice
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from unsw_vision_msgs.msg import PersonDetection, DetectionList
from task_msgs.msg import intro
import sys
import os
sys.path.remove('/home/robocupathome/.local/lib/python3.8/site-packages')
sys.path.append('/home/robocupathome/workspace/robocup_ws/src/world_model/world_model_store/scripts')
from dbase import DBase
from agent_api import AgentApi


class introduce_guests:
    def __init__(self):
        
        # Subscribers
        self.start_introduction = rospy.Subscriber('/introduce_guests', intro, callback=self.introduction, queue_size=1 )
        self.vision_sub = rospy.Subscriber('/unsw_vision/detections/objects/coordinates', DetectionList, self.vision_callback, queue_size=10)
        
        # Publishers
        self.talker = rospy.Publisher("/talk_request", Voice, queue_size=10)
        self.look_at_pub = rospy.Publisher("/look_at", PoseStamped, queue_size=10)
        self.finished_pub = rospy.Publisher("/introduction_finished", Bool, queue_size=10)

        self.information_found = False
        self.target_id = None
        self.db = DBase(os.path.abspath('/home/robocupathome/workspace/robocup_ws/src/world_model/world_model_store/scripts'))
      # Need to interact with the database. Need to get the person ID and then store correct name and drink   
        
    def introduction(self, msg: intro):
        # check the script can start
        self.target_id = msg.person2
        person1_name = AgentApi.get_agent_name(self.db.con_pool, msg.person1) # check right function name
        person2_name = AgentApi.get_agent_name(self.db.con_pool, msg.person2) # check right function name
        person1_drink = AgentApi.get_agent_attribute(self.db.con_pool, msg.person1, 1) # check right function name and inputs
        self.say(f"Hello {person2_name}, this is my friend, {person1_name}. Their favourite drink is {person1_drink}!")
        self.target_id = None
        self.finished_pub.publish(Bool(True))



    def vision_callback(self, msg: DetectionList):
        if self.target_id == None:
            return
        
        personPose = PoseStamped()
        personPose.header.frame_id = "map"
        personPose.header.stamp = rospy.get_rostime()
        personDetection = next(filter(lambda x: x.id == self.target_id, msg.people))
        personPose.pose.position = personDetection.face_position
        self.look_at_pub.publish(personPose)


    def say(self, sentence: str):
        rospy.loginfo(f"saying: {sentence}")
        self.talker.publish(False, False, 1, sentence)

    
if __name__ == "__main__":
    rospy.init_node('introduce_guests')
    details_received_node = introduce_guests()
    while not rospy.is_shutdown():
        rospy.spin()


