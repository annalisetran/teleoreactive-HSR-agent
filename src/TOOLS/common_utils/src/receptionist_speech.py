#!/usr/bin/env python3

import rospy    
from tmc_msgs.msg import Voice
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from ros_vosk.msg import speech_recognition
from unsw_vision_msgs.msg import PersonDetection, DetectionList
import math
import tf
import sys
import os
sys.path.remove('/home/robocupathome/.local/lib/python3.8/site-packages')
sys.path.append('/home/robocupathome/workspace/robocup_ws/src/world_model/world_model_store/scripts')
from dbase import DBase
from agent_api import AgentApi


class receptionist_speech:
    def __init__(self):
        
        # Subscribers
        self.vosk_subscriber = rospy.Subscriber('/speech_recognition/vosk_result', speech_recognition, callback=self.vosk_callback, queue_size=1)
        self.start_details_collection = rospy.Subscriber('/start_details', Bool, callback=self.details_collection, queue_size=1 )
        self.vision_sub = rospy.Subscriber('/unsw_vision/detections/objects/coordinates', DetectionList, self.vision_callback, queue_size=10)
        
        # Publishers
        self.talker = rospy.Publisher("/talk_request", Voice, queue_size=10)
        self.details_received_pub = rospy.Publisher("/details_received", String, queue_size=1)
        self.look_at_pub = rospy.Publisher("/look_at", PoseStamped, queue_size=10)

        self.information_found = False
        self.affirmative_words = ["yes", "correct", "true", "yeah", "yis", "yay"]
        self.ongoing = False
        self.target_id = None
        self.tf_listener = tf.TransformListener()
        self.db = DBase(os.path.abspath('/home/robocupathome/workspace/robocup_ws/src/world_model/world_model_store/scripts'))
      # Need to interact with the database. Need to get the person ID and then store correct name and drink   
        
    def details_collection(self):
        # check the script can start
        self.ongoing = True
        self.find_name_drink()
        self.details_received_pub.publish(self.target_id)
        self.target_id = None
        self.ongoing = False


    def vision_callback(self, msg: DetectionList):
        if not self.ongoing:
            return

        if self.target_id == None:
            self.findClosestPerson(msg)
        
        personPose = PoseStamped()
        personPose.header.frame_id = "map"
        personPose.header.stamp = rospy.get_rostime()
        personDetection = next(filter(lambda x: x.id == self.target_id, msg.people))
        personPose.pose.position = personDetection.face_position
        self.look_at_pub.publish(personPose)


    def findClosestPerson(self, msg: DetectionList):
        shortest_dist = 99999
        for person in msg.people:
            personPose = PoseStamped()
            personPose.header.frame_id = "map"
            personPose.header.stamp = rospy.get_rostime()
            personPose.pose.position = person.position
            transformedPose = self.transform_pose(personPose, "base_link")
            dist = math.sqrt(transformedPose.pose.position.x ** 2 + transformedPose.pose.position.y ** 2)
            if  dist < shortest_dist:
                shortest_dist = dist
                self.target_id = person.id


    def transform_pose(self, pose, target_frame):
        self.tf_listener.waitForTransform(pose.header.frame_id, target_frame, pose.header.stamp, rospy.Duration(2))
        return self.tf_listener.transformPose(target_frame, pose)


    def say(self, sentence: str):
        rospy.loginfo(f"saying: {sentence}")
        self.talker.publish(False, False, 1, sentence)
        
        
    def vosk_callback(self, msg: speech_recognition):
        self.vosk_msg = msg.final_result
        
    
    def find_name_drink(self):
        # Check the person ID from world model here
        self.say("What is your name? ")
        self.name_count = 0
        while self.name_count < 3:
            if len(self.vosk_msg.split()) > 0:
                self.name = self.vosk_msg.split()[-1]
            self.say("Please confirm by saying yes or no. Is your name " + self.name)
            if self.check_affirmative(self.vosk_msg):
                self.name_count = 3
            else:
                self.say("Sorry, What is your name? ")
                self.name_count += 1
        # storing name in world model
        AgentApi.update_agent_name(self.db.con_pool, self.target_id, self.name)
        self.drink_count = 0
        self.say("Now I know your name " + self.name + "Please tell me your favourite drink ")
        while self.drink_count < 3:
            if len(self.vosk_msg.split()) > 0:
                self.drink = self.vosk_msg.split()[-1]
            self.say("Please confim by saying yes or no. Is your favourite drink " + self.drink)
            if self.check_affirmative(self.vosk_msg):
                self.drink_count = 3
            else:
                self.say("Sorry, what is your favourite drink? ")
                self.drink_count += 1
        AgentApi.update_agent_attribute(self.db.con_pool, self.target_id, 1, self.drink)


    def check_affirmative(self, words):
        for word in self.affirmative_words:
            if word in words:
                return True
        return False

    
if __name__ == "__main__":
    rospy.init_node('details_received')
    details_received_node = receptionist_speech()
    while not rospy.is_shutdown():
        rospy.spin()


