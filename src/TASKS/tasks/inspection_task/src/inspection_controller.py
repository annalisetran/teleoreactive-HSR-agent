#! /usr/bin/env python3

import rospy
from std_msgs.msg import String, Int8
from geometry_msgs.msg import PoseStamped
from speech_to_text.msg import speech_recognition
from tmc_msgs.msg import Voice

import sys
import os
sys.path.remove('/home/robocupathome/.local/lib/python3.8/site-packages')
sys.path.append('/home/robocupathome/workspace/temp_ws/src/world_model/world_model_kb/scripts')
from dbase import DBase
from regions_api import RegionsApi
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class inspection_controller:
    def __init__(self) -> None:
        self.door_sub = rospy.Subscriber('/door_open_close', String, callback=self.doors_callback, queue_size=10)
        self.move_base_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.vosk_sub = rospy.Subscriber('speech_recognition/vosk_result', speech_recognition, callback=self.vosk_callback, queue_size=10)
        self.wrist_sub = rospy.Subscriber('/start_task', Int8, callback=self.wrist_callback, queue_size=10)
        self.talker = rospy.Publisher("/talk_request", Voice, queue_size=10)
        self.door = None
        self.moved = False
        self.exit_word_heard = False
        self.task_started = False
        self.opening_line_said = False
        self.closing_line_said = False
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        
        
        
        
        db = DBase("/home/robocupathome/workspace/temp_ws/src/world_model/world_model_kb/scripts/database.ini")
        region_api = RegionsApi()
        inspection_point_from_db = region_api.get_poi_by_name(db.con_pool, "inspection point")[0]
        exit_point_from_db = region_api.get_poi_by_name(db.con_pool, "door2")[0]

        # self.start = PoseStamped()
        # self.start.pose.position.x = -4.544942855834961
        # self.start.pose.position.y = 0.3288428783416748
        # self.start.pose.position.z = 0
        # self.start.pose.orientation.w = 1
        # self.start.header.frame_id = "map"
        
        self.dest = PoseStamped()
        self.dest.pose.position.x = inspection_point_from_db['loc_x']
        self.dest.pose.position.y = inspection_point_from_db['loc_y']
        self.dest.pose.position.z = inspection_point_from_db['loc_z']
        self.dest.pose.orientation.w = 1
        self.dest.header.frame_id = "map"
        
        
        self.exit = PoseStamped()
        self.exit.pose.position.x = exit_point_from_db['loc_x']
        self.exit.pose.position.y = exit_point_from_db['loc_y']
        self.exit.pose.position.z = exit_point_from_db['loc_z']
        self.exit.pose.orientation.w = 1
        self.exit.header.frame_id = "map"
        
        
        """
        start
        x: -3.379775047302246
        y: -1.7274247407913208
        z: 0.0070323944091796875
        
        inside start
        x: -4.544942855834961
        y: 0.3288428783416748
        z: 0.2587261199951172

        end
        x: -3.5152721405029297
        y: 2.3607101440429688
        z: 0.0035686492919921875


        """

        self.finish_words = ['go', 'leave', 'done', 'finish', 'finished', 'all good', 'inspection finished' 'finished']

    def say(self, sentence: str):
        rospy.loginfo(f"saying: {sentence}")
        self.talker.publish(False, False, 1, sentence)

    def wrist_callback(self, msg: Int8):
        rospy.loginfo("wrist pressed")
        self.task_started = True
        if not self.opening_line_said:
            self.say("I am ready for inspection. please open the door!")          
            self.opening_line_said = True


    def vosk_callback(self, msg: speech_recognition):
        if not self.moved:
            return

        for word in msg.final_result.split():
            print(f"{word}, {self.finish_words}")
            if word in self.finish_words:
                print("word heard!")
                self.exit_word_heard = True


    def doors_callback(self, msg: String):
        self.door = msg.data
        
    def run(self):
        while (True):
            if self.door == None or not self.task_started:
                continue

            if not self.moved and self.door != None:
                self.say("Ah, thank you for opening the door! I will now move to the inspection point")
                
                #self.move_base_pub.publish(self.dest)
                
                goal = MoveBaseGoal()
                goal.target_pose = self.dest
                goal.target_pose.header.stamp = rospy.Time.now()
                self.client.send_goal_and_wait(goal)
                self.say("you can inspect now! let me know when you are finished")
                self.moved = True
                rospy.loginfo("moved to inspection point")    

            
            if self.moved and self.exit_word_heard and not self.closing_line_said:
                self.say("Thanks for the positive inspection! I will be on my way now. Have a nice day!")
                goal = MoveBaseGoal()
                goal.target_pose = self.exit
                goal.target_pose.header.stamp = rospy.Time.now()
                self.client.send_goal_and_wait(goal)
                self.closing_line_said = True
    


if __name__ == "__main__":
    rospy.init_node('inspection_controller')
    inspection_controller_node = inspection_controller()
    while not rospy.is_shutdown(): 
        inspection_controller_node.run()
