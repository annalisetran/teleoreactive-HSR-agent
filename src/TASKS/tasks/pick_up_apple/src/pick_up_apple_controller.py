#!/usr/bin/env python3
from __future__ import annotations
import rospy    
from tmc_msgs.msg import Voice
from std_msgs.msg import String, Int8, Bool
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from nav_msgs.msg import Odometry
from speech_to_text.msg import speech_recognition
from actionlib_msgs.msg import GoalID
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from unsw_vision_msgs.msg import DetectionList, BagPointDetection

import actionlib
import tf2_ros
import math
import tf2_geometry_msgs
import sys
from visualization_msgs.msg import Marker
import datetime
import hsrb_interface

from abc import ABC, abstractmethod

class Context:
    _state = None
    # past_pose = None
    
    def __init__(self, state: State) -> None:
        
        self.task_started = False
        self.detection_list = None
        self.vosk_msg = None
        self.apple_point = None

        self.state_transitioned = False

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.pose_in_map = None
        
        # Subscribers
        self.vosk_sub = rospy.Subscriber('speech_recognition/vosk_result', speech_recognition, callback=self.vosk_callback, queue_size=1)
        self.vision_sub = rospy.Subscriber('/unsw_vision/detections/objects/positions', DetectionList, callback=self.vision_callback, queue_size=1)
        
        # db = DBase("/home/robocupathome/workspace/robocup_ws/src/world_model/world_model_store/scripts/database.ini")
        # region_api = RegionsApi()
        # self.inspection_point = region_api.get_poi_by_name(db.con_pool, "inspection point")[0]

        # Publishers
        self.apple_point_pub = rospy.Publisher('/pickup_point', PointStamped)
        self.move_head_pub = rospy.Publisher('/hsrb/head_trajectory_controller/command', JointTrajectory, queue_size=1)
        self.init_pose_pub = rospy.Publisher('/init_pose', String, queue_size=1)
        
        rospy.sleep(1)
        self.init_pose_pub.publish(String("init"))
            
        self.transition_to(state)
        rospy.loginfo('init content')

    def transition_to(self, state: State) -> None:
        self._state = state
        self.state_time = False
        self._state.context = self

    def say(self, sentence: str):
        rospy.loginfo(f"saying: {sentence}")
        #self.talker.publish(False, False, 1, sentence)

    def run(self):
        self._state.run()

    # Callbacks
    def vision_callback(self, msg: DetectionList):
        for apple in list(filter(lambda obj: obj.object_class == "apple", msg.objects)):
            print(apple)
            self.apple_point = Point()
            self.apple_point.x = apple.position.x
            self.apple_point.y = apple.position.y
            self.apple_point.z = apple.position.z

    
    def vosk_callback(self, msg: speech_recognition):
        self.vosk_msg = msg.final_result

        
class State(ABC):
    @property
    def context(self) -> Context:
        return self._context

    @context.setter
    def context(self, context: Context) -> None:
        self._context = context

    @abstractmethod
    def run(self) -> None:
        pass

class start(State):
    def run(self):
        self.start_words = ['start', 'go', 'begin']

        if self.context.vosk_msg != None:
            for word in self.context.vosk_msg.split():
                if (word in self.start_words):
                    self.context.task_started = True
        if (self.context.task_started):
            #Don't delete next line
            self.context.transition_to(scanning())
            
class scanning(State):
    def run(self):
        if self.context.apple_point is not None:
            rospy.loginfo("find apple. no scan Pick up")
            self.context.transition_to(picking_up())
        rospy.sleep(1)
        self.context.say("I am looking for the apple")
        rospy.loginfo("move head to left(robot perspective)")
        headPos = JointTrajectory()
        headPos.joint_names = ["head_pan_joint", "head_tilt_joint"]
        point1 = JointTrajectoryPoint()
        point1.positions = [0, -0.6]
        point1.time_from_start = rospy.Duration(secs=1)
        point2 = JointTrajectoryPoint()
        point2.positions = [1.3, -0.6]
        point2.time_from_start = rospy.Duration(secs=8)
        point3 = JointTrajectoryPoint()
        point3.positions = [1.3, -0.0]
        point3.time_from_start = rospy.Duration(secs=10)
        point4 = JointTrajectoryPoint()
        point4.positions = [0.0, -0.0]
        point4.time_from_start = rospy.Duration(secs=12)
        headPos.points = [point1, point2, point3,point4]
        self.context.move_head_pub.publish(headPos)
        rospy.sleep(14)
        rospy.loginfo("move head to right(robot perspective)")
        headPos = JointTrajectory()
        headPos.joint_names = ["head_pan_joint", "head_tilt_joint"]
        point1 = JointTrajectoryPoint()
        point1.positions = [0, -0.6]
        point1.time_from_start = rospy.Duration(secs=1)
        point2 = JointTrajectoryPoint()
        point2.positions = [-1.3, -0.6]
        point2.time_from_start = rospy.Duration(secs=8)
        point3 = JointTrajectoryPoint()
        point3.positions = [-1.3, -0.0]
        point3.time_from_start = rospy.Duration(secs=10)
        point4 = JointTrajectoryPoint()
        point4.positions = [0.0, -0.0]
        point4.time_from_start = rospy.Duration(secs=12)
        headPos.points = [point1, point2, point3,point4]
        self.context.move_head_pub.publish(headPos)
        rospy.sleep(14)
        rospy.loginfo("transiting into pickup")
        self.context.transition_to(picking_up())

class picking_up(State):
    def run(self):
        if self.context.apple_point is None:
            rospy.loginfo("Cannot find apple. Rescaning")
            self.context.transition_to(scanning())
        else:
            rospy.loginfo("pick up now")
            self.apple_stamped = PointStamped()
            self.apple_stamped.point = self.context.apple_point
            self.apple_stamped.header.seq = 0
            self.apple_stamped.header.stamp = rospy.Time.now()
            self.apple_stamped.header.frame_id = 'map'

            self.context.apple_point_pub.publish(self.apple_stamped)

            self.context.transition_to(idle())
class idle(State):
    def run(self):
        rospy.loginfo("in idle")
        pass

# original 
if __name__ == "__main__":
    rospy.init_node('carry_my_luggage_controller')
    carry_my_luggage_controller_node = Context(start())
    while not rospy.is_shutdown():
        carry_my_luggage_controller_node.run()
