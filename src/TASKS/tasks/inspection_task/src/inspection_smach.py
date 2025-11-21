#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import actionlib
import tf.transformations
import sys 
import os

from std_msgs.msg import String, Int8, Bool
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from speech_to_text.msg import speech_recognition

module_path = os.environ.get("UNSW_WS")
sys.path.append(module_path + "/WORLD_MODEL/world_model/world_model_kb/scripts")
from dbase import DBase
from regions_api import RegionsApi


class Talker:
    def __init__(self):
        self.talker = rospy.Publisher("tts/phrase", String, queue_size=10)

    def talk(self, sentence):
        rospy.loginfo(f"saying: {sentence}")
        self.talker.publish(sentence)
            
class Start(smach.State):
    def __init__(self, myTalker):
        smach.State.__init__(self, outcomes=['task_started', 'waiting'])
        self.myTalker = myTalker
        start_topic = rospy.get_param("/inspection_controller/start_topic")
        self.task_started = False
        rospy.loginfo("init wrist sub callback")
        self.wrist_sub = rospy.Subscriber(start_topic, Int8, callback=self.wrist_callback, queue_size=1)
        
    def wrist_callback(self, msg: Int8):
        rospy.logwarn("Task started!")
        self.task_started = True
        
    def execute(self, userdata):
        if self.task_started:
            self.myTalker.talk("Inspection task started. Waiting for the door to be opened.")
            return 'task_started'
        return 'waiting'
    # Create a smach state machine
        

class DoorCheck(smach.State):
    def __init__(self, myTalker):
        smach.State.__init__(self, outcomes=['door_open', 'door_closed'])
        self.myTalker = myTalker
        self.door_open = False
        
        self.door_sub = rospy.Subscriber('/door_open_close', String, callback=self.doors_callback, queue_size=10)
        
    def doors_callback(self, msg: String):
        if msg.data == 'open':
            self.door_open = True

    def execute(self, userdata):
        if self.door_open:
            self.myTalker.talk("I detected the door opening. I'm moving to the inspection point now.")
            return 'door_open'
        if not self.door_open:
            return 'door_closed'

class NavToInspection(smach.State):
    def __init__(self, myTalker):
        smach.State.__init__(self, outcomes=['nav_success', 'nav_fail'])
        self.myTalker = myTalker
        self.cli = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        
        # if nothing heard back from the server in 5 seconds then kill the node
        if not self.cli.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Timeout waiting for /move_base/move action server")
            assert False, "Timeout waiting for action server"
    
        module_path = os.environ.get("UNSW_WS")
        db = DBase(module_path + "/WORLD_MODEL/world_model/world_model_kb/scripts/" + "database.ini")
        #db = DBase("/home/robocupathome/workspace/temp_ws/src/world_model/world_model_kb/scripts/database.ini")
        region_api = RegionsApi()

        self.inspection_point = region_api.get_poi_by_name(db.con_pool, "inspection point")[0]
        # TODO check structure of inspection_point
        
    def execute(self, userdata):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"

        pose.pose.position = Point(self.inspection_point['loc_x'], self.inspection_point['loc_y'], 0)
        quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        pose.pose.orientation = Quaternion(*quat)

        goal = MoveBaseGoal()
        goal.target_pose = pose

        self.cli.send_goal(goal)

        self.cli.wait_for_result()

        navigation_state = self.cli.get_state()
        if navigation_state == GoalStatus.SUCCEEDED:
            self.myTalker.talk("Inspect me now. Press my wrist when finished.")
            rospy.sleep(3)
            return 'nav_success'
        if navigation_state == GoalStatus.ABORTED or navigation_state == GoalStatus.REJECTED:
            return 'nav_fail'

class WristPress(smach.State):
    def __init__(self, myTalker):        
        smach.State.__init__(self, outcomes=['wrist_pressed', 'waiting'])
        self.myTalker = myTalker
        #speech_topic = rospy.get_param("/inspection_controller/speech_topic")
        #self.listen_sub = rospy.Subscriber(speech_topic, speech_recognition, callback=self.listen_callback, queue_size=10)
        wrist_topic = rospy.get_param('/inspection_controller/start_topic')
        self.wrist_pressed = False
        self.in_state = False
        self.wrist_sub = rospy.Subscriber(wrist_topic, Int8, callback=self.wrist_callback, queue_size=1)
        
        
    def wrist_callback(self, msg: Int8):
        rospy.logwarn("Wrist pressed, I'm leaving!")
        if not self.in_state: return
        self.wrist_pressed = True

    #def listen_callback(self, msg:speech_recognition):
    #    for word in msg.final_result.split():
    #        print(f"{word}, {self.finish_words}")
    #        if word in self.finish_words:
    #            print("word heard!")
    #            self.wrist_pressed = True

    def execute(self, userdata):
        self.in_state = True
        if self.wrist_pressed:
            self.myTalker.talk("I'm leaving now.")
            return 'wrist_pressed'
        return 'waiting'
        
class NavToStart(smach.State):
    def __init__(self, myTalker):
        smach.State.__init__(self, outcomes=['nav_success', 'nav_fail'])
        self.myTalker = myTalker
        db = DBase(module_path + "/WORLD_MODEL/world_model/world_model_kb/scripts/" + "database.ini")
        region_api = RegionsApi()
        self.exit_door = region_api.get_poi_by_name(db.con_pool, "door2")[0]

        self.cli = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        # if nothing heard back from the server in 5 seconds then kill the node
        if not self.cli.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Timeout waiting for /move_base/move action server")
            assert False, "Timeout waiting for action server"
    
    def execute(self, userdata):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"

        pose.pose.position = Point(self.exit_door['loc_x'], self.exit_door['loc_y'], 0)
        quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        pose.pose.orientation = Quaternion(*quat)

        goal = MoveBaseGoal()
        goal.target_pose = pose

        self.cli.send_goal(goal)

        self.cli.wait_for_result()

        navigation_state = self.cli.get_state()
        if navigation_state == GoalStatus.SUCCEEDED:
            return 'nav_success'
        if navigation_state == GoalStatus.ABORTED or navigation_state == GoalStatus.REJECTED:
            return 'nav_fail'

def main():
    rospy.init_node('inspection_smach')
    myTalker = Talker()
    # Create a smach state machine
    sm = smach.StateMachine(outcomes=['inspection_success'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('START', Start(myTalker), transitions={'task_started':'DOOR_CHECK', 'waiting':'START'})
        smach.StateMachine.add('DOOR_CHECK', DoorCheck(myTalker), transitions={'door_open':'NAV_TO_INSPECTION', 'door_closed':'DOOR_CHECK'})
        smach.StateMachine.add('NAV_TO_INSPECTION', NavToInspection(myTalker), transitions={'nav_success': 'WRIST_PRESS', 'nav_fail':'NAV_TO_INSPECTION'})
        smach.StateMachine.add('WRIST_PRESS', WristPress(myTalker), transitions={'wrist_pressed':'NAV_TO_START','waiting':'WRIST_PRESS'})
        smach.StateMachine.add('NAV_TO_START', NavToStart(myTalker), transitions={'nav_success':'inspection_success', 'nav_fail':'NAV_TO_START'})
    
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()
    rospy.logwarn(f"State machine finished with outcome: {outcome}")

    rospy.spin()
    sis.stop()
if __name__ == '__main__':
    main()

