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
from dbase import DBase

from robot_state_api import RobotStateApi
from regions_api import RegionsApi

robotState = RobotStateApi()

# TODO: Change to use world model instead of subscribing to topics as much as possible. 
# TODO: Change to make use of action clients
# TODO: add regions for table pickup points to wm

# TODO: change to directly use action server instead of publishing to topic


# TODO: Test table view pose for store groceries
# Table for pick up 
# Frame:map, Position(0.990, -0.186, 0.000), Orientation(0.000, 0.000, 0.496, 0.868) = Angle: 1.039
'''
arm_flex_joint  OK      -0.48   
arm_lift_joint  OK      0.44    
arm_roll_joint  OK  -0.08

head_pan_joint -1.60
head_tilt_joint -0.56   
'''
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

# Table for placing
# Setting goal: Frame:map, Position(-1.805, -0.239, 0.000), Orientation(0.000, 0.000, -0.268, 0.963) = Angle: -0.544
'''
arm_flex_joint  OK      -0.03   
arm_lift_joint  OK      0.53    
arm_roll_joint  OK  -0.00

head_pan_joint 0
head_tilt_joint 0
'''
cabinet_pose = Pose()
cabinet_pose.position.x = -1.805
cabinet_pose.position.y = -0.239
cabinet_pose.position.z = 0.000
cabinet_pose.orientation.x = 0.000
cabinet_pose.orientation.y = 0.000
cabinet_pose.orientation.z = -0.268
cabinet_pose.orientation.w = 0.963

cabinet_arm_lift_joint = 0.53
cabinet_arm_flex_joint = -0.03
cabinet_head_pan_joint = -1.60
cabinet_head_tilt_joint = 0

cabinet_pose1 = cabinet_pose
cabinet_pose2 = cabinet_pose
cabinet_pose3 = cabinet_pose


object_category_dict = {
    "banana": "fruit",
    "apple": "fruit",
    "cornflakes": "food",
    "mayonaise": "food"
}

class Talker:
    def __init__(self):
        tts_topic = rospy.get_param('/store_groceries_controller/tts_topic')
        self.talker = rospy.Publisher(tts_topic, String, queue_size=1)
    
    def talk(self, sentence):
        rospy.loginfo(f"saying: {sentence}")
        self.talker.publish(sentence)

class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['task_started'], input_keys=['action_sub', 'start_pose']
                             , output_keys=['start_pose'])
        module_path = os.environ.get("UNSW_WS")
        self.db = DBase(module_path + "/WORLD_MODEL/world_model/world_model_kb/scripts/" + "database.ini")
        self.start_task_pub = rospy.Publisher('/start_task', Int8, queue_size=1)

    def execute(self, userdata):
        wrist_pressed = False
        while not wrist_pressed:
            robot_state = robotState.get_robot_state_current(self.db.con_pool)[0][0][0]
            wrist_pressed = robot_state['is_wrist_pressed']
        
        self.start_task_pub.publish(1)
        return 'task_started'

class DoorCheck(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['door_open', 'door_closed'])
        self.door_open = False
        self.door_sub = rospy.Subscriber('/door_open_close', String, callback=self.doors_callback, queue_size=10)
        
    def doors_callback(self, msg: String):
        if msg.data == 'open':
            self.door_open = True

    def execute(self, userdata):
        while not self.door_open:
            pass
        return 'door_open'

class MoveToCabinet(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["move_success", "move_in_progress"])
        self.action_topic = rospy.Publisher(rospy.get_param('/store_groceries_controller/action_topic'),  UNSWActionMsg, queue_size=1)
        rospy.Subscriber(rospy.get_param('/store_groceries_controller/action_results_topic'), UNSWActionResult, self.action_results_callback, queue_size=1)
        self.sent_goal = False
        self.move_success = False
    def action_results_callback(self, msg: UNSWActionResult):
        if msg.action_name == "goto_goal" and msg.action_result:
            self.move_success = True
        else:
            self.move_success = False
    
    def execute(self, ud):
        action_msg = UNSWActionMsg()
        action_msg.action_name = 'goto_goal'

        goal_data_dict= {"GoalPose":
            {
                "header":{"frame_id":"map"},
                "pose":{
                    "position": {
                        "x":cabinet_pose.position.x,
                        "y":cabinet_pose.position.y,
                        "z":cabinet_pose.position.z
                    },
                    "orientation": {
                        "x":cabinet_pose.orientation.x,
                        "y":cabinet_pose.orientation.y,
                        "z":cabinet_pose.orientation.z,
                        "w":cabinet_pose.orientation.w
                    }
                }
            }
        }
        action_msg.data=json.dumps(goal_data_dict)

        if not self.sent_goal:
            # set status success to false
            self.move_success = False
            self.sent_goal = True
            self.action_topic.publish(action_msg)

        while not self.move_success:
            pass

        if self.move_success:
            # reset the status so next time entering the state it works
            self.sent_goal = False
            self.move_success = False
            return "move_success"
        
class OpenCabinetDoorA(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['door_a_opened'])
        self.talker = Talker()
        self.wait = 5.0
        
    def execute(self, ud):
        # TODO: Add door opening behaviour

        self.talker.talk('I cannot open the first cabinet door, please open it for me.')
        rospy.sleep(self.wait)
        return  'door_a_opened'

class OpenCabinetDoorB(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['door_b_opened'])
        self.talker = Talker()
        self.wait = 5.0
        
    def execute(self, ud):
        # TODO: Add door opening behaviour

        self.talker.talk('I cannot open the second cabinet door, please open it for me.')
        rospy.sleep(self.wait)
        return  'door_b_opened'

# TODO: Move to two different poses
# TODO: For each pose use the plane detection to find the shelves, then define the categories with the DetectionList
class DefineShelves(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['shelves_defined'], input_keys=['place_position'], output_keys=['place_position', 'current_cabinet', 'categories_set'])
        self.plane_marker_dict = {}
        rospy.Subscriber(rospy.get_param('/store_groceries_controller/plane_detection_topic'), Marker, self.plane_detection_callback, queue_size=1)
        self.wait = 0.0
        self.client = actionlib.SimpleActionClient("manip_view_pose", ManipViewPoseAction)

    def plane_detection_callback(self, msg: Marker):
        self.plane_marker_dict[msg.id] = {"marker": msg, "categories": []}
    
    # TODO: Go through the three different poses, find all planes + detection list for each, define shelves using these two
    def execute(self, ud):
        # TODO: Define first cabinet viewing pose
        self.goal = ManipViewPoseGoal()
        self.goal.arm_lift_joint = table_arm_lift_joint
        self.goal.arm_flex_joint = table_arm_flex_joint
        self.goal.head_pan_joint = table_head_pan_joint
        self.goal.head_tilt_joint = table_head_tilt_joint

        self.client.send_goal(self.goal)
        self.client.wait_for_result()

        rospy.sleep(self.wait)
        detection_list_init = rospy.wait_for_message('/unsw_vision/detections/objects', DetectionList)
        plane_dict_init = self.plane_marker_dict.copy()


        # TODO: Define second cabinet viewing pose
        self.goal = ManipViewPoseGoal()
        self.goal.arm_lift_joint = table_arm_lift_joint
        self.goal.arm_flex_joint = table_arm_flex_joint
        self.goal.head_pan_joint = table_head_pan_joint
        self.goal.head_tilt_joint = table_head_tilt_joint

        self.client.send_goal(self.goal)
        self.client.wait_for_result()

        rospy.sleep(self.wait)
        detection_list = rospy.wait_for_message('/unsw_vision/detections/objects', DetectionList)
        plane_dict = self.plane_marker_dict.copy()

        plane_list = sorted(
            list(plane_dict_init.values()) + list(plane_dict.values()),
            key=lambda m: m.marker.pose.position.z
        )
        
        shelf_dict = {index: value for index, value in enumerate(plane_list)}

        sorted_markers = sorted(shelf_dict.items(), key=lambda x: x[1].marker.pose.position.z)

        detections = list(detection_list_init.detections) + list(detection_list.detections)

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

        ud.current_cabinet = shelf_dict
        # create set of all the categories currently on shelves
        shelf_category_list = []
        for shelf in ud.current_cabinet.values():
            shelf_category_list.extend(shelf.categories)
        ud.categories_set = set(shelf_category_list)

        return  'shelves_defined'

class MoveToTable(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["move_success", "move_in_progress"])
        self.action_topic = rospy.Publisher(rospy.get_param('/store_groceries_controller/action_topic'),  UNSWActionMsg, queue_size=1)
        self.action_result_sub= rospy.Subscriber(rospy.get_param('/store_groceries_controller/action_results_topic'), UNSWActionResult, self.action_results_callback, queue_size=1) 
        self.sent_goal = False
        self.move_success = False

    def action_results_callback(self, msg: UNSWActionResult):
        if msg.action_name == "goto_goal" and msg.action_result:
            self.move_success = True
        else:
            self.move_success = False

    def execute(self, ud):
        action_msg = UNSWActionMsg()
        action_msg.action_name = 'goto_goal'

        # TODO: get point for table pickup
        # add orientation to poi in db
        # table_pickup_pose = json.loads(regions.get_poi_by_name(db.con_pool, 'table_pickup'))
        # pose = table_pickup_pose['pose']
        # goal_data_dict= {"GoalPose":
        #     {
        #         "header":{"frame_id":"map"},
        #         "pose":pose
        #     }
        # }
        goal_data_dict= {"GoalPose":
            {
                "header":{"frame_id":"map"},
                "pose":{
                    "position": {
                        "x":table_pose.position.x,
                        "y":table_pose.position.y,
                        "z":table_pose.position.z
                    },
                    "orientation": {
                        "x":table_pose.orientation.x,
                        "y":table_pose.orientation.y,
                        "z":table_pose.orientation.z,
                        "w":table_pose.orientation.w
                    }
                }
            }
        # feedback from nav client??
        }
        action_msg.data=json.dumps(goal_data_dict)

        if not self.sent_goal:
            # set status success to false
            self.move_success = False
            self.sent_goal = True
            self.action_topic.publish(action_msg)
    
        while not self.move_success:
            pass
        # reset the status so next time entering the state it works
        self.sent_goal = False    
        return "move_success"
    
class ViewTable(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['item_detected', 'no_item_detected'], input_keys=['pickup'], output_keys=['pickup'])
        # VIEW POSE
        self.client = actionlib.SimpleActionClient("manip_view_pose", ManipViewPoseAction)
        if not self.client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Timeout waiting for /manip_view_pose action server")
            assert False, "Timeout waiting for action server"

        # TODO can also change to world model?? - current scene, or all seen objects?
        self.vision_sub = rospy.Subscriber(rospy.get_param("/store_groceries_controller/vision_topic"), DetectionList, self.vision_callback, queue_size=1)
        self.talker = Talker()

        rospy.wait_for_service(rospy.get_param('/store_groceries_controller/object_frames_service_topic'))
        self.findObjectFrame = rospy.ServiceProxy(rospy.get_param('/store_groceries_controller/object_frames_service_topic'), FindObjectFrame)

        self.detections = None

    def vision_callback(self, msg: DetectionList):
        self.detections = msg.objects
    
    def execute(self, ud):
        self.goal = ManipViewPoseGoal()
        self.goal.arm_lift_joint = table_arm_lift_joint
        self.goal.arm_flex_joint = table_arm_flex_joint
        self.goal.head_pan_joint = table_head_pan_joint
        self.goal.head_tilt_joint = table_head_tilt_joint

        self.client.send_goal(self.goal)
        result = self.client.wait_for_result()

        object_chosen = False
        while result:
            for detected_obj in self.detections:
                obj_class = detected_obj.object_class
                if detected_obj.object_class in object_category_dict and object_category_dict[obj_class] in ud.categories_set:
                    print(f"Detected object: {detected_obj.object_class}")
                    self.talker.talk(f'{detected_obj.object_class} found, {object_category_dict[obj_class]} category')
                    self.findObjectFrame(detected_obj.object_class, detected_obj.tracking_id)
                    ud.pickup = detected_obj
                    object_chosen = True
                    break
            if not object_chosen:
                for detected_obj in self.detections:
                    obj_class = detected_obj.object_class
                    if detected_obj.object_class in object_category_dict:
                        print(f"Detected object: {detected_obj.object_class}")
                        self.talker.talk(f'{detected_obj.object_class} found, {object_category_dict[obj_class]} category')
                        self.findObjectFrame(detected_obj.object_class, detected_obj.tracking_id)
                        ud.pickup = detected_obj
                        break
        if object_chosen:
            return 'item_detected'
        else:
            return 'no_item_detected'
        
                
        
# TODO: define approach poses for each of the new objects
class AttemptApproach(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['approach_fail', 'approach_success', 'approach_in_progress'], input_keys=['pickup'], output_keys=['distance'])
        # define approaches for each of the object categories
        self.approach_pose = {
            'banana': 'top',
            'apple': 'top',
            'cornflakes': 'front',
            'mayonaise': 'front'
        }
        self.action_pub = rospy.Publisher(rospy.get_param('/store_groceries_controller/action_topic'), UNSWActionMsg, queue_size=1)
        self.action_sub = rospy.Subscriber('/approach_distance', Float64, self.approach_distance_callback, queue_size=1)
        self.distance = None
        self.goal_sent = False
    def approach_distance_callback(self, msg: Float64):
        self.distance = msg.data

    def execute(self, ud):
        print(f"pick_up_object: {ud.pickup}")
        action_data = {
            'target_point' : {
                'header' : {
                    'frame_id' : 'map'
                },
                'point' : {
                    'x' : ud.pickup.position.x,
                    'y' : ud.pickup.position.y,
                    'z' : ud.pickup.position.z
                }
            },
            'direction' : self.approach_pose[ud.pickup.object_class]
        }

        action_msg = UNSWActionMsg()
        action_msg.action_name = 'manip_approach'
        action_msg.data = json.dumps(action_data)
        
        # TODO: revise this later. This is used to stop sending the same goal
        if not self.goal_sent:
            self.distance = None
            self.action_pub.publish(action_msg)
            self.goal_sent = True
        
        while self.distance is None:
            pass

        ud.distance = self.distance
        self.goal_sent = False
        return 'approach_success'
    
class HandCentring(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['centred'])
    
    def execute(self, ud):
        return  'centred'

class AttemptGrasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['grasp_success', 'grasp_fail','grasp_in_progress'], input_keys=['distance'])
        self.action_pub = rospy.Publisher(rospy.get_param('/store_groceries_controller/action_topic'), UNSWActionMsg, queue_size=1)
        self.action_sub = rospy.Subscriber(rospy.get_param("/store_groceries_controller/action_results_topic"), UNSWActionResult, self.grasp_result_callback, queue_size=1)
        self.grasp_result = False
        self.goal_sent = False
    def grasp_result_callback(self, msg: UNSWActionResult):
        if msg.action_name == "manip_grasp" and msg.action_result:
            self.grasp_result = True
        else: self.grasp_result = False

    def execute(self, ud):

        action_msg = UNSWActionMsg()
        action_msg.action_name = 'manip_grasp'
        action_msg.data = json.dumps({'forward_distance': ud.distance})
        print(action_msg.data)
        if not self.goal_sent:
            self.goal_sent = True
            self.grasp_result = False
            self.action_pub.publish(action_msg)
        
        # TODO: WAIT until feedback from action server
        while not self.grasp_result:
            pass

        if self.grasp_result:
            # reset the status so next time entering the state it works
            self.goal_sent = False
            return  'grasp_success'
        
class RetractArm(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['retract_success','retract_in_progress'])
        self.action_pub = rospy.Publisher(rospy.get_param('/store_groceries_controller/action_topic'), UNSWActionMsg, queue_size=1)
        self.action_sub = rospy.Subscriber(rospy.get_param("/store_groceries_controller/action_results_topic"), UNSWActionResult, self.retract_result_callback, queue_size=1)
        self.goal_sent = False
        self.retract_result = False
    def retract_result_callback(self, msg: UNSWActionResult):
        if msg.action_name == "manip_retract" and msg.action_result:
            self.retract_result = msg
        else:
            self.retract_result = False
    def execute(self, ud):
        action_msg = UNSWActionMsg()
        action_msg.action_name = 'manip_retract'
        action_msg.data = json.dumps({})
        if not self.goal_sent:
            rospy.sleep(2)
            self.retract_result = False
            self.action_pub.publish(action_msg)
            self.goal_sent = True

        while not self.retract_result:
            pass

        if self.retract_result:
            # reset the status so next time entering the state it works
            self.goal_sent = False
            self.retract_result = False
            return  'retract_success'
    

class LookAtCabinet(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['place_position_detected'], input_keys=['place_position', 'categories_set', 'pickup'], output_keys=['place_position', 'categories_set', 'pickup'])
        self.client = actionlib.SimpleActionClient("manip_view_pose", ManipViewPoseAction)
        if not self.client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Timeout waiting for /manip_view_pose action server")
            assert False, "Timeout waiting for action server"
    
    # TODO: Go through the three different poses, find all planes + detection list for each, define shelves using these two
    def execute(self, ud):
        if ud.pickup.object_class not in object_category_dict or object_category_dict[ud.pickup.object_class] not in ud.categories_set:
            return 'place_fail'

        self.client.wait_for_server()
        goal = ManipViewPoseGoal()
        goal.arm_lift_joint = cabinet_arm_lift_joint
        goal.arm_flex_joint = cabinet_arm_flex_joint
        goal.head_pan_joint = cabinet_head_pan_joint
        goal.head_tilt_joint = cabinet_head_tilt_joint

        self.client.send_goal(goal)
        self.client.wait_for_result()
        if self.client.get_result():
            return  'place_position_detected'
        
class ApproachPlace(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['approach_success'])
        
    def execute(self, ud):
        return  'approach_success'
    
class PlaceObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['place_success', 'place_in_progress', 'place_fail'],  input_keys=['current_cabinet', 'pickup'], output_keys=['current_cabinet', 'pickup'])
        self.action_pub = rospy.Publisher(rospy.get_param('/store_groceries_controller/action_topic'), UNSWActionMsg, queue_size=1)
        self.action_sub = rospy.Subscriber(rospy.get_param("/store_groceries_controller/action_results_topic"), UNSWActionResult, self.place_result_callback, queue_size=1)
        self.talker = Talker()
        self.goal_sent = False
        self.place_result = False
    def place_result_callback(self, msg: UNSWActionResult):
        #TODO: actual check for grasp success
        if msg.action_name == "manip_placing" and msg.action_result:
            self.place_result = True
        else:
            self.place_result = False
        '''
        x: -1.851973056793213
        y: -1.1357269287109375

        '''

    def execute(self, ud):
        # Find object category [based on view table, object must be in object cat dict]
        object_category = object_category_dict[ud.pickup.object_class]
        final_shelf_marker = None
        for shelf in ud.current_cabinet.values():
            if object_category in shelf.categories:
                final_shelf_marker = shelf.marker

        if final_shelf_marker is None:
            # TODO: make hand open
            self.talker.talk(f'Cannot find correct shelf for {ud.pickup.object_class} in {object_category} category. Please place it for me.')
        # ud.pickup = object
        action_msg = UNSWActionMsg()
        action_data = {
            'target_point' : {
                'header' : {
                    'frame_id' : 'map'
                },
                'point' : {
                    'x' : final_shelf_marker.pose.position.x,
                    'y' : final_shelf_marker.pose.position.y,
                    'z' : final_shelf_marker.pose.position.z + 0.05
                }
            },
            'direction' : 'table'
        }
        action_msg.action_name = 'manip_placing'
        action_msg.data = json.dumps(action_data)
        if not self.goal_sent:
            self.place_result = False
            self.action_pub.publish(action_msg)
            self.goal_sent = True

        while not self.place_result:
            pass

        if self.place_result:
            # reset the status so next time entering the state it works
            self.goal_sent=False
            self.place_result=False
            return  'place_success'
    
class RetractArmPlace(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['retract_success','retract_in_progress'])
        self.action_pub = rospy.Publisher(rospy.get_param('/store_groceries_controller/action_topic'), UNSWActionMsg, queue_size=1)
        self.action_sub = rospy.Subscriber(rospy.get_param("/store_groceries_controller/action_results_topic"), UNSWActionResult, self.retract_result_callback, queue_size=1)
        self.goal_sent = False
        self.retract_result = False
    def retract_result_callback(self, msg: UNSWActionResult):
        if msg.action_name == 'manip_retract' and msg.action_result:
            self.retract_result = True
        else:
            self.retract_result = False
    def execute(self, ud):
        action_msg = UNSWActionMsg()
        action_msg.action_name = 'manip_retract'
        action_msg.data = json.dumps({})
        if not self.goal_sent:
            self.retract_result = False
            self.goal_sent = True
            self.action_pub.publish(action_msg)

        if self.retract_result:
            # reset the status so next time entering the state it works
            self.goal_sent = False
            self.retract_result = False
            return  'retract_success'
        else:
            return 'retract_in_progress'

class PlaceFail(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['fail_complete'])
        self.talker = Talker()
        self.wait = 10.0
    
    # TODO: Go through the three different poses, find all planes + detection list for each, define shelves using these two
    def execute(self, ud):
        self.talker.talk(f'No shelf found for {object_category_dict[ud.pickup.object_class]} category. Please place for me')
        rospy.sleep(self.wait)
        # Open hand
        return 'fail_complete'

def main():
    rospy.init_node('store_groceries_controller_smach')
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    sm = smach.StateMachine(outcomes = ["task_success"])

    with sm:
        smach.StateMachine.add('START', 
                               Start(), 
                               transitions={'task_started':'DOOR_CHECK',
                                            })
        smach.StateMachine.add('DOOR_CHECK', 
                               DoorCheck(), 
                               transitions={'door_open':'MOVE_TO_CABINET_A', 
                                            'door_closed':'DOOR_CHECK'})
        smach.StateMachine.add('MOVE_TO_CABINET_A', 
                MoveToCabinet(), 
                transitions={'move_success':'DEFINE_SHELVES', 
                            'move_in_progress': 'MOVE_TO_CABINET_A'
                            })
        # smach.StateMachine.add('MOVE_TO_CABINET_A', 
        #         MoveToCabinet(), 
        #         transitions={'move_success':'OPEN_CABINET_DOOR', 
        #                     'move_in_progress': 'MOVE_TO_CABINET_A'
        #                     })
        # smach.StateMachine.add('OPEN_CABINET_DOOR_A',
        #         OpenCabinetDoorA(), 
        #         transitions={'door_a_opened':'OPEN_CABINET_DOOR_B'
        #                     })
        # smach.StateMachine.add('OPEN_CABINET_DOOR_B',
        #         OpenCabinetDoorB(), 
        #         transitions={'door_b_opened':'DEFINE_SHELVES'
        #                     })
        smach.StateMachine.add('DEFINE_SHELVES', 
                DefineShelves(), 
                transitions={'shelves_defined':'MOVE_TO_TABLE'
                            })
        smach.StateMachine.add('MOVE_TO_TABLE', 
                        MoveToTable(), 
                        transitions={'move_success':'VIEW_TABLE',
                                     'move_in_progress':'MOVE_TO_TABLE' 
                                    })
        smach.StateMachine.add('VIEW_TABLE', 
                        ViewTable(), 
                        transitions={'item_detected':'ATTEMPT_APPROACH', 
                                     'no_item_detected':'VIEW_TABLE'
                                    })
        smach.StateMachine.add('ATTEMPT_APPROACH', 
                AttemptApproach(), 
                transitions={'approach_fail':'ATTEMPT_GRASP', 
                            'approach_success': 'ATTEMPT_GRASP',
                            'approach_in_progress': 'ATTEMPT_APPROACH'})
        smach.StateMachine.add('ATTEMPT_GRASP', 
                AttemptGrasp(), 
                transitions={'grasp_fail':'MOVE_TO_TABLE', 
                            'grasp_success': 'RETRACT_ARM',
                            'grasp_in_progress': 'ATTEMPT_GRASP'})
        smach.StateMachine.add('RETRACT_ARM', 
                RetractArm(), 
                transitions={'retract_success':'MOVE_TO_CABINET_B', 
                             'retract_in_progress': 'RETRACT_ARM'
                            })
        smach.StateMachine.add('MOVE_TO_CABINET_B', 
                MoveToCabinet(), 
                transitions={'move_success':'LOOK_AT_CABINET', 
                            'move_in_progress': 'MOVE_TO_CABINET_B'
                            })
        smach.StateMachine.add('LOOK_AT_CABINET', 
                LookAtCabinet(), 
                transitions={'place_position_detected':'APPROACH_PLACE',
                             'place_fail':'PLACE_FAIL'
                            })
        smach.StateMachine.add('APPROACH_PLACE', 
                ApproachPlace(), 
                transitions={'approach_success':'PLACE_OBJECT', 
                            })
        smach.StateMachine.add('PLACE_OBJECT', 
                PlaceObject(), 
                transitions={'place_success':'RETRACT_ARM_PLACE',
                            'place_in_progress':  'PLACE_OBJECT',
                            'place_fail': 'PLACE_FAIL'
                            })
        smach.StateMachine.add('RETRACT_ARM_PLACE', 
                RetractArmPlace(), 
                transitions={'retract_success':'MOVE_TO_TABLE', 
                              'retract_in_progress': 'RETRACT_ARM_PLACE'
                            })
        smach.StateMachine.add('PLACE_FAIL',
                PlaceFail(),
                transitions={'fail_complete': 'MOVE_TO_TABLE'
                            })
        

    sis = smach_ros.IntrospectionServer('server_name',sm,'/SM_ROOT')
    sis.start()

    outcome = sm.execute()
    rospy.logwarn(f"State machine finished with outcome: {outcome}")
    rospy.spin()
    sis.stop()
if __name__ == '__main__':
    main()