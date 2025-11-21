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
dishwasher_pose = Pose()
dishwasher_pose.position.x = -1.805
dishwasher_pose.position.y = -0.239
dishwasher_pose.position.z = 0.000
dishwasher_pose.orientation.x = 0.000
dishwasher_pose.orientation.y = 0.000
dishwasher_pose.orientation.z = -0.268
dishwasher_pose.orientation.w = 0.963

dishwasher_arm_lift_joint = 0.53
dishwasher_arm_flex_joint = -0.03
dishwasher_head_pan_joint = -1.60
dishwasher_head_tilt_joint = 0

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

class MoveToTable(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["move_success", "move_in_progress"])
        self.action_topic = rospy.Publisher(rospy.get_param('/clean_the_table_controller/action_topic'),  UNSWActionMsg, queue_size=1)
        self.action_result_sub= rospy.Subscriber(rospy.get_param('/clean_the_table_controller/action_results_topic'), UNSWActionResult, self.action_results_callback, queue_size=1) 
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
        smach.State.__init__(self, outcomes=['item_detected', 'no_item_detected'], input_keys=['pick_up'], output_keys=['pick_up'])
        # plate doesn't actually exist in coco dataset ... 
        self.priority_queue = ['cup', 'bowl', 'spoon', 'fork', 'knife']

        # VIEW POSE
        self.client = actionlib.SimpleActionClient("manip_view_pose", ManipViewPoseAction)
        if not self.client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Timeout waiting for /manip_view_pose action server")
            assert False, "Timeout waiting for action server"

        # TODO can also change to world model?? - current scene, or all seen objects?
        self.vision_sub = rospy.Subscriber(rospy.get_param("/clean_the_table_controller/vision_topic"), DetectionList, self.vision_callback, queue_size=1)
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

        while result:
            for obj in self.priority_queue:
                for detected_obj in self.detections:
                    if (detected_obj.object_class == obj):
                        # for now remove object from pq, will need to change if we want to clear multiple of the same item.
                        print(f"Detected object: {detected_obj.object_class}")
                        # self.priority_queue.remove(obj)
                        ud.pick_up = detected_obj
                        return 'item_detected'
                
        
    
class AttemptApproach(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['approach_fail', 'approach_success', 'approach_in_progress'], input_keys=['pick_up'], output_keys=['distance'])
        self.approach_pose = {
            'cup': 'front',
            'bowl': 'top',
            'spoon': 'top2',
            'fork': 'top2',
            'knife': 'top2'
        }
        self.action_pub = rospy.Publisher(rospy.get_param('/clean_the_table_controller/action_topic'), UNSWActionMsg, queue_size=1)
        self.action_sub = rospy.Subscriber('/approach_distance', Float64, self.approach_distance_callback, queue_size=1)
        self.distance = None
        self.goal_sent = False
    def approach_distance_callback(self, msg: Float64):
        self.distance = msg.data

    def execute(self, ud):
        print(f"pick_up_object: {ud.pick_up}")
        action_data = {
            'target_point' : {
                'header' : {
                    'frame_id' : 'map'
                },
                'point' : {
                    'x' : ud.pick_up.position.x,
                    'y' : ud.pick_up.position.y,
                    'z' : ud.pick_up.position.z
                }
            },
            'direction' : self.approach_pose[ud.pick_up.object_class]
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
        self.action_pub = rospy.Publisher(rospy.get_param('/clean_the_table_controller/action_topic'), UNSWActionMsg, queue_size=1)
        self.action_sub = rospy.Subscriber(rospy.get_param("/clean_the_table_controller/action_results_topic"), UNSWActionResult, self.grasp_result_callback, queue_size=1)
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
        self.action_pub = rospy.Publisher(rospy.get_param('/clean_the_table_controller/action_topic'), UNSWActionMsg, queue_size=1)
        self.action_sub = rospy.Subscriber(rospy.get_param("/clean_the_table_controller/action_results_topic"), UNSWActionResult, self.retract_result_callback, queue_size=1)
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
    
class MoveToDishwasher(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["move_success", "move_in_progress"])
        self.action_topic = rospy.Publisher(rospy.get_param('/clean_the_table_controller/action_topic'),  UNSWActionMsg, queue_size=1)
        rospy.Subscriber(rospy.get_param('/clean_the_table_controller/action_results_topic'), UNSWActionResult, self.action_results_callback, queue_size=1)
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
                        "x":dishwasher_pose.position.x,
                        "y":dishwasher_pose.position.y,
                        "z":dishwasher_pose.position.z
                    },
                    "orientation": {
                        "x":dishwasher_pose.orientation.x,
                        "y":dishwasher_pose.orientation.y,
                        "z":dishwasher_pose.orientation.z,
                        "w":dishwasher_pose.orientation.w
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
    
class LookAtTray(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['place_position_detected'], input_keys=['place_position'], output_keys=['place_position'])
        # TODO view pose
        self.client = actionlib.SimpleActionClient("manip_view_pose", ManipViewPoseAction)
        if not self.client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Timeout waiting for /manip_view_pose action server")
            assert False, "Timeout waiting for action server"
        # Need to make a region for tray and create a script to find an empty spot on the tray
    
    def execute(self, ud):
        self.client.wait_for_server()
        goal = ManipViewPoseGoal()
        goal.arm_lift_joint = dishwasher_arm_lift_joint
        goal.arm_flex_joint = dishwasher_arm_flex_joint
        goal.head_pan_joint = dishwasher_head_pan_joint
        goal.head_tilt_joint = dishwasher_head_tilt_joint

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
        smach.State.__init__(self, outcomes=['place_success', 'place_in_progress'])
        self.action_pub = rospy.Publisher(rospy.get_param('/clean_the_table_controller/action_topic'), UNSWActionMsg, queue_size=1)
        self.action_sub = rospy.Subscriber(rospy.get_param("/clean_the_table_controller/action_results_topic"), UNSWActionResult, self.place_result_callback, queue_size=1)
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
        action_msg = UNSWActionMsg()
        action_data = {
            'target_point' : {
                'header' : {
                    'frame_id' : 'map'
                },
                'point' : {
                    'x' : -1.74,
                    'y' : -1.08,
                    'z' : 0.8
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
        self.action_pub = rospy.Publisher(rospy.get_param('/clean_the_table_controller/action_topic'), UNSWActionMsg, queue_size=1)
        self.action_sub = rospy.Subscriber(rospy.get_param("/clean_the_table_controller/action_results_topic"), UNSWActionResult, self.retract_result_callback, queue_size=1)
        self.goal_sent = False
        self.retract_status = False
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
            self.retract_status = False
            return  'retract_success'
        else:
            return 'retract_in_progress'

def main():
    rospy.init_node('clean_the_table_controller_smach')
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
                               transitions={'door_open':'MOVE_TO_TABLE', 
                                            'door_closed':'DOOR_CHECK'})
        smach.StateMachine.add('MOVE_TO_TABLE', 
                        MoveToTable(), 
                        transitions={'move_success':'VIEW_TABLE',
                                     'move_in_progress':'MOVE_TO_TABLE' 
                                    })
        smach.StateMachine.add('VIEW_TABLE', 
                        ViewTable(), 
                        transitions={'item_detected':'ATTEMPT_APPROACH', 
                                     'no_item_detected':'MOVE_TO_TABLE'
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
                transitions={'retract_success':'MOVE_TO_DISHWASHER', 
                             'retract_in_progress': 'RETRACT_ARM'
                            })
        smach.StateMachine.add('MOVE_TO_DISHWASHER', 
                MoveToDishwasher(), 
                transitions={'move_success':'LOOK_AT_TRAY', 
                            'move_in_progress': 'MOVE_TO_DISHWASHER'
                            })
        smach.StateMachine.add('LOOK_AT_TRAY', 
                LookAtTray(), 
                transitions={'place_position_detected':'APPROACH_PLACE', 
                            })
        smach.StateMachine.add('APPROACH_PLACE', 
                ApproachPlace(), 
                transitions={'approach_success':'PLACE_OBJECT', 
                            })
        smach.StateMachine.add('PLACE_OBJECT', 
                PlaceObject(), 
                transitions={'place_success':'MOVE_TO_TABLE',
                            'place_in_progress':  'PLACE_OBJECT'
                            })
        smach.StateMachine.add('RETRACT_ARM_PLACE', 
                RetractArmPlace(), 
                transitions={'retract_success':'RETRACT_ARM_PLACE', 
                              'retract_in_progress': 'MOVE_TO_TABLE'
                            })
        

    sis = smach_ros.IntrospectionServer('server_name',sm,'/SM_ROOT')
    sis.start()

    outcome = sm.execute()
    rospy.logwarn(f"State machine finished with outcome: {outcome}")
    rospy.spin()
    sis.stop()
if __name__ == '__main__':
    main()

