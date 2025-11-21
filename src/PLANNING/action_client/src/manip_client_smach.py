#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import json
import actionlib

from std_msgs.msg import String, Float64, Bool, Int32
from unsw_action_msg.msg import UNSWActionMsg, UNSWActionResult
from action_server.msg import ManipApproachAction, ManipApproachGoal, ManipApproachResult, ManipRetractAction, ManipRetractGoal, ManipRetractResult, ManipGraspAction, ManipGraspGoal, ManipGraspResult, ManipPlacingAction, ManipPlacingGoal, ManipPlacingResult, ManipGazeAtAction, ManipGazeAtGoal

class WaitForAction(smach.State):
    def __init__(self) :
        smach.State.__init__(self, outcomes=['approach_goal_action', 'retract_action', 'grasp_goal_action', 'place_goal_action', 'pickup_goal_action', 'gaze_action', 'waiting'], input_keys = ['data'], output_keys = ['data'])
        # , 'pickup_goal_action', 'open_door_action', 'open_cupboard_action', 'open_dishwasher_action', 'pull_rack_action', 'close_dishwasher_action', 'pour_liquid_action', 'pour_solid_action',
        action_subcriber = rospy.Subscriber("/actions", UNSWActionMsg , self.action_cb, queue_size=1)
        self.action = None     
        self.data = {}   

    def action_cb(self, msg: UNSWActionMsg):
        self.data = json.loads(msg.data)
        if msg.action_name == "manip_approach":
            self.action = "approach_goal_action"
        elif msg.action_name == "manip_retract" :
            self.action = "retract_action"
        elif msg.action_name == "manip_grasp" :
            self.action = "grasp_goal_action"
        elif msg.action_name == "manip_pickup":
            self.action = "pickup_goal_action"
        elif msg.action_name == "manip_placing":
            self.action = "place_goal_action"
        elif msg.action_name == "manip_gaze":
            self.action = "gaze_action"
        # elif msg.data == "open_door":
        #     self.action = "open_door_action"
        # elif msg.data == "open_cupboard":
        #     self.action = "open_cupboard_action"
        # elif msg.data == "open_dishwasher":
        #     self.action = "open_dishwasher_action"
        # elif msg.data == "pull_rack":
        #     self.action = "pull_rack_action"
        # elif msg.data == "close_dishwasher":
        #     self.action = "close_dishwasher_action"
        # elif msg.data == "pour_liquid":
        #     self.action = "pour_liquid_action"
        # elif msg.data == "pour_solid":
        #     self.action = "pour_solid_action"

    def execute(self, userdata):
        userdata.data = self.data
        if self.action is not None:
            action_to_return = self.action
            self.action = None  # Reset the action to prevent re-triggering
            return action_to_return
        
        return 'waiting'
'''
    x: -0.1697993278503418
    y: -0.9925937652587891
    z: 0.0019373893737792969
'''

# '{ "direction": "front","target_point": {"header": {"frame_id": "map"},"point": {"x":-0.1697993278503418,"y":-0.9925937652587891,"z": 0.5}}}'
# '{ "direction": "front","target_point": {"header": {"frame_id": "base_link"},"point": {"x":0.8,"y":0,"z": 0.5}}}'
class ApproachGoalAction(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'], input_keys = ['data'])
        # from control.msgs.msg import PickupGoalAction
        self.distance_pub = rospy.Publisher("/approach_distance", Float64 , queue_size=1)
        self.client = actionlib.SimpleActionClient("manip_approach", ManipApproachAction)
        if not self.client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Timeout waiting for /approach_goal action server")
            assert False, "Timeout waiting for action server"


    def execute(self, userdata):
        rospy.loginfo('Executing Approach Goal Action State')
        self.client.wait_for_server()

        print(userdata.data)
        goal = ManipApproachGoal()
        goal.direction = userdata.data['direction']

        goal.target_point.header.frame_id = userdata.data['target_point']['header']['frame_id']
        goal.target_point.point.x = userdata.data['target_point']['point']['x']     
        goal.target_point.point.y = userdata.data['target_point']['point']['y']
        goal.target_point.point.z = userdata.data['target_point']['point']['z']  
        goal.object_id = userdata.data['object_id']
        goal.class_name = userdata.data['class_name']

        print(goal)
        self.client.send_goal_and_wait(goal)

        result = ManipApproachResult()
        self.client.wait_for_result()
        result = self.client.get_result()
        
        print("Action Outcome : {}".format(result))
        
        if (result) : 
            # set input_keys = ['foward_distance'] TODO: update world model
            self.distance_pub.publish(result.forward_distance)
            return 'success'
        else :
            return 'failure'
        

class ApproachGoalSuccess(smach.State):
    def __init__(self):        
        smach.State.__init__(self, outcomes=['action_complete'])
        self.result_pub = rospy.Publisher("/actions/result", UNSWActionResult, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo('Pickup Action Suceeded')
        result = UNSWActionResult()
        result.action_name = "manip_approach"
        result.action_result = True
        self.result_pub.publish(result)
        return 'action_complete'

class ApproachGoalFailure(smach.State):
    def __init__(self):        
        smach.State.__init__(self, outcomes=['action_failed'])
        self.result_pub = rospy.Publisher("/actions/result", UNSWActionResult, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo('Approaching action has failed')
        result = UNSWActionResult()
        result.action_name = "manip_approach"
        result.action_result = False
        self.result_pub.publish(result)
        return 'action_failed'

class RetractAction(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'], input_keys = ['data'])
        # from control.msgs.msg import PickupGoalAction
        self.client = actionlib.SimpleActionClient("manip_retract", ManipRetractAction)
        if not self.client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Timeout waiting for manip_retract/goal action server")
            assert False, "Timeout waiting for action server"


    def execute(self, userdata):
        rospy.loginfo('Executing Retract Action State')
        self.client.wait_for_server()

        goal = ManipRetractGoal()
        
        print(goal)
        self.client.send_goal_and_wait(goal)

        self.client.wait_for_result()
        result = self.client.get_result()
        print("Action Outcome : {}".format(result))
        
        if (result.action_success) :        
            return 'success'
        else :
            return 'failure'
        

class RetractSuccess(smach.State):
    def __init__(self):        
        smach.State.__init__(self, outcomes=['action_complete'])
        self.result_pub = rospy.Publisher("/actions/result", UNSWActionResult, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo('Retracting Action Suceeded')
        result = UNSWActionResult()
        result.action_name = "manip_retract"
        result.action_result = True
        self.result_pub.publish(result)
        return 'action_complete'

class RetractFailure(smach.State):
    def __init__(self):        
        smach.State.__init__(self, outcomes=['action_failed'])
        self.result_pub = rospy.Publisher("/actions/result", UNSWActionResult, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo('Retracting action has failed')
        result = UNSWActionResult()
        result.action_name = "manip_retract"
        result.action_result = False
        self.result_pub.publish(result)
        return 'action_failed'

class GraspGoalAction(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'], input_keys=['data'])
        # from control.msgs.msg import GraspGoalAction
        self.client = actionlib.SimpleActionClient("manip_grasp", ManipGraspAction)
        if not self.client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Timeout waiting for manip_grasp/goal action server")
            assert False, "Timeout waiting for action server"

    def execute(self, userdata):
        rospy.loginfo('Executing Pickup Goal Action State')
        self.client.wait_for_server()

        goal = ManipGraspGoal()
        goal.forward_distance = userdata.data['forward_distance']         

        print(goal)
        self.client.send_goal_and_wait(goal)

        self.client.wait_for_result()
        result = self.client.get_result()
        print("Action Outcome : {}".format(result))
        
        if (result.action_success) :        
            return 'success'
        else :
            return 'failure'

class GraspGoalSuccess(smach.State):
    def __init__(self):        
        smach.State.__init__(self, outcomes=['action_complete'])
        self.result_pub = rospy.Publisher("/actions/result", UNSWActionResult, queue_size=1)

    def execute(self, userdata):
        result = UNSWActionResult()
        result.action_name = "manip_grasp"
        result.action_result = True
        self.result_pub.publish(result)
        return 'action_complete'

class GraspGoalFailure(smach.State):
    def __init__(self):        
        smach.State.__init__(self, outcomes=['action_failed'])
        self.result_pub = rospy.Publisher("/actions/result", UNSWActionResult, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo('Grasp action has failed')
        result = UNSWActionResult()
        result.action_name = "manip_grasp"
        result.action_result = False
        self.result_pub.publish(result)
        return 'action_failed'

class PickupGoalAction(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'], input_keys=['data'])
        
        # Create action clients
        self.approach_client = actionlib.SimpleActionClient("manip_approach", ManipApproachAction)
        self.grasp_client = actionlib.SimpleActionClient("manip_grasp", ManipGraspAction)
        self.retract_client = actionlib.SimpleActionClient("manip_retract", ManipRetractAction)
        
        if not self.approach_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Timeout waiting for manip_approach action server")
            assert False, "Timeout waiting for manip_approach action server"
            
        if not self.grasp_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Timeout waiting for manip_grasp action server")
            assert False, "Timeout waiting for manip_grasp action server"

        if not self.grasp_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Timeout waiting for manip_retract action server")
            assert False, "Timeout waiting for manip_retract action server"

        self.holding_pub = rospy.Publisher("/holding_object_id", Int32, queue_size=1, latch=True)

    def execute(self, userdata):
        rospy.loginfo('Executing Pickup Goal Action State (Approach + Grasp)')
        
        # Extract data from userdata
        class_name = userdata.data.get('class_name', '')
        object_id = userdata.data.get('object_id', 0)
        direction = userdata.data.get('direction', 'front')  # Default to 'top' grasp
        
        rospy.loginfo(f"Picking up {class_name} with ID {object_id} from direction {direction}")
        
        # ===== STEP 1: APPROACH =====
        approach_goal = ManipApproachGoal()
        approach_goal.class_name = class_name
        approach_goal.object_id = object_id
        approach_goal.direction = direction
        
        # Target point can be empty as it's not used in the current implementation
        approach_goal.target_point.header.frame_id = "base_footprint"
        approach_goal.target_point.point.x = 0.0
        approach_goal.target_point.point.y = 0.0
        approach_goal.target_point.point.z = 0.0
        
        rospy.loginfo("Sending approach goal...")
        self.approach_client.send_goal_and_wait(approach_goal)
        
        approach_result = self.approach_client.get_result()
        
        if not approach_result or not approach_result.action_success:
            rospy.logerr("Approach action failed")
            return 'failure'
            
        rospy.loginfo(f"Approach succeeded. Forward distance: {approach_result.forward_distance}")
        
        # ===== STEP 2: GRASP =====
        grasp_goal = ManipGraspGoal()
        grasp_goal.forward_distance = approach_result.forward_distance
        
        rospy.loginfo("Sending grasp goal...")
        self.grasp_client.send_goal_and_wait(grasp_goal)
        
        grasp_result = self.grasp_client.get_result()
        
        if not grasp_result or not grasp_result.action_success:
            rospy.logerr("Grasp action failed")
            return 'failure'
        
        # ===== STEP 3: RETRACT =====
        retract_goal = ManipRetractGoal()
        rospy.loginfo("Sending Retract goal")
        self.retract_client.send_goal_and_wait(retract_goal)

        retract_result = self.retract_client.get_result()

        if not retract_result or not retract_result.action_success:
            rospy.loggerr("retract action failed")
            return 'failure'
        
        rospy.loginfo(f"Pickup complete! Publishing holding_object_id = {object_id}")
        msg = Int32()
        msg.data = int(object_id)
        self.holding_pub.publish(msg)
 
        return 'success'


class PickupGoalSuccess(smach.State):
    def __init__(self):        
        smach.State.__init__(self, outcomes=['action_complete'])
        self.result_pub = rospy.Publisher("/actions/result", UNSWActionResult, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo('Pickup Action Succeeded')
        result = UNSWActionResult()
        result.action_name = "manip_pickup"
        result.action_result = True
        self.result_pub.publish(result)
        return 'action_complete'


class PickupGoalFailure(smach.State):
    def __init__(self):        
        smach.State.__init__(self, outcomes=['action_failed'])
        self.result_pub = rospy.Publisher("/actions/result", UNSWActionResult, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo('Pickup action has failed')
        result = UNSWActionResult()
        result.action_name = "manip_pickup"
        result.action_result = False
        self.result_pub.publish(result)
        return 'action_failed'

class PlaceGoalAction(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'],input_keys = ['data'])
        self.client = actionlib.SimpleActionClient("manip_place", ManipPlacingAction)
        if not self.client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Timeout waiting for manip_placing/goal action server")
            assert False, "Timeout waiting for action server"

    def execute(self, userdata):
        rospy.loginfo('Executing Place Goal Action State')
        self.client.wait_for_server()

        goal = ManipPlacingGoal()
        goal.direction = userdata.data['direction']
        goal.target_point.header.frame_id = userdata.data['target_point']['header']['frame_id']
        goal.target_point.point.x = userdata.data['target_point']['point']['x']     
        goal.target_point.point.y = userdata.data['target_point']['point']['y'] 
        goal.target_point.point.z = userdata.data['target_point']['point']['z']          

        print(goal)
        self.client.send_goal_and_wait(goal)

        self.client.wait_for_result()
        result = self.client.get_result()
        print("Action Outcome : {}".format(result))
        
        if (result.action_success) :        
            return 'success'
        else :
            return 'failure'

class PlaceGoalSuccess(smach.State):
    def __init__(self):        
        smach.State.__init__(self, outcomes=['action_complete'])
        self.result_pub = rospy.Publisher("/actions/result", UNSWActionResult, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo('Placing Action Suceeded')    
        result = UNSWActionResult()
        result.action_name = "manip_placing"
        result.action_result = True
        self.result_pub.publish(result)
        return 'action_complete'

class PlaceGoalFailure(smach.State):
    def __init__(self):        
        smach.State.__init__(self, outcomes=['action_failed'])
        self.result_pub = rospy.Publisher("/actions/result", UNSWActionResult, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo('Placing action has failed')
        result = UNSWActionResult()
        result.action_name = "manip_placing"
        result.action_result = False
        self.result_pub.publish(result)
        return 'action_failed'

class GazeAction(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.client = actionlib.SimpleActionClient("manip_gaze_at", ManipGazeAtAction)
        if not self.client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Timeout waiting for manip_gaze_at/goal action server")
            assert False, "Timeout waiting for action server"

    def execute(self, userdata):
        rospy.loginfo('Executing Gaze Action State')
        self.client.wait_for_server()

        goal = ManipGazeAtGoal()
        goal.target_point.header.frame_id = userdata.data['target_point']['header']['frame_id']
        goal.target_point.point.x = userdata.data['target_point']['point']['x']     
        goal.target_point.point.y = userdata.data['target_point']['point']['y'] 
        goal.target_point.point.z = userdata.data['target_point']['point']['z']          

        print(goal)
        self.client.send_goal_and_wait(goal)

        self.client.wait_for_result()
        result = self.client.get_result()
        print("Action Outcome : {}".format(result))
        
        if (result.action_success) :        
            return 'success'
        else :
            return 'failure'

class GazeSuccess(smach.State):
    def __init__(self):        
        smach.State.__init__(self, outcomes=['action_complete'])
        self.result_pub = rospy.Publisher("/actions/result", UNSWActionResult, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo('Gazing Action Suceeded')    
        result = UNSWActionResult()
        result.action_name = "manip_gaze"
        result.action_result = True
        self.result_pub.publish(result)
        return 'action_complete'

class GazeFailure(smach.State):
    def __init__(self):        
        smach.State.__init__(self, outcomes=['action_failed'])
        self.result_pub = rospy.Publisher("/actions/result", UNSWActionResult, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo('Gazing action has failed')
        result = UNSWActionResult()
        result.action_name = "manip_gaze"
        result.action_result = False
        self.result_pub.publish(result)
        return 'action_failed'

# class OpenDoorAction(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['success', 'failure'])
#         client = actionlib.SimpleActionClient(
#                             "/hsrb/arm_trajectory_controller/follow_joint_trajectory", 
#                             control_msgs.msg.FollowJointTrajectoryAction)
#         client.wait_for_server()

#         # TODO: figure out what goes here need to fill 'goal ='

#         print(goal)
#         client.send_goal(goal)RetractActionAction


#     def execute(self, userdata):
#         rospy.loginfo('Executing Pickup Goal Action State')
#         # Perform some operations
#         # If succeeded return success if failed return failure
    
#         return 'success'

# class OpenCupboardAction(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['success', 'failure'])
#         client = actionlib.SimpleActionClient(
#                             "/hsrb/arm_trajectory_controller/follow_joint_trajectory", 
#                             control_msgs.msg.FollowJointTrajectoryAction)
#         client.wait_for_server()

#         # TODO: figure out what goes here need to fill 'goal ='

#         print(goal)
#         client.send_goal(goal)

#         result = client.wait_for_result()
#         print("Action Outcome : {}".format(result))


#     def execute(self, userdata):
#         rospy.loginfo('Executing Pickup Goal Action State')
#         # Perform some operations
#         # If succeeded return success if failed return failure
    
#         return 'success'

# class OpenDishwasherAction(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['success', 'failure'])
#         client = actionlib.SimpleActionClient(
#                             "/hsrb/arm_trajectory_controller/follow_joint_trajectory", 
#                             control_msgs.msg.FollowJointTrajectoryAction)
#         client.wait_for_server()

#         # TODO: figure out what goes here need to fill 'goal ='

#         print(goal)
#         client.send_goal(goal)

#         result = client.wait_for_result()
#         print("Action Outcome : {}".format(result))


#     def execute(self, userdata):
#         rospy.loginfo('Executing Pickup Goal Action State')
#         # Perform some operations
#         # If succeeded return success if failed return failure
    
#         return 'success'

# class PullRackAction(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['success', 'failure'])
#         client = actionlib.SimpleActionClient(
#                             "/hsrb/arm_trajectory_controller/follow_joint_trajectory", 
#                             control_msgs.msg.FollowJointTrajectoryAction)
#         client.wait_for_server()

#         # TODO: figure out what goes here need to fill 'goal ='

#         print(goal)
#         client.send_goal(goal)

#         result = client.wait_for_result()
#         print("Action Outcome : {}".format(result))


#     def execute(self, userdata):
#         rospy.loginfo('Executing Pickup Goal Action State')
#         # Perform some operations
#         # If succeeded return success if failed return failure
    
#         return 'success'

# class CloseDishwasherAction(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['success', 'failure'])
#         client = actionlib.SimpleActionClient(
#                             "/hsrb/arm_trajectory_controller/follow_joint_trajectory", 
#                             control_msgs.msg.FollowJointTrajectoryAction)
#         client.wait_for_server()

#         # TODO: figure out what goes here need to fill 'goal ='

#         print(goal)
#         client.send_goal(goal)

#         result = client.wait_for_result()
#         print("Action Outcome : {}".format(result))


#     def execute(self, userdata):
#         rospy.loginfo('Executing Pickup Goal Action State')
#         # Perform some operations
#         # If succeeded return success if failed return failure
    
#         return 'success'

# class PourLiquidAction(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['success', 'failure'])
#         client = actionlib.SimpleActionClient(
#                             "/hsrb/arm_trajectory_controller/follow_joint_trajectory", 
#                             control_msgs.msg.FollowJointTrajectoryAction)
#         client.wait_for_server()

#         # TODO: figure out what goes here need to fill 'goal ='

#         print(goal)
#         client.send_goal(goal)

#         result = client.wait_for_result()
#         print("Action Outcome : {}".format(result))


#     def execute(self, userdata):
#         rospy.loginfo('Executing Pickup Goal Action State')
#         # Perform some operations
#         # If succeeded return success if failed return failure
    
#         return 'success'

# class PourSolidAction(smach.State):
    # def __init__(self):
    #     smach.State.__init__(self, outcomes=['success', 'failure'])
    #     client = actionlib.SimpleActionClient(
    #                         "/hsrb/arm_trajectory_controller/follow_joint_trajectory", 
    #                         control_msgs.msg.FollowJointTrajectoryAction)
    #     client.wait_for_server()

    #     # TODO: figure out what goes here need to fill 'goal ='

    #     print(goal)
    #     client.send_goal(goal)

    #     result = client.wait_for_result()
    #     print("Action Outcome : {}".format(result))


    # def execute(self, userdata):
    #     rospy.loginfo('Executing Pickup Goal Action State')
    #     # Perform some operations
    #     # If succeeded return success if failed return failure
    
    #     return 'success'

def main():
    rospy.init_node('manipulation_client_smach')
    sm = smach.StateMachine(outcomes = ["action_success"])

    with sm:
        smach.StateMachine.add('WAIT_FOR_ACTION', 
                               WaitForAction(), 
                               transitions={'approach_goal_action':'APPROACH_GOAL_ACTION',
                                            'retract_action':'RETRACT_ACTION',
                                            'grasp_goal_action':'GRASP_GOAL_ACTION',
                                            'pickup_goal_action':'PICKUP_GOAL_ACTION',                                            
                                            'place_goal_action': 'PLACE_GOAL_ACTION',
                                            'gaze_action':'GAZE_ACTION',
                                            # 'open_door_action': 'OPEN_DOOR_ACTION',
                                            # 'open_cupboard_action': 'OPEN_CUPBOARD_ACTION',
                                            # 'open_dishwasher_action': 'OPEN_DISHWASHER_ACTION',
                                            # 'pull_rack_action': 'PULL_RACK_ACTION',
                                            # 'close_dishwasher_action': 'CLOSE_DISHWASHER_ACTION',
                                            # 'pour_liquid_action': 'POUR_LIQUID_ACTION',
                                            # 'pour_solid_action': 'POUR_SOLID_ACTION',
                                            'waiting': 'WAIT_FOR_ACTION'})

        smach.StateMachine.add('APPROACH_GOAL_ACTION', 
                               ApproachGoalAction(), 
                               transitions={'success':'APPROACH_GOAL_SUCCESS',
                                            'failure': 'APPROACH_GOAL_FAILURE',
                                            })

        smach.StateMachine.add('APPROACH_GOAL_SUCCESS', 
                               ApproachGoalSuccess(), 
                               transitions={'action_complete':'WAIT_FOR_ACTION',                                            
                                            })

        smach.StateMachine.add('APPROACH_GOAL_FAILURE', 
                               ApproachGoalFailure(), 
                               transitions={'action_failed':'WAIT_FOR_ACTION',                                            
                                            })

        smach.StateMachine.add('RETRACT_ACTION', 
                               RetractAction(), 
                               transitions={'success':'RETRACT_SUCCESS',
                                            'failure': 'RETRACT_FAILURE',
                                            })
        smach.StateMachine.add('RETRACT_SUCCESS', 
                               RetractSuccess(), 
                               transitions={'action_complete':'WAIT_FOR_ACTION',                                            
                                            })

        smach.StateMachine.add('RETRACT_FAILURE', 
                               RetractFailure(), 
                               transitions={'action_failed':'WAIT_FOR_ACTION',                                            
                                            })
        
        smach.StateMachine.add('GRASP_GOAL_ACTION', 
                               GraspGoalAction(), 
                               transitions={'success':'GRASP_GOAL_SUCCESS',
                                            'failure': 'GRASP_GOAL_FAILURE',
                                            })
        smach.StateMachine.add('GRASP_GOAL_SUCCESS', 
                               GraspGoalSuccess(), 
                               transitions={'action_complete':'WAIT_FOR_ACTION',                                            
                                            })

        smach.StateMachine.add('GRASP_GOAL_FAILURE', 
                               GraspGoalFailure(), 
                               transitions={'action_failed':'WAIT_FOR_ACTION',                                            
                                            })

        smach.StateMachine.add('PICKUP_GOAL_ACTION', 
                               PickupGoalAction(), 
                               transitions={'success':'PICKUP_GOAL_SUCCESS',
                                            'failure': 'PICKUP_GOAL_FAILURE',
                                            })

        smach.StateMachine.add('PICKUP_GOAL_SUCCESS', 
                               PickupGoalSuccess(), 
                               transitions={'action_complete':'WAIT_FOR_ACTION',                                            
                                            })

        smach.StateMachine.add('PICKUP_GOAL_FAILURE', 
                               PickupGoalFailure(), 
                               transitions={'action_failed':'WAIT_FOR_ACTION',                                            
                                            })                        
        
        smach.StateMachine.add('PLACE_GOAL_ACTION', 
                               PlaceGoalAction(), 
                               transitions={'success':'PLACE_GOAL_SUCCESS',
                                            'failure': 'PLACE_GOAL_FAILURE',
                                            })

        smach.StateMachine.add('PLACE_GOAL_SUCCESS', 
                               PlaceGoalSuccess(), 
                               transitions={'action_complete':'WAIT_FOR_ACTION',                                            
                                            })

        smach.StateMachine.add('PLACE_GOAL_FAILURE', 
                               PlaceGoalFailure(), 
                               transitions={'action_failed':'WAIT_FOR_ACTION',                                            
                                            })
        
        smach.StateMachine.add('GAZE_ACTION', 
                               GazeAction(), 
                               transitions={'success':'GAZE_SUCCESS',
                                            'failure': 'GAZE_FAILURE',
                                            })
        smach.StateMachine.add('GAZE_SUCCESS', 
                               GazeSuccess(), 
                               transitions={'action_complete':'WAIT_FOR_ACTION',                                            
                                            })

        smach.StateMachine.add('GAZE_FAILURE', 
                               GazeFailure(), 
                               transitions={'action_failed':'WAIT_FOR_ACTION',                                            
                                            })

        # smach.StateMachine.add('OPEN_DOOR_ACTION', 
        #                        WaitForAction(), 
        #                        transitions={'success':'SUCCESS',
        #                                     'failure': 'FAILURE',
        #                                     })
        
        # smach.StateMachine.add('OPEN_CUPBOARD_ACTION', 
        #                        WaitForAction(), 
        #                        transitions={'success':'SUCCESS',
        #                                     'failure': 'FAILURE',
        #                                     })

        # smach.StateMachine.add('OPEN_DISHWASHER_ACTION', 
        #                        WaitForAction(), 
        #                        transitions={'success':'SUCCESS',
        #                                     'failure': 'FAILURE',
        #                                     })

        # smach.StateMachine.add('PULL_RACK_ACTION', 
        #                        WaitForAction(), 
        #                        transitions={'success':'SUCCESS',
        #                                     'failure': 'FAILURE',
        #                                     })

        # smach.StateMachine.add('CLOSE_DISHWASHER_ACTION', 
        #                        WaitForAction(), 
        #                        transitions={'success':'SUCCESS',
        #                                     'failure': 'FAILURE',
        #                                     })

        # smach.StateMachine.add('POUR_LIQUID_ACTION', 
        #                        WaitForAction(), 
        #                        transitions={'success':'SUCCESS',
        #                                     'failure': 'FAILURE',
        #                                     })

        # smach.StateMachine.add('POUR_SOLID_ACTION', 
        #                        WaitForAmanip_placingction(), 
        #                        transitions={'success':'SUCCESS',
        #                                     'failure': 'FAILURE',
        #                                     })

    sis = smach_ros.IntrospectionServer('server_name',sm,'/SM_ROOT')
    sis.start()

    outcome = sm.execute()
    rospy.logwarn(f"State machine finished with outcome: {outcome}")
    rospy.spin()
    sis.stop()
if __name__ == '__main__':
    main()