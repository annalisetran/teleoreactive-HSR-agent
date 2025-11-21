#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import actionlib
import tf.transformations
import sys
import os
import tf2_geometry_msgs

from std_msgs.msg import String, Int8, Bool
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PointStamped, Point, Pose, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from speech_to_text.msg import speech_recognition
from manip_srv.srv import approach_object, grasp_object, placing_object, center_object_move, view_pose, retract_arm, receive_pose

from unsw_vision_msgs.srv import ViewObjects
from unsw_vision_msgs.msg import DetectionList, ObjectDetection

KITCHEN_COUNTER_VIEW_POSE = Pose(Point(-9.60, -7.42, 0),Quaternion(0.000, 0.000, 0.726, 0.688))
KITCHEN_COUNTER_PICKUP_POSE = Pose(Point(-9.413, -8.465, 0),Quaternion(0.000, 0.000, 1.000, 0.029))

#TABLE_PLACE_POSE = Pose(Point(-9.333,-8.282, 0.0),Quaternion(0,0,0.720,0.694))
TABLE_PLACE_POSE = Pose(Point(-7.782,-5.317, 0.0),Quaternion(0.000, 0.000, -0.699, 0.715))

SPOON_PLACE_POINT = Point(-7.660689353942871,-6.681951522827148, 0.9)
BOWL_PLACE_POINT = Point(-7.749194145202637, -6.617109298706055, 0.9)

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
        start_topic = rospy.get_param("/serve_breakfast_controller/start_topic")
        self.task_started = False
        rospy.loginfo("init wrist sub callback")
        self.wrist_sub = rospy.Subscriber(start_topic, Int8, callback=self.wrist_callback, queue_size=1)
        
    def wrist_callback(self, msg: Int8):
        rospy.logwarn("Task started!")
        self.task_started = True
    def execute(self, ud):
        if self.task_started:
            self.myTalker.talk("Serve breakfast task started. Waiting for the door to be opened.")
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
            self.myTalker.talk("I detected the door opening. I'm moving to the kitchen now.")
            return 'door_open'
        if not self.door_open:
            return 'door_closed'

class NavToKitchen(smach.State):
    def __init__(self, myTalker):
        smach.State.__init__(self, outcomes=['nav_success'])
        self.myTalker = myTalker

        self.cli = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        # if nothing heard back from the server in 5 seconds then kill the node
        if not self.cli.wait_for_server(rospy.Duration(5.0)):
            rospy.loginfo("Timeout waiting for /move_base/move action server")
            assert False, "Timeout waiting for action server"
        
        self.counter_view_pose = KITCHEN_COUNTER_VIEW_POSE
        
    def execute(self, ud):
        self.myTalker.talk("I moving to the kitchen now.")

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose = self.counter_view_pose

        goal = MoveBaseGoal()
        goal.target_pose = pose

        self.cli.send_goal_and_wait(goal)

        navigation_state = self.cli.get_state()
        if navigation_state == GoalStatus.SUCCEEDED:
            self.myTalker.talk("I have arrived at the kitchen, I will now pick up breakfast items.")
            rospy.sleep(2)
            return 'nav_success'
        if navigation_state == GoalStatus.ABORTED or navigation_state == GoalStatus.REJECTED:
            return 'nav_fail'
        
class FindBowl(smach.State):
    def __init__(self, myTalker):
        smach.State.__init__(self, outcomes=['bowl_found', 'received_bowl'], output_keys=['bowl_point'])
        self.myTalker = myTalker
        # view pose service
        rospy.wait_for_service("/unsw/manip/view_pose", timeout=5)
        self.view_pose_service = rospy.ServiceProxy("/unsw/manip/view_pose", view_pose)

        # subscribe to vision
        self.vision_topic = rospy.get_param('serve_breakfast_controller/vision_topic')
        self.bowl_point = None

        self.vision_subscriber = rospy.Subscriber(self.vision_topic, DetectionList, self.vision_callback, queue_size=1)

        rospy.wait_for_service('/unsw/manip/receive_pose', timeout=5)
        self.receive_pose_service = rospy.ServiceProxy("/unsw/manip/receive_pose", receive_pose)

    def vision_callback(self, msg:DetectionList):
        for o in msg.objects:
            if o.object_class == 'bowl' and o.position.x != 0:
                self.bowl_point = o.position

    def execute(self, ud):
        # move to view pose
        arm_lift_joint = 0.40
        head_tilt_joint = -0.20
        self.view_pose_service(arm_lift_joint, head_tilt_joint)

        rospy.sleep(5)
        # if bowl is found transition to next state
        if self.bowl_point != None:
            ud.bowl_point = self.bowl_point
            return 'bowl_found'
        else:
            self.myTalker.talk("I can't find the Bowl, could you please hold the bowl in my gripper and then press my fingertips.")
            self.receive_pose_service()
            return 'received_bowl'
            
        
        
class PickUpBowl(smach.State):
    def __init__(self, myTalker):
        smach.State.__init__(self, outcomes=['bowl_pickup_success', 'bowl_pickup_fail'], input_keys=['bowl_point'])
        self.myTalker = myTalker

        self.home_pose_pub = rospy.Publisher('/init_pose', String)

        self.cli = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        # if nothing heard back from the server in 5 seconds then kill the node
        if not self.cli.wait_for_server(rospy.Duration(5.0)):
            rospy.loginfo("Timeout waiting for /move_base/move action server")
            assert False, "Timeout waiting for action server"

        rospy.wait_for_service('/unsw/manip/approach_object', timeout=5)
        rospy.wait_for_service('unsw/manip/grasp_object', timeout=5)
        rospy.wait_for_service('unsw/manip/retract_arm', timeout=5)

        self.approach_service = rospy.ServiceProxy("/unsw/manip/approach_object", approach_object)
        self.grasp_service = rospy.ServiceProxy("/unsw/manip/grasp_object", grasp_object)
        self.retract_service = rospy.ServiceProxy('unsw/manip/retract_arm', retract_arm)

        self.kitchen_counter_pickup_pose = KITCHEN_COUNTER_PICKUP_POSE

    def execute(self, ud):
        # move to home_pose
        self.home_pose_pub.publish("home")

        rospy.sleep(1)
        # moves to pickup pose
        kitchen_counter_pickup_pose_stamped = PoseStamped()
        kitchen_counter_pickup_pose_stamped.header.seq = 0
        kitchen_counter_pickup_pose_stamped.header.stamp = rospy.Time.now()
        kitchen_counter_pickup_pose_stamped.header.frame_id = "map"
        kitchen_counter_pickup_pose_stamped.pose = self.kitchen_counter_pickup_pose
        goal = MoveBaseGoal()
        goal.target_pose = kitchen_counter_pickup_pose_stamped
        self.cli.send_goal_and_wait(goal)

        # attempts to pickup
        pick_up_point_stamped = PointStamped()
        pick_up_point_stamped.header.seq = 0
        pick_up_point_stamped.header.stamp = rospy.Time.now()
        pick_up_point_stamped.header.frame_id = "map"
        pick_up_point_stamped.point = ud.bowl_point
        approach = self.approach_service('front-vertical', pick_up_point_stamped)
        grasp = self.grasp_service(approach.forward_distance)

        self.retract_service()
        return 'bowl_pickup_success'
        if grasp.success:
            return 'bowl_pickup_success'
        else:
            return 'bowl_pickup_fail'
        
class PlaceBowl(smach.State):
    def __init__(self, myTalker):
        smach.State.__init__(self, outcomes=['bowl_placed_success'])
        self.myTalker = myTalker
        self.bowl_place_point = BOWL_PLACE_POINT

        rospy.wait_for_service("/unsw/manip/placing_down/table", timeout=5)
        self.place_service = rospy.ServiceProxy("/unsw/manip/placing_down/table", placing_object)
    
        # nav to table place pose
        self.cli = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        # if nothing heard back from the server in 5 seconds then kill the node
        if not self.cli.wait_for_server(rospy.Duration(5.0)):
            rospy.loginfo("Timeout waiting for /move_base/move action server")
            assert False, "Timeout waiting for action server"

        self.home_pose_pub = rospy.Publisher('/init_pose', String)

        self.table_place_pose = TABLE_PLACE_POSE

    def execute(self, ud):     
        self.myTalker.talk("Let me place the bowl down for you.")
        # move to table 
        table_place_pose_stamped = PoseStamped()
        table_place_pose_stamped.header.seq = 0
        table_place_pose_stamped.header.stamp = rospy.Time.now()
        table_place_pose_stamped.header.frame_id = "map"
        table_place_pose_stamped.pose = self.table_place_pose

        goal = MoveBaseGoal()
        goal.target_pose = table_place_pose_stamped
        self.cli.send_goal_and_wait(goal)

        rospy.loginfo("start placing")
        # place bowl
        bowl_point_stamped = PointStamped()
        bowl_point_stamped.header.seq = 0
        bowl_point_stamped.header.stamp = rospy.Time.now()
        bowl_point_stamped.header.frame_id = "map"
        bowl_point_stamped.point = self.bowl_place_point

        place_result = self.place_service(bowl_point_stamped, 'top')
        if place_result:
            self.home_pose_pub.publish("home")
            return 'bowl_placed_success' 
        
class FindSpoon(smach.State):
    def __init__(self, myTalker):
        smach.State.__init__(self, outcomes=['spoon_found', 'received_spoon'], output_keys=['spoon_point'])
        self.myTalker = myTalker
        self.vision_topic = rospy.get_param('serve_breakfast_controller/vision_topic')
        self.spoon_point = None

        self.cli = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        # if nothing heard back from the server in 5 seconds then kill the node
        if not self.cli.wait_for_server(rospy.Duration(5.0)):
            rospy.loginfo("Timeout waiting for /move_base/move action server")
            assert False, "Timeout waiting for action server"
        
        self.counter_view_pose = KITCHEN_COUNTER_VIEW_POSE
        vision_sub = rospy.Subscriber(self.vision_topic, DetectionList, self.vision_callback, queue_size=1)

        rospy.wait_for_service('/unsw/manip/view_pose', timeout=5)
        self.view_pose_service = rospy.ServiceProxy("/unsw/manip/view_pose", view_pose)

        rospy.wait_for_service('/unsw/manip/receive_pose', timeout=5)
        self.receive_pose_service = rospy.ServiceProxy("/unsw/manip/receive_pose", receive_pose)

    def vision_callback(self, msg:DetectionList):
        for o in msg.objects:
            if o.object_class == 'spoon' and o.position.x != 0:
                self.spoon_point = o.position

    def execute(self, ud):
        # move to counter view pose
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose = self.counter_view_pose
        goal = MoveBaseGoal()
        goal.target_pose = pose
        self.cli.send_goal_and_wait(goal)


        arm_lift_joint = 0.40
        head_tilt_joint = -0.2
        self.view_pose_service(arm_lift_joint, head_tilt_joint)

        rospy.sleep(5)

        # updates spoon point 
        if not self.spoon_point == None:
            ud.spoon_point = self.spoon_point
            return 'spoon_found'
        else:
            self.myTalker.talk("I can't find the spoon, could you please hold the spoon in my gripper and then press my fingertips.")
            self.receive_pose_service()
            return 'received_spoon'
            

class PickUpSpoon(smach.State):
    def __init__(self, myTalker):
        smach.State.__init__(self, outcomes=['spoon_pickup_success'], input_keys=['spoon_point'])
        self.myTalker = myTalker

        self.cli = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        # if nothing heard back from the server in 5 seconds then kill the node
        if not self.cli.wait_for_server(rospy.Duration(5.0)):
            rospy.loginfo("Timeout waiting for /move_base/move action server")
            assert False, "Timeout waiting for action server"
        
        self.counter_pickup_pose = KITCHEN_COUNTER_PICKUP_POSE

        rospy.wait_for_service('/unsw/manip/approach_object', timeout=5)
        rospy.wait_for_service('unsw/manip/grasp_object', timeout=5)
        rospy.wait_for_service('unsw/manip/retract_arm', timeout=5)

        self.approach_service = rospy.ServiceProxy("/unsw/manip/approach_object", approach_object)
        self.grasp_service = rospy.ServiceProxy("/unsw/manip/grasp_object", grasp_object)
        self.retract_service = rospy.ServiceProxy('/unsw/manip/retract_arm', retract_arm)

    def execute(self, ud):
        # Move to counter pickup pose
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose = self.counter_pickup_pose
        goal = MoveBaseGoal()
        goal.target_pose = pose
        self.cli.send_goal_and_wait(goal)
        self.myTalker.talk("I am picking up the spoon")
        # attempt to pick up spoon
        pick_up_point_stamped = PointStamped()
        pick_up_point_stamped.header.seq = 0
        pick_up_point_stamped.header.stamp = rospy.Time.now()
        pick_up_point_stamped.header.frame_id = "map"
        pick_up_point_stamped.point = ud.spoon_point
        approach = self.approach_service('top', pick_up_point_stamped)
        grasp = self.grasp_service(approach)

        self.retract_service()

        if grasp.success:
            return 'spoon_pickup_success'         
        return 'spoon_pickup_success'
class PlaceSpoon(smach.State):
    def __init__(self, myTalker):
        smach.State.__init__(self, outcomes=['spoon_placed_success'])
        self.myTalker = myTalker
        self.spoon_place_point = SPOON_PLACE_POINT

        rospy.wait_for_service("/unsw/manip/placing_down/table", timeout=5)
        self.place_service = rospy.ServiceProxy("/unsw/manip/placing_down/table", placing_object)
    
        # nav to table place pose
        self.cli = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        # if nothing heard back from the server in 5 seconds then kill the node
        if not self.cli.wait_for_server(rospy.Duration(5.0)):
            rospy.loginfo("Timeout waiting for /move_base/move action server")
            assert False, "Timeout waiting for action server"

        self.table_place_pose = TABLE_PLACE_POSE

    def execute(self, ud):
        self.myTalker.talk("Let me place the spoon down for you.")
        # move to table 
        table_place_pose_stamped = PoseStamped()
        table_place_pose_stamped.header.seq = 0
        table_place_pose_stamped.header.stamp = rospy.Time.now()
        table_place_pose_stamped.header.frame_id = "map"
        table_place_pose_stamped.pose = self.table_place_pose

        goal = MoveBaseGoal()
        goal.target_pose = table_place_pose_stamped
        self.cli.send_goal_and_wait(goal)

        # place spoon
        spoon_point_stamped = PointStamped()
        spoon_point_stamped.header.seq = 0
        spoon_point_stamped.header.stamp = rospy.Time.now()
        spoon_point_stamped.header.frame_id = "map"
        spoon_point_stamped.point = self.spoon_place_point

        place_result = self.place_service(spoon_point_stamped, 'top')
        if place_result:
            return 'spoon_placed_success'

def main():
    rospy.init_node('serve_breakfast_controller')
    myTalker = Talker()
    # Create a smach state machine
    sm = smach.StateMachine(outcomes=['serve_breakfast_success'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('START', 
                               Start(myTalker), 
                               transitions={'task_started':'DOOR_CHECK', 
                                            'waiting':'START'})
        smach.StateMachine.add('DOOR_CHECK', 
                               DoorCheck(myTalker), 
                               transitions={'door_open':'NAV_TO_KITCHEN', 
                                            'door_closed':'DOOR_CHECK'})
        smach.StateMachine.add('NAV_TO_KITCHEN', 
                               NavToKitchen(myTalker), 
                               transitions={'nav_success': 'FIND_BOWL'})
        smach.StateMachine.add('FIND_BOWL', 
                               FindBowl(myTalker), 
                               transitions={'bowl_found': 'PICK_UP_BOWL',
                                            'received_bowl':'PLACE_BOWL'})
        smach.StateMachine.add('PICK_UP_BOWL',
                               PickUpBowl(myTalker),
                               transitions={'bowl_pickup_success' : 'PLACE_BOWL',
                                            'bowl_pickup_fail': 'FIND_SPOON'})
        smach.StateMachine.add('PLACE_BOWL',
                               PlaceBowl(myTalker),
                               transitions={'bowl_placed_success' : 'FIND_SPOON'})
        smach.StateMachine.add('FIND_SPOON',
                               FindSpoon(myTalker),
                               transitions={'spoon_found':'PICK_UP_SPOON', 'received_spoon':'PLACE_SPOON'})
        smach.StateMachine.add("PICK_UP_SPOON",
                               PickUpSpoon(myTalker),
                               transitions={'spoon_pickup_success':'PLACE_SPOON'})
        smach.StateMachine.add("PLACE_SPOON",
                               PlaceSpoon(myTalker),
                               transitions={'spoon_placed_success':'serve_breakfast_success'})
    
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()
    rospy.logwarn(f"State machine finished with outcome: {outcome}")

    rospy.spin()
    sis.stop()
if __name__ == '__main__':
    main()

