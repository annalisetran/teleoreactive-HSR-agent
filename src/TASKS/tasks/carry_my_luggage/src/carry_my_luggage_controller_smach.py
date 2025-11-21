#!/usr/bin/env python3

import rospy
import math

import smach
import actionlib
import tf2_ros
import tf2_geometry_msgs

from std_msgs.msg import String, Int8, Bool
from unsw_vision_msgs.msg import DetectionList, BagPointDetection
from geometry_msgs.msg import PoseStamped, PointStamped, Point, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from speech_to_text.msg import speech_recognition

from manip_srv.srv import approach_object, grasp_object, placing_object, receive_pose, retract_arm, give_pose

from queue import Queue
import smach_ros
class Talker:
    def __init__(self):
        tts_topic = rospy.get_param('/carry_my_luggage_controller/tts_topic')
        self.talker = rospy.Publisher(tts_topic, String, queue_size=1)
    
    def talk(self, sentence):
        rospy.loginfo(f"saying: {sentence}")
        self.talker.publish(sentence)

class Start(smach.State):
    def __init__(self, myTalker, tfBuffer):
        smach.State.__init__(self, outcomes=['task_started', 'waiting'], input_keys=['start_pose']
                             , output_keys=['start_pose'])
        self.myTalker = myTalker
        self.tfBuffer = tfBuffer
        start_topic = rospy.get_param("/carry_my_luggage_controller/start_topic")
        self.task_started = False
        self.press_time = None
        rospy.loginfo("init wrist sub callback")
        self.wrist_sub = rospy.Subscriber(start_topic, Int8, callback=self.wrist_callback, queue_size=1)  
        
        
    def wrist_callback(self, msg: Int8):
        rospy.logwarn("Task started!")
        if not self.task_started:
            self.task_started = True
            self.press_time = rospy.Time.now().to_sec()
            self.myTalker.talk("Start in 10 seconds.")

    def execute(self, userdata):
        if self.task_started and rospy.Time.now().to_sec() - self.press_time > 10:
            # Get the robots current pose
            start_pose = PoseStamped()
            start_pose.pose = Pose()
            
            start_pose.pose.orientation.w = 1
            start_pose.header.stamp = rospy.Time.now()
            start_pose.header.frame_id = 'base_footprint'
            # transform to get the orientation of the robot in the map frame
            try:
                trans = self.tfBuffer.lookup_transform("map","base_footprint",rospy.Time.now(), rospy.Duration(2))
                transformed_point_stamped= tf2_geometry_msgs.do_transform_pose(start_pose, trans)
                userdata.start_pose = transformed_point_stamped
                rospy.loginfo(transformed_point_stamped)
            except Exception as e:
                rospy.loginfo("Failed transform point to footprint. {}".format(e))
                return 'waiting'


            return 'task_started'
        return 'waiting'

class FindBag(smach.State):
    def __init__(self, myTalker):
        smach.State.__init__(self, outcomes=['bag_found', 'bag_not_found'],
                             input_keys=['handle_coordinate', 'person_id'],
                             output_keys=['handle_coordinate', 'person_id'])
        self.myTalker = myTalker

        self.found_started = False
        self.scan_finished = False
        self.handle_coordinate = None
        self.person_id = None
        # Publishers
        self.bag_scan_ready_pub = rospy.Publisher('/bag_scan_ready', Bool, queue_size=1)
        
        pointing_topic = rospy.get_param("/carry_my_luggage_controller/pointing_topic")
        self.pointing_sub = rospy.Subscriber(pointing_topic, BagPointDetection, callback=self.pointing_callback, queue_size=1)

    def pointing_callback(self, msg: BagPointDetection):
        if msg.handleCoordinates.z == -1:
            rospy.loginfo("bag not found from scan")
            self.myTalker.talk("I was not able to find the bag. Please try again")
            self.handle_coordinate = None
            self.found_started = False
        else:
            self.person_id = msg.pointingPerson.id
            rospy.loginfo(f"Person id of the person pointing is {self.person_id}")
            self.handle_coordinate = msg.handleCoordinates
        self.scan_finished = True

    def execute(self, userdata):
        # publish to bag_scan_ready ONLY ONCE
        self.myTalker.talk("Please face me and point at the bag")
        if not self.found_started:
            rospy.loginfo("Publishing to bag_scan_ready")
            self.bag_scan_ready_pub.publish(Bool(True))
            self.found_started = True
        
        while not self.scan_finished:
            rospy.sleep(1)
            continue
        if self.handle_coordinate != None and self.person_id != None:
            userdata.handle_coordinate = Point()
            userdata.handle_coordinate.x = self.handle_coordinate.x
            userdata.handle_coordinate.y = self.handle_coordinate.y
            userdata.handle_coordinate.z = self.handle_coordinate.z

            userdata.person_id = self.person_id
            self.myTalker.talk("Ah yes, that bag! Let me pick it up!")
            return 'bag_found'
        else:
            return 'bag_not_found'
            


class MoveToBag(smach.State):
    def __init__(self, myTalker, tfBuffer):
        smach.State.__init__(self, outcomes=['move_to_bag_success', 'move_to_bag_fail']
                             , input_keys=['handle_coordinate'])
        self.myTalker = myTalker
        self.tfBuffer = tfBuffer
        self.cli = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        
        # if nothing heard back from the server in 5 seconds then kill the node
        if not self.cli.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Timeout waiting for /move_base/move action server")
            assert False, "Timeout waiting for action server"

        self.target_pose = PoseStamped()

    def execute(self, userdata):
        self.myTalker.talk("I am now moving to the bag")
        # Get the robots current pose
        robot_pose_stamped = PoseStamped()
        robot_pose_stamped.pose = Pose()
        
        robot_pose_stamped.pose.orientation.w = 1
        robot_pose_stamped.header.stamp = rospy.Time.now()
        robot_pose_stamped.header.frame_id = 'base_footprint'
        # transform to get the orientation of the robot in the map frame
        try:
            trans = self.tfBuffer.lookup_transform("map","base_footprint",rospy.Time.now(), rospy.Duration(2))
            robot_pose_stamped= tf2_geometry_msgs.do_transform_pose(robot_pose_stamped, trans)
            rospy.loginfo(robot_pose_stamped)
        except Exception as e:
            rospy.loginfo("Failed transform point to footprint. {}".format(e))
            return 'move_to_bag_fail'
        
        # Calculate point for robot to move so it is in front of the bag. 
        best_point = Point()
        robotX = robot_pose_stamped.pose.position.x
        robotY = robot_pose_stamped.pose.position.y
        bagX = userdata.handle_coordinate.x
        bagY = userdata.handle_coordinate.y
 
        mylen = math.sqrt((bagX - robotX)**2 + (bagY - robotY)**2)
        best_point.x = bagX + (robotX-bagX)/mylen * 0.6
        best_point.y = bagY + (robotY-bagY)/mylen * 0.6

        # Construct nav goal based on calculated point and pose 
        self.target_pose.header.seq = 0
        self.target_pose.header.stamp = rospy.Time.now()
        self.target_pose.header.frame_id = 'map'

        self.target_pose.pose.position.x = best_point.x
        self.target_pose.pose.position.y = best_point.y
        self.target_pose.pose.position.z = 0.0
        self.target_pose.pose.orientation = robot_pose_stamped.pose.orientation

        # Send goal to nav action server
        goal = MoveBaseGoal()
        goal.target_pose = self.target_pose
        self.cli.send_goal_and_wait(goal)
        

        navigation_state = self.cli.get_state()
        if navigation_state == GoalStatus.SUCCEEDED:
            self.myTalker.talk("I have reached the bag.")
            return 'move_to_bag_success'
        if navigation_state == GoalStatus.ABORTED or navigation_state == GoalStatus.REJECTED:
            return 'move_to_bag_fail'

class Approach(smach.State):
    def __init__(self, myTalker):
        smach.State.__init__(self, outcomes=['approach_fail', 'approach_success', 'approach_reattempt']
                             , input_keys=['handle_coordinate', 'forward_distance'],
                             output_keys=['forward_distance'])
        self.myTalker = myTalker

        rospy.wait_for_service('unsw/manip/approach_object', timeout=15)
        self.approach_service = rospy.ServiceProxy('unsw/manip/approach_object', approach_object)

        self.q = Queue()
        self.q.put('top')
        self.q.put('front-vertical')

    def execute(self, userdata):
        self.myTalker.talk("I am trying to pick up the bag")
        if self.q.empty():
            return 'approach_fail'
        
        point_stamped = PointStamped()
        point_stamped.header.seq = 0
        point_stamped.header.stamp = rospy.Time.now()
        point_stamped.header.frame_id = 'map'
        point_stamped.point = userdata.handle_coordinate

        approach_result = self.approach_service(self.q.get(), point_stamped)
        
        if approach_result.success:
            userdata.forward_distance = approach_result.forward_distance
            return 'approach_success'
        else:
            return 'approach_reattempt'

class PickUp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['bag_secured', 'reattempt'],
                             input_keys=['forward_distance'])

        rospy.wait_for_service('unsw/manip/grasp_object', timeout=15)
        self.grasp_service = rospy.ServiceProxy('unsw/manip/grasp_object', grasp_object)
        rospy.wait_for_service('unsw/manip/retract_arm', timeout=15)
        self.retract_service = rospy.ServiceProxy('unsw/manip/retract_arm', retract_arm)
        
    def execute(self, userdata):        
        # Top position moved by 3cm
        grasp_result = self.grasp_service(userdata.forward_distance + 0.06)
        retract_result = self.retract_service()
        if grasp_result.success and retract_result.success:
            return 'bag_secured'
        else:
            return 'reattempt'

class Receive(smach.State):
    def __init__(self, myTalker):
        smach.State.__init__(self, outcomes=['bag_secured', 'waiting'])
        self.myTalker = myTalker

        rospy.wait_for_service("unsw/manip/receive_pose")
        self.receive_service = rospy.ServiceProxy("unsw/manip/receive_pose", receive_pose)

    def execute(self, userdata):        
        self.myTalker.talk("Please place the bag in my hand!")

        receive_result = self.receive_service()

        if receive_result.success:
            return 'bag_secured'
        if not receive_result.success:
            self.myTalker.talk("Hurry up and put the bag in my hand!")
            return 'waiting'

class PersonFollowing(smach.State):
    def __init__(self, myTalker):
        smach.State.__init__(self, outcomes=['following', 'arrived']
                             , input_keys=['person_id'])
        self.myTalker = myTalker
        self.following_triggered = False
        follow_topic = rospy.get_param('/carry_my_luggage_controller/follow_topic')
        self.follow_pub = rospy.Publisher(follow_topic, String, queue_size = 1)
        self.follow_stop_pub = rospy.Publisher("/navigation/person_following/stop", String, queue_size=1)
        stt_topic = rospy.get_param('/carry_my_luggage_controller/stt_topic')
        self.listen_sub = rospy.Subscriber(stt_topic, speech_recognition, callback=self.listen_callback, queue_size=10)

        self.stop_heard = False
    
    def listen_callback(self, msg: speech_recognition):
        for word in msg.final_result.split():
            if word == "stop":
                self.stop_heard = True

    def execute(self, userdata):
        if not self.following_triggered:
            self.myTalker.talk("Please take me to the car. Say stop when we have reached the car")
            self.follow_pub.publish(String(userdata.person_id))
            self.following_triggered = True
          
        
        if self.stop_heard:
            self.follow_stop_pub.publish("a")
            return 'arrived'
        return 'following'
    
class ReturnBag(smach.State):
    def __init__(self, myTalker):
        smach.State.__init__(self, outcomes=['return_success', 'return_fail', 'return_fail_multiple'])
        self.myTalker = myTalker

        rospy.wait_for_service('unsw/manip/placing_down', timeout=5)
        self.place_service = rospy.ServiceProxy('unsw/manip/placing_down', placing_object)
        self.num_attempt = 0 
        
    def execute(self, userdata):

        place_point = PointStamped()

        place_point.header.seq = 0
        place_point.header.stamp = rospy.Time.now()
        place_point.header.frame_id = 'base_link'

        place_point.point.x = 0.5
        place_point.point.y = 0
        place_point.point.z = 0.3

        self.myTalker.talk("Thanks for leading me to the car. I will now return your bag. and head back after 5 seconds")
        place_result = self.place_service(place_point, "top")
        
        if place_result.success or self.num_attempt >= 3:
            rospy.sleep(8)
            return 'return_success'
        self.myTalker.talk("Please wait,I will drop again.")
        self.num_attempt += 1
        return 'return_fail'

class BagIntervention(smach.State):
    def __init__(self, myTalker):
        smach.State.__init__(self, outcomes=['intervention_complete'])
        self.myTalker = myTalker

        rospy.wait_for_service('unsw/manip/give_pose', timeout=5)
        self.give_bag_service = rospy.ServiceProxy('unsw/manip/give_pose', give_pose)
        
    def execute(self, userdata):

        # Make position of blinky holding bag with open hand

        self.myTalker.talk("I can't get the bag out of my hand, would you be able to take it for me?")
        intervention_result = self.give_bag_service()
        
        if intervention_result.success:
            return 'intervention_complete'
        self.myTalker.talk("The bag has not been taken. I will still return to the start point.")
        return 'intervention_complete'
        
class ReturnToStart(smach.State):
    def __init__(self, myTalker):
        smach.State.__init__(self, outcomes=['return_to_start_success', 'return_to_start_fail'],
                             input_keys=['start_pose'])
        self.myTalker = myTalker

        self.cli = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        
        # if nothing heard back from the server in 5 seconds then kill the node
        if not self.cli.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Timeout waiting for /move_base/move action server")
            assert False, "Timeout waiting for action server"

    def execute(self, userdata):
        goal = MoveBaseGoal()
        goal.target_pose = userdata.start_pose
        
        self.cli.send_goal_and_wait(goal)

        navigation_state = self.cli.get_state()
        if navigation_state == GoalStatus.SUCCEEDED:
            self.myTalker.talk("I have returned to the start.")
            return 'return_to_start_success'
        if navigation_state == GoalStatus.ABORTED or navigation_state == GoalStatus.REJECTED:
            return 'return_to_start_fail'

def main():
    rospy.init_node('carry_my_luggage_smach')
    myTalker = Talker()
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    sm = smach.StateMachine(outcomes = ["task_success"])

    with sm:
        smach.StateMachine.add('START', 
                               Start(myTalker,tfBuffer), 
                               transitions={'task_started':'FIND_BAG', 
                                            'waiting': 'START'})
        smach.StateMachine.add('FIND_BAG', 
                               FindBag(myTalker), 
                               transitions={'bag_found':'MOVE_TO_BAG',
                                            'bag_not_found':'RECEIVE'})
        smach.StateMachine.add('MOVE_TO_BAG',
                               MoveToBag(myTalker,tfBuffer),
                               transitions={'move_to_bag_success':'APPROACH',
                                            'move_to_bag_fail':'MOVE_TO_BAG'})
        smach.StateMachine.add('APPROACH', 
                               Approach(myTalker), 
                               transitions={'approach_fail':'RECEIVE',
                                           'approach_success':'PICKUP',
                                           'approach_reattempt':'APPROACH'})
        smach.StateMachine.add('PICKUP',
                               PickUp(),
                               transitions={'bag_secured':'PERSON_FOLLOWING',
                                            'reattempt':'APPROACH'})
        smach.StateMachine.add('RECEIVE',
                               Receive(myTalker),
                               transitions={'waiting':'RECEIVE',
                                            'bag_secured':'PERSON_FOLLOWING'})
        smach.StateMachine.add('PERSON_FOLLOWING',
                               PersonFollowing(myTalker),
                               transitions={'following':'PERSON_FOLLOWING',
                                            'arrived':'RETURN_BAG'})
        smach.StateMachine.add('RETURN_BAG',
                               ReturnBag(myTalker),
                               transitions={'return_success':'RETURN_TO_START',
                                            'return_fail': 'RETURN_BAG',
                                            'return_fail_multiple': 'BAG_INTERVENTION'})
        smach.StateMachine.add('BAG_INTERVENTION',
                               BagIntervention(myTalker),
                               transitions={'intervention_complete':'RETURN_TO_START'})
        smach.StateMachine.add('RETURN_TO_START',
                               ReturnToStart(myTalker),
                               transitions={'return_to_start_success':'task_success', 'return_to_start_fail': 'RETURN_TO_START'})

    sis = smach_ros.IntrospectionServer('server_name',sm,'/SM_ROOT')
    sis.start()

    outcome = sm.execute()
    rospy.logwarn(f"State machine finished with outcome: {outcome}")
    rospy.spin()
    sis.stop()
if __name__ == '__main__':
    main()
        

        
