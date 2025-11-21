#!/usr/bin/env python3
from __future__ import annotations
import rospy    
from std_msgs.msg import String, Int8, Bool
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from nav_msgs.msg import Odometry
from speech_to_text.msg import speech_recognition
from unsw_vision_msgs.msg import DetectionList, BagPointDetection
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from actionlib_msgs.msg import GoalID
import actionlib
import tf2_ros
import math
import tf2_geometry_msgs
import sys
from visualization_msgs.msg import Marker
import datetime
import hsrb_interface
# sys.path.remove('/home/robocupathome/.local/lib/python3.8/site-packages')
# sys.path.append('/home/robocupathome/workspace/robocup_ws/src/world_model/world_model_store/scripts')
# from dbase import DBase
# from regions_api import RegionsApi

from abc import ABC, abstractmethod

class Context:
    _state = None
    # past_pose = None
    
    def __init__(self, state: State) -> None:
        
        self.task_started = False
        self.cancelled = False
        self.pointing_result = None
        self.detection_list = None
        self.handle_coordinate = None
        self.pickup_results = "Initialised Not heard from manip yet"
        self.person_id = None
        self.person_coords = None
        self.car_reached = False
        self.release_results = None
        self.returned = False
        self.vosk_msg = ""
        self.pose_diffs = [1000, 1000, 1000]
        self.past_pose = None
        self.rescanned_bag = None
        self.state_transitioned = False
        self.scan_retry = False
        # TBD
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.pose_in_map = None
        

        # Subscribers
        self.wrist_sub = rospy.Subscriber('/start_task', Int8, callback=self.wrist_callback, queue_size=1)
        self.vosk_sub = rospy.Subscriber('speech_recognition/vosk_result', speech_recognition, callback=self.vosk_callback, queue_size=1)
        self.pointing_sub = rospy.Subscriber('/unsw_vision/detections/pointedBag', BagPointDetection, callback=self.pointing_callback, queue_size=1)
        self.vision_sub = rospy.Subscriber('/unsw_vision/detections/objects/positions', DetectionList, callback=self.vision_callback, queue_size=1)
        self.pickup_sub = rospy.Subscriber('/object_grasper/bag/pickup_status', String, callback=self.pickup_callback, queue_size=1)
        self.release_sub = rospy.Subscriber('/object_grasper/bag/release_status', String, callback=self.release_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber('/hsrb/odom', Odometry, callback=self.odom_callback, queue_size=1)
        self.bag_reached_sub = rospy.Subscriber("/move_near_reached", Bool, callback=self.bag_reached_callback, queue_size=1)
        
        # Publishers
        self.talker = rospy.Publisher("tts/phrase", String, queue_size=10)
        # self.bag_pub = rospy.Publisher("/object_grasp_action_server", PointStamped, queue_size=1)
        self.bag_pub = rospy.Publisher("/pickup_point", PointStamped, queue_size=1)
        self.follow_pub = rospy.Publisher("/navigation/person_following/id", String, queue_size = 1)
        self.follow_stop_pub = rospy.Publisher("/navigation/person_following/stop", String, queue_size=1)
        self.release_pub = rospy.Publisher("/object_grasper/bag/release_start", String, queue_size = 1)
        self.move_base_pub = rospy.Publisher( '/move_base_simple/goal', PoseStamped, queue_size=1 )
        self.move_head_pub = rospy.Publisher('hsrb/head_trajectory_controller/command', JointTrajectory, queue_size=1)
        self.move_base_near_pub = rospy.Publisher('/clicked_point', PointStamped, queue_size=1)
        self.bag_scan_ready_pub = rospy.Publisher('/bag_scan_ready', Bool, queue_size=1)
        self.maker_pub = rospy.Publisher('/bag_marker', Marker, queue_size=1)
        self.init_pose_pub = rospy.Publisher('/init_pose', String, queue_size=1)
        self.open_grasp_pub = rospy.Publisher('/open_grasp', String, queue_size=1)
        
        
        
        # db = DBase("/home/robocupathome/workspace/robocup_ws/src/world_model/world_model_store/scripts/database.ini")
        # region_api = RegionsApi()
        # self.inspection_point = region_api.get_poi_by_name(db.con_pool, "inspection point")[0]






        self.transition_to(state)
        rospy.loginfo('init content')

    def transition_to(self, state: State) -> None:
        self._state = state
        self.state_time = False
        self._state.context = self

    def say(self, sentence: str):
        rospy.loginfo(f"saying: {sentence}")
        self.talker.publish(sentence)

    def run(self):
        self._state.run()

    # Callbacks
    
    def bag_reached_callback(self, msg: GoalID):
        rospy.loginfo("Goal cancelled")
        self.cancelled = True
        
    def wrist_callback(self, msg: Int8):
        # rospy.loginfo("here")
        self.wrist_sub.unregister()
        self.say("I will start the task in 5 seconds")
        rospy.sleep(5)
        self.task_started = True
    
    def pointing_callback(self, msg: BagPointDetection):
        # rospy.loginfo("pointing callback")
        # if not finding a bag
        if msg.handleCoordinates.z == -1:
            rospy.loginfo("not finding a bag from scan")
            self.say("I was not able to find the bag. Please try again")
            self.state_transitioned = False
            self.handle_coordinate = None
        else:
            self.pointing_result = msg
            self.person_id = msg.pointingPerson.id
            rospy.loginfo(f"Person id of the person pointing is {self.person_id}")
            self.handle_coordinate = msg.handleCoordinates
    
    def pickup_callback(self, msg: String):
        self.pickup_results = msg.data

    def release_callback(self, msg: String):
        self.release_results = msg

    def vision_callback(self, msg: DetectionList):
        self.detection_list = msg
    
    def vosk_callback(self, msg: speech_recognition):
        self.vosk_msg = msg.final_result
    
    def odom_callback(self, msg: Odometry):
        if (self.past_pose == None):
            self.past_pose = msg.pose.pose
        else:
            self.pose_diffs = [abs(msg.pose.pose.position.x - self.past_pose.position.x), 
                               abs(msg.pose.pose.position.y - self.past_pose.position.y),
                               abs(msg.pose.pose.position.z - self.past_pose.position.z)]
            self.past_pose = msg.pose.pose
        myPoseStamp = PoseStamped()
        myPoseStamp.header = msg.header
        myPoseStamp.pose = msg.pose.pose
        if self.pose_in_map == None:
            try:
                trans = self.tfBuffer.lookup_transform("map",msg.header.frame_id,rospy.Time.now(), rospy.Duration(2))
                transformed_pose_stamped = tf2_geometry_msgs.do_transform_pose(myPoseStamp, trans)
                self.pose_in_map = transformed_pose_stamped
                print(transformed_pose_stamped)
            except Exception as e:
                rospy.loginfo(f"Carry lugguge failed to do transformation {e}")
            
            #print(trans)
        
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
        #self.start_words = ['start', 'go', 'begin', 'good', , 'come', 'pick']
        self.start_words = ['start', 'go', 'begin']

        if self.context.vosk_msg != None:
            for word in self.context.vosk_msg.split():
                if (word in self.start_words):
                    self.context.task_started = True
        if (self.context.task_started):
            #Don't delete next line
            rospy.sleep(1)
            self.getInitialHeadPose()
            self.context.init_pose_pub.publish(String("init"))
            self.context.transition_to(pointing())
            
    def getInitialHeadPose(self):
        rospy.loginfo("moving head to original pos")
        initialHeadPos = JointTrajectory()
        initialHeadPos.joint_names = ["head_pan_joint", "head_tilt_joint"]
        initialHeadPoint = JointTrajectoryPoint()
        initialHeadPoint.positions = [0.0, 0.0]
        initialHeadPoint.time_from_start = rospy.Duration(1)
        initialHeadPos.points = [initialHeadPoint]
        self.context.move_head_pub.publish(initialHeadPos)
        return initialHeadPos 
            

class pointing(State):
    
    def run(self):
        if (self.context.handle_coordinate != None):
            self.context.say("Ah yes, that bag. Let me pick it up")
            self.handle_stamped = PointStamped()
            self.handle_stamped.point = self.context.handle_coordinate
            self.handle_stamped.header.seq = 0
            self.handle_stamped.header.stamp = rospy.Time.now()
            self.handle_stamped.header.frame_id = 'map'
            self.context.say("start moving to the bag")
            self.context.cancelled = False
            
            # calc the point it should go to
            best_point = PointStamped()
            best_point.header = self.handle_stamped.header
            robotX = self.context.pose_in_map.pose.position.x
            robotY = self.context.pose_in_map.pose.position.y
            bagX = self.handle_stamped.point.x
            bagY = self.handle_stamped.point.y
            mylen = math.sqrt((bagX - robotX)**2 + (bagY - robotY)**2)
            best_point.point.x = bagX + (robotX-bagX)/mylen * 0.5
            best_point.point.y = bagY + (robotY-bagY)/mylen * 0.5
            
            best_point_marker = Marker()
            best_point_marker.header.frame_id = "map"
            best_point_marker.header.stamp = rospy.Time.now()
            best_point_marker.ns = "pickup robot point"
            best_point_marker.action = Marker.ADD
            best_point_marker.pose.position = best_point.point
            best_point_marker.pose.orientation.w = 1
            best_point_marker.lifetime.secs = 0
            best_point_marker.lifetime.nsecs = 0
            best_point_marker.type = 1
            best_point_marker.scale.x = 0.5
            best_point_marker.scale.y = 0.5
            best_point_marker.scale.z = 0.5
            best_point_marker.color.a = 1.0
            best_point_marker.color.r = 1
            best_point_marker.color.g = 0
            best_point_marker.color.b = 0
            self.context.maker_pub.publish(best_point_marker)
            
            
            bag_marker = Marker()
            bag_marker.header.frame_id = "map"
            bag_marker.header.stamp = rospy.Time.now()
            bag_marker.ns = "bag initial"
            bag_marker.action = Marker.ADD
            bag_marker.pose.position = self.context.handle_coordinate
            bag_marker.pose.orientation.w = 1
            bag_marker.lifetime.secs = 0
            bag_marker.lifetime.nsecs = 0
            bag_marker.type = 1
            bag_marker.scale.x = 0.5
            bag_marker.scale.y = 0.5
            bag_marker.scale.z = 0.5
            bag_marker.color.a = 1.0
            bag_marker.color.r = 0
            bag_marker.color.g = 0
            bag_marker.color.b = 1
            self.context.maker_pub.publish(bag_marker)
            
            
            self.context.move_base_near_pub.publish(best_point)
            
            #self.context.move_base_near_pub.publish(self.handle_stamped)
            self.detection_list_sub = rospy.Subscriber("/unsw_vision/detections/objects/coordinates", DetectionList, callback = self.detection_list_callback, queue_size=1)
            self.context.transition_to(movingToBag())
            # self.context.transition_to(bag_pickup())
        elif (self.context.state_transitioned == False):
            self.context.say("Please face me and point at the bag")
            self.context.bag_scan_ready_pub.publish(Bool(True))
            self.context.state_transitioned = True
    
    def detection_list_callback(self, msg:DetectionList):
        self.mindiff = 0.4**2
        for handle in list(filter(lambda obj: obj.object_class == "baghandle", msg.objects)):
            dx2 = (handle.position.x - self.context.handle_coordinate.x) **2
            dy2 = (handle.position.y - self.context.handle_coordinate.y) **2
            dz2 = (handle.position.z - self.context.handle_coordinate.z) **2
            self.context.say(f"New detection difference is {dx2},{dy2},{dz2}")
            rospy.loginfo(f"New detection difference is {dx2},{dy2},{dz2}")
            if (dx2 + dy2 + dz2) < self.mindiff and dz2 < 0.2:
                self.bagUpdatedP = handle.position
        rospy.loginfo(f"pos updated from {self.context.handle_coordinate} to {self.bagUpdatedP}")
        self.context.handle_coordinate = self.bagUpdatedP
class movingToBag(State):
    bagUpdatedP = Point()
    def run(self):
        if (self.context.cancelled):
            
            self.context.say("I've reached the bag ")
            
            #rescan here
            # rospy.sleep(1)
            
            
            # headPos = JointTrajectory()
            # headPos.joint_names = ["head_pan_joint", "head_tilt_joint"]
            # point1 = JointTrajectoryPoint()
            # point1.positions = [0, -0.8]
            # point1.time_from_start = rospy.Duration(secs=1)
            # point2 = JointTrajectoryPoint()
            # point2.positions = [0.6, -0.8]
            # point2.time_from_start = rospy.Duration(secs=4)
            # point3 = JointTrajectoryPoint()
            # point3.positions = [-0.6, -0.8]
            # point3.time_from_start = rospy.Duration(secs=12)
            # point4 = JointTrajectoryPoint()
            # point4.positions = [0.0, 0.0]
            # point4.time_from_start = rospy.Duration(secs=16)
            # headPos.points = [point1, point2, point3,point4]
            # self.context.move_head_pub.publish(headPos)
            # rospy.sleep(18)
            
            self.handle_stamped = PointStamped()
            self.handle_stamped.point = self.context.handle_coordinate
            self.handle_stamped.header.seq = 0
            self.handle_stamped.header.stamp = rospy.Time.now()
            self.handle_stamped.header.frame_id = 'map'
            self.context.say("I'm trying to pick up now, give me some time")
            rospy.loginfo(str(self.handle_stamped))
            
            bag_marker = Marker()
            bag_marker.header.frame_id = "map"
            bag_marker.header.stamp = rospy.Time.now()
            bag_marker.ns = "bag final"
            bag_marker.action = Marker.ADD
            bag_marker.pose.position = self.context.handle_coordinate
            bag_marker.pose.orientation.w = 1
            bag_marker.lifetime.secs = 0
            bag_marker.lifetime.nsecs = 0
            bag_marker.type = 1
            bag_marker.scale.x = 0.5
            bag_marker.scale.y = 0.5
            bag_marker.scale.z = 0.5
            bag_marker.color.a = 1.0
            bag_marker.color.r = 0
            bag_marker.color.g = 1
            bag_marker.color.b = 0
            self.context.maker_pub.publish(bag_marker)
            
            #self.context.client.wait_for_server()follow_stop_pub
            
            # # Waits for the server to finish performing the action.
            # self.context.client.wait_for_result()
            
            # result = self.context.client.get_result()
            # rospy.loginfo(f"result from manipulation {result}")
            
            # self.context.bag_pub.publish(self.handle_stamped)
            rospy.sleep(10)
            self.context.bag_pub.publish(self.handle_stamped)
            rospy.sleep(2)
            self.context.transition_to(bag_pickup())
            
    
            
class bag_pickup(State):
    def run(self):
        if (self.context.pickup_results == "Success"):
            self.context.say("I've got the bag, I'm ready to follow you to the car")
            self.context.follow_pub.publish(String("idsub"))
            self.context.transition_to(following())
        else:
            pass

class following(State):
    def run(self):
        rospy.loginfo(f"I am still following, the pose change is {self.context.pose_diffs}")
        if (max(self.context.pose_diffs) < 0.0001 ):
            self.context.say("Are we there yet")
            if self.context.vosk_msg != None:
                for word in self.context.vosk_msg.split():
                    if (word in self.goal_reached_words in ["Yes", "yes"]):
                        self.say("Thanks for taking me to the car, here is your bag.")
                        self.car_reached = True
        tstart = datetime.datetime.now().timestamp()
        while(datetime.datetime.now().timestamp() - tstart < 3):
            self.goal_reached_words = ['give', 'reached', 'arrived', 'reach']
            for word in self.context.vosk_msg.split():
                if (word in self.goal_reached_words):
                    self.context.say("Thanks for taking me to the car, here is your bag.")
                    self.car_reached = True
                    self.context.follow_stop_pub.publish(String("stop"))
                    self.context.open_grasp_pub.publish(String("open"))
                    
                    rospy.sleep(200)
                    self.context.transition_to(bag_release())
                    
                    
                    
            
        

class bag_release(State):
    def run(self):
        self.context.release_pub.publish("Start")
        if (self.context.release_results == "Success"):
            rospy.sleep(2)
            self.context.transition_to(return_to_start())

class return_to_start(State):
    def run(self):
        dest = PoseStamped()
        dest.pose.position.x = self.context.inspection_point['loc_x']
        dest.pose.position.y = self.context.inspection_point['loc_y']
        dest.pose.position.z = self.context.inspection_point['loc_z']
        dest.pose.orientation.w = 1
        dest.header.frame_id = "map"
        self.context.move_base_pub.publish(dest)
        self.say("I am going to return to the house now")
        self.context.returned = True
        self.context.transition_to(idle())

class idle(State):
    def run(self):
        pass

# original 
if __name__ == "__main__":
    rospy.init_node('carry_my_luggage_controller')
    carry_my_luggage_controller_node = Context(start())
    while not rospy.is_shutdown():
        carry_my_luggage_controller_node.run()
