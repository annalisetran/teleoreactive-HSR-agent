#!/usr/bin/env python
import rospy
import sys
import math
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped, PointStamped, Pose
from actionlib_msgs.msg import GoalID
from unsw_vision_msgs.msg import DetectionList
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
import tf.transformations
import tf2_ros
import tf2_geometry_msgs
import hsrb_interface
from hsrb_interface import geometry
from sensor_msgs.msg import JointState

from visualization_msgs.msg import Marker

class PersonFollow:

  def __init__(self):
    print("Initializing PersonFollow node")
    self._robot = hsrb_interface.Robot()
    self._whole_body = self._robot.get('whole_body')
    self.tfBuffer = tf2_ros.Buffer()
    self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
    self.id = None
    # if you want to auto start do this # self.id="0"
    # self.id="0"

    self.tts = rospy.Publisher("tts/phrase", String, queue_size=1)
    self.person_last_seen_point = None
    self.head_pan_angle = 0.0
    self.vision_temp_blind = False
    self.tracking_person = ""
    self.operater_height = 0

    self.search_triggered_time = rospy.get_time()
    self.search_triggered = False
    self.operater_first_registered = False
    self.tracking_person_tracking_id = -1
    self.operate_frame_lost = 0

    self.nav_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    self.pose_pub = rospy.Publisher('/debug_point', PoseStamped, queue_size=10)
    self.tracked_person_pub = rospy.Publisher('/tracked_person', PointStamped, queue_size=10)
    self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

    self.timer = rospy.Timer(rospy.Duration(3), self.on_timer)
    self.head_timer = rospy.Timer(rospy.Duration(0.08), self.head_on_timer)
    self.vision_sub = rospy.Subscriber("/unsw_vision/detections/objects/positions", DetectionList, self.callback)
    
    self.clicked_point_sub = rospy.Subscriber("/clicked_point", PointStamped, self.clicked_point_callback)
    self.start_sub = rospy.Subscriber("/navigation/person_following/id", String, self.id_callback)
    self.head_pan_angle_sub = rospy.Subscriber("/hsrb/joint_states", JointState, self.head_pan_angle_callback)
    self.stopSub = rospy.Subscriber("/navigation/person_following/stop", String, self.stop_callback)
    
    self.stopSub = rospy.Subscriber
    
    
    rospy.loginfo("PersonFollow node initialized")

  
  def head_pan_angle_callback(self, data):
    self.head_pan_angle = data.position[data.name.index('head_pan_joint')]

  def id_callback(self, data: String):
    self.id = data.data
  
  def stop_callback(self):
    self.id = None
  # For testing my math
  def clicked_point_callback(self, data: PointStamped):
    point = data.point
    self.person_last_seen_point = point
    

  def stop_callback(self, data: String):
    self.id = None
    self.person_last_seen_point = None

  def callback(self, data: DetectionList):
    if self.id is None:
      return
    if self.vision_temp_blind:
      rospy.loginfo("Vision is temporarily blind due to a full rotation, skipping person detection")
      return

    person_found = False



    if self.tracking_person != "":
      for human in data.people:
          if human.id == self.tracking_person:
            person_point = human.position
            person_found = True
            self.search_triggered = False
            rospy.loginfo("Found person id based")
            break
    
    if not person_found:
      if self.tracking_person_tracking_id != -1:
        for human in data.people:
            if human.tracking_id == self.tracking_person_tracking_id:
              person_point = human.position
              person_found = True
              self.search_triggered = False
              self.tracking_person = human.id
              self.tracking_person_tracking_id = human.tracking_id
              rospy.loginfo("Found person tracking_id based")
              break

    # Clostest person that match condition
    dis = 1000
    if not person_found:
      # filtering base on height
      for human in data.people:
          if human.distance < dis and self.operater_height != 0 and self.operater_height + 0.15 > human.skeleton[10].z and self.operater_height - 0.15 < human.skeleton[10].z:
            dis = human.distance
            person_point = human.position
            person_found = True
            self.tracking_person = human.id
            self.tracking_person_tracking_id = human.tracking_id
            self.operater_height = human.skeleton[10].z
            rospy.loginfo("Found person based on conditions")
            self.search_triggered = False


    # BUGGY BLOCK
    # if self.operater_first_registered and not person_found:
    #   if self.search_triggered == False:
    #     # check if we need to search
    #     self.search_triggered = True
    #     self.search_triggered_time = rospy.get_time()
    #     rospy.loginfo("say I lost you")
    #     #self.tts.publish("I lost you please come close to me")
    #   else:
    #     # check if we need to exit
    #     time_since_search_start = rospy.get_time() - self.search_triggered_time
    #     if time_since_search_start > 3:
    #       # exit
    #       rospy.loginfo(f"time_since_search_start {time_since_search_start}")
    #       self.search_triggered = False
    #     else:
    #       # dont use closest person
    #       return
    
    
    # if not person_found:
    #   self.operate_frame_lost += 1
    # else:
    #   # reset
    #   self.operate_frame_lost = 0
      
    # if self.operater_first_registered and self.operate_frame_lost > 32:
    #   if self.operate_frame_lost == 33:
    #     self.tts.publish("I am losing you please come back")
    #   return
    # elif self.operater_first_registered and self.operate_frame_lost < 60:
    #   return
    
    
    use_closest_fallback = False
    # Closest person with no condition
    dis = 1000
    if not person_found:
      # filtering base on height
      for human in data.people:
          if human.distance < dis:
            use_closest_fallback = True
            dis = human.distance
            person_point = human.position
            person_found = True
            self.tracking_person = human.id
            self.tracking_person_tracking_id = human.tracking_id
            self.operater_height = human.skeleton[10].z
            rospy.loginfo("Found person fall back to nothing detected")
    if not person_found : 
      return
    
    if person_point.x == 0 and person_point.y == 0:
      rospy.loginfo("person point is not usable")
      return
    
    if use_closest_fallback == True:
      # say something
      self.tts.publish("I detected a person for me to follow")

    # we know who we tracking
    self.operater_first_registered = True
    self.person_last_seen_point = person_point
    self.tracked_person_pub.publish(PointStamped(header=data.header, point=person_point))
    
  def on_timer(self, event):
    if self.id is None:
      return

    if self.person_last_seen_point is None:
      rospy.loginfo("No person point available")
      return

    robotPoseStamped = PoseStamped()
    robotPoseStamped.pose = Pose()

    robotPoseStamped.pose.orientation.w = 1
    robotPoseStamped.header.stamp = rospy.Time.now()
    robotPoseStamped.header.frame_id = 'base_footprint'
    # transform to get the oritation of the robot pose the map frame
    try:
        trans = self.tfBuffer.lookup_transform("map","base_footprint",rospy.Time.now(), rospy.Duration(2))
        map_robot_point_stamped= tf2_geometry_msgs.do_transform_pose(robotPoseStamped, trans)
    except Exception as e:
        rospy.loginfo("Failed transform point to footprint. {}".format(e))
        return

    # vector from robot to person
    vector = Point()
    vector.x = self.person_last_seen_point.x - map_robot_point_stamped.pose.position.x
    vector.y = self.person_last_seen_point.y - map_robot_point_stamped.pose.position.y
    vector_length = math.sqrt(vector.x**2 + vector.y**2)
    # do nothing if already close
    if vector_length < 0.7:
      return
    approach_goal = PoseStamped()
    approach_goal.pose = Pose()
    approach_goal.pose.position.x = self.person_last_seen_point.x - vector.x/vector_length * 0.6
    approach_goal.pose.position.y = self.person_last_seen_point.y - vector.y/vector_length * 0.6
    # direction should be the same as the vector
    yaw = math.atan2(vector.y, vector.x)
    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    approach_goal.pose.orientation.x = quaternion[0]
    approach_goal.pose.orientation.y = quaternion[1]
    approach_goal.pose.orientation.z = quaternion[2]
    approach_goal.pose.orientation.w = quaternion[3]
    approach_goal.header.stamp = rospy.Time.now()
    approach_goal.header.frame_id = "map"
    self.pose_pub.publish(approach_goal)
    # move the robot to the approach goal
    self.nav_pub.publish(approach_goal)


  def head_on_timer(self, event):
    if self.id is None:
      return

    if self.person_last_seen_point is None:
      rospy.loginfo("No person point available for head")
      return
    person_point_steamped_map = PointStamped()
    person_point_steamped_map.point = self.person_last_seen_point
    person_point_steamped_map.header.stamp = rospy.Time.now()
    person_point_steamped_map.header.frame_id = "map"
    # get the position of the person in the base_footprint frame
    try:
        trans = self.tfBuffer.lookup_transform("base_footprint","map",rospy.Time.now(), rospy.Duration(2))
        person_point_steamped_footprint = tf2_geometry_msgs.do_transform_point(person_point_steamped_map, trans)
    except Exception as e:
        rospy.loginfo("Failed transform point to footprint. {}".format(e))
        return

    person_point_steamped_footprint.point.z = 1.1  # Set a height for the gaze point to be fixed
    p = geometry.Vector3(person_point_steamped_footprint.point.x, person_point_steamped_footprint.point.y, person_point_steamped_footprint.point.z)
    

    # to handle the case it trying to go over the limit, which wont happen, therefore it will turn using the further direction
    # which is handled by gaze_point
    # but we want to make sure dont retrack the person before the head is done moving
    angle = math.atan2(p.y, p.x)
    if angle > 1.73:
      angle -= math.pi * 2

    if abs(angle- self.head_pan_angle) > 3.14:
      rospy.loginfo("Head pan angle is too high, temporarily blind vision")
      self.vision_temp_blind = True
      

    self._whole_body.gaze_point(p, "base_footprint")
    self.vision_temp_blind = False


    

  def id_callback(self, data: String):
    self.id = data.data
    rospy.loginfo(f"I receive id for the person {self.id}")

def main(args):
  rospy.init_node('point_follow', anonymous=True)
  obc = PersonFollow()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)