#!/usr/bin/env python3
import rospy
import math
from datetime import datetime
import tf.transformations
import tf2_ros
import tf

# msg imports 
from nav_msgs.msg import Odometry
import tf2_geometry_msgs
from sensor_msgs.msg import JointState
from tmc_msgs.msg import BatteryState
from tmc_control_msgs.msg import ServoState
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Int32

# WM imports 
import os
import sys
import json
from dbase import DBase
from robot_state_api import RobotStateApi
from regions_api import RegionsApi
module_path = os.environ.get("UNSW_WS")
sys.path.append(module_path + "/WORLD_MODEL/world_model/world_model_kb/scripts/" + "database.ini")

class RobotState:
    def __init__(self):
        self.frame_id = 1

        self.base_pose_x = 'null'
        self.base_pose_y = 'null'
        self.base_pose_z = 'null'
        self.region_id = 'null'
        self.base_r = 'null'
        self.base_p = 'null'
        self.base_yw = 'null'

        self.head_pose_z = 'null'
        self.head_r = 'null'
        self.head_p = 'null'
        self.head_yw = 'null'

        # end_effector pose in map frame
        self.end_effector_x = 'null'
        self.end_effector_y = 'null'
        self.end_effector_z = 'null'
        self.end_effector_r = 'null'
        self.end_effector_p = 'null'
        self.end_effector_yw = 'null'

        self.driveable_state = 'null'
        self.is_wrist_pressed = 'null'
        self.gripper_closed = 'null' # True for closed, False for open
        self.listening_state = 'null'
        self.speaking_state = 'null'
        self.battery_status = 'null' # battery status ranges from 0-100 representing a percentage charge. 

        self.holding_object_id = 'null'
        
class RobotStateUpdater:
    def __init__(self, tfBuffer:tf2_ros.Buffer):
        # initialise RobotState class
        self.robot = RobotState()

        # set tf_buffer
        self.tf_buffer = tfBuffer

        # initialise database connection
        self.robot_state_api = RobotStateApi()
        self.regions_api = RegionsApi()
        module_path = os.environ.get("UNSW_WS")
        self.db = DBase(module_path + "/WORLD_MODEL/world_model/world_model_kb/scripts/database.ini")  # database connections

        # Initialise subscribers
        odom_sub = rospy.Subscriber("/hsrb/odom", Odometry, self.odom_callback, queue_size=1)
        battery_sub = rospy.Subscriber("/hsrb/battery_state", BatteryState, self.battery_callback, queue_size=1)
        joint_sub = rospy.Subscriber("/hsrb/joint_states", JointState, self.joint_callback, queue_size=1)
        wrist_wrench_sub = rospy.Subscriber("/hsrb/wrist_wrench/raw",WrenchStamped, self.wrist_wrench_callback, queue_size=1)
        holding_sub = rospy.Subscriber("/holding_object_id", Int32, self.holding_object_callback, queue_size=1)
        
        self.wrist_press_updated = 0
        self.initialize_holding_object_id()

        self.robot.holding_object_id = 'null'

        print("initialising")

    # Callbacks
    def holding_object_callback(self, msg):
        """Update holding_object_id when commanded"""
        if msg.data < 0:
            self.robot.holding_object_id = 'null'
            rospy.loginfo("Set holding_object_id to null")
        else:
            self.robot.holding_object_id = msg.data
            rospy.loginfo(f"Set holding_object_id to {msg.data}")
    
    def odom_callback(self, msg:Odometry):
        rate = rospy.Rate(10.0)

        # Construct pose stamped in odom frame for odometry message which corresponds to robots location
        odom_pose_stamped = tf2_geometry_msgs.PoseStamped()
        odom_pose_stamped.header.frame_id = msg.header.frame_id
        odom_pose_stamped.header.stamp = rospy.Time.now()
        odom_pose_stamped.pose = msg.pose.pose

        try:
            # transform from odom to map frame
            transform = self.tf_buffer.lookup_transform("map", "odom",rospy.Time(0), rospy.Duration(1.0))
            map_pose_stamped = tf2_geometry_msgs.do_transform_pose(odom_pose_stamped, transform)

        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        self.robot.base_pose_x = map_pose_stamped.pose.position.x
        self.robot.base_pose_y = map_pose_stamped.pose.position.y
        self.robot.base_pose_z = map_pose_stamped.pose.position.z

        (self.robot.base_r, self.robot.base_p, self.robot.base_yw) = tf.transformations.euler_from_quaternion([map_pose_stamped.pose.orientation.x, map_pose_stamped.pose.orientation.y, map_pose_stamped.pose.orientation.z, map_pose_stamped.pose.orientation.w])

        # determine region_id based on base_pose_x and base_pose_y
        res = self.regions_api.find_point_label(self.db.con_pool, self.robot.base_pose_x, self.robot.base_pose_y)
        if res is not None and len(res) > 0 and res[0][0] is not None:
            self.robot.region_id = res[0][0]['region_id']
        else:
            self.robot.region_id = 'null'

    
    def battery_callback(self, msg:BatteryState):
        self.robot.battery_status = msg.power

    def joint_callback(self, msg:JointState):
        # gripper is hand_motor joint, open position is ~1.24, closed position is -0.88
        gripper_index = msg.name.index("hand_motor_joint")
        if (msg.position[gripper_index] < 1.2):
            self.robot.gripper_closed = True
        else:
            self.robot.gripper_closed = False

        # arm_flex_joint -0.84
        arm_lift_index = msg.name.index("arm_lift_joint")
        arm_flex_index = msg.name.index("arm_flex_joint")
        if (msg.position[arm_lift_index] > 0.3 or msg.position[arm_flex_index] < -0.84):
            self.robot.driveable_state = False
        else:
            self.robot.driveable_state = True

    def wrist_wrench_callback(self, msg:WrenchStamped):
        if msg.wrench.force.x > 40:
            self.robot.is_wrist_pressed = True
            self.wrist_press_updated = msg.header.stamp.secs
        elif (msg.header.stamp.secs - self.wrist_press_updated) > 5 and msg.wrench.force.x < 12:
            self.robot.is_wrist_pressed = False
    

    def initialize_holding_object_id(self):
        """Initialize holding_object_id from current database state"""
        try:
            current = self.robot_state_api.get_robot_state_current(self.db.con_pool)
            if current and current[0] and current[0][0]:
                data = current[0][0][0]
                holding_val = data.get('holding_object_id', 'null')
                # Handle None or 'null' from database
                if holding_val is None or holding_val == 'null':
                    self.robot.holding_object_id = 'null'
                else:
                    self.robot.holding_object_id = holding_val
                rospy.loginfo(f"Initialized holding_object_id to {self.robot.holding_object_id}")
            else:
                self.robot.holding_object_id = 'null'
        except Exception as e:
            rospy.logerr(f"Failed to initialize holding_object_id: {e}")
            self.robot.holding_object_id = 'null'
    def head_updater(self):
        try:
            head_transform = self.tf_buffer.lookup_transform("map", "head_center_camera_frame", rospy.Time(0), rospy.Duration(1.0))
            head_pos = head_transform.transform.translation
            head_rot = head_transform.transform.rotation

            self.robot.head_pose_z = head_pos.z
            [self.robot.head_r, self.robot.head_p, self.robot.head_yw] = tf.transformations.euler_from_quaternion([head_rot.x, head_rot.y, head_rot.z, head_rot.w])
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr(f"Could not find map->head transformation")
            raise
    
    def end_effector_updater(self):
        try:
            # TODO could change to hand_l_finger_tip_frame or hand_r_finger_tip_frame, etc
            end_effector_transform = self.tf_buffer.lookup_transform("map", "hand_palm_link", rospy.Time(0), rospy.Duration(1.0))
            end_effector_pos = end_effector_transform.transform.translation
            end_effector_rot = end_effector_transform.transform.rotation

            self.robot.end_effector_x = end_effector_pos.x
            self.robot.end_effector_y = end_effector_pos.y
            self.robot.end_effector_z = end_effector_pos.z
            [self.robot.end_effector_r, self.robot.end_effector_p, self.robot.end_effector_yw] = tf.transformations.euler_from_quaternion([end_effector_rot.x, end_effector_rot.y, end_effector_rot.z, end_effector_rot.w])
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr(f"Could not find map->hand transformation")
            raise

    def update_wm(self):
        self.head_updater()
        self.end_effector_updater()
        
        # Helper to convert None to 'null' string
        def safe_value(val):
            return 'null' if val is None else val
        
        robot_state_dict = {
            "frame_id" : self.robot.frame_id,

            "base_pose_x" : safe_value(self.robot.base_pose_x),
            "base_pose_y" : safe_value(self.robot.base_pose_y), 
            "base_pose_z" : safe_value(self.robot.base_pose_z), 
            "region_id" : safe_value(self.robot.region_id),
            "base_r" : safe_value(self.robot.base_r), 
            "base_p" : safe_value(self.robot.base_p), 
            "base_yw" : safe_value(self.robot.base_yw),

            "head_pose_z" : safe_value(self.robot.head_pose_z),
            "head_r" : safe_value(self.robot.head_r),
            "head_p" : safe_value(self.robot.head_p),
            "head_yw" : safe_value(self.robot.head_yw),

            "end_effector_x" : safe_value(self.robot.end_effector_x),
            "end_effector_y" : safe_value(self.robot.end_effector_y),
            "end_effector_z" : safe_value(self.robot.end_effector_z),
            "end_effector_r" : safe_value(self.robot.end_effector_r),
            "end_effector_p" : safe_value(self.robot.end_effector_p),
            "end_effector_yw" : safe_value(self.robot.end_effector_yw),

            "driveable_state" : safe_value(self.robot.driveable_state), 
            "is_wrist_pressed" : safe_value(self.robot.is_wrist_pressed),
            "gripper_closed" : safe_value(self.robot.gripper_closed),
            "listening_state" : safe_value(self.robot.listening_state),
            "speaking_state" : safe_value(self.robot.speaking_state),
            "battery_status" : safe_value(self.robot.battery_status),

            "holding_object_id" : safe_value(self.robot.holding_object_id),
        }   

        self.robot_state_api.insert_robot_state(self.db.con_pool, json.dumps(robot_state_dict))
        #rospy.loginfo("Robot state updated")

def main():
    # initialise node
    rospy.init_node("robot_state_updater")
    
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    robot_state_updater = RobotStateUpdater(tfBuffer)
    while not rospy.is_shutdown():
        robot_state_updater.update_wm()
        rospy.sleep(0.2)

if __name__ == '__main__':
    main()


