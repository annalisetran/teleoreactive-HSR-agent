#! /usr/bin/env python3

import rospy
import actionlib
import hsrb_interface
from hsrb_interface import geometry
import tf2_geometry_msgs
from action_server.msg import ManipApproachAction, ManipApproachFeedback, ManipApproachResult
from sensor_msgs.msg import Image, CameraInfo
from unsw_vision_msgs.msg import DetectionList
from scipy.spatial.transform import Rotation as R
from tf2_geometry_msgs import do_transform_pose
from math import pi
import numpy as np
import tf2_ros

import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply
from geometry_msgs.msg import PoseStamped
import sys
import os
module_path = os.environ.get("UNSW_WS")
sys.path.append(module_path + "/PLANNING/action_server/src")

from grasp_code.pointCloudData import PointCloudData
from grasp_code.superquadric import Superquadric
from grasp_code.grasps import Grasps

PALM_SIZE = 0.06
class ManipApproach(object):
    def __init__(self, name):
        # init the robot
        self._robot = hsrb_interface.Robot()
        self._whole_body = self._robot.get('whole_body')
        self._gripper = self._robot.get('gripper')
        self.debug = True

        self.move_base_action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        if not self.move_base_action_client.wait_for_server(rospy.Duration(10)):
            rospy.logerr("move_base action server is not available")
            rospy.signal_shutdown("move_base action server is not available")
            return
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        #Vision Subscribers - Store latest messages instead of subscribers
        self.latest_rgb = None
        self.latest_depth = None
        self.latest_camera_info = None
        self.latest_detections = None
        self.base_grasp_pose_pub = rospy.Publisher('base_grasp_pose', PoseStamped, queue_size=10)
        self.hand_grasp_pose_pub = rospy.Publisher('hand_grasp_pose', PoseStamped, queue_size=10)
        self.rgb_sub = rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/image_raw', Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/image_raw', Image, self.depth_callback)
        self.camera_info_sub = rospy.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/camera_info', CameraInfo, callback=self.camera_info_callback)
        self.detection_sub = rospy.Subscriber('/unsw_vision/detections/objects/positions', DetectionList, callback=self.detections_callback)

        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, ManipApproachAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo("Action server manip-approach started")

    # Callback functions to store latest messages
    def rgb_callback(self, msg):
        self.latest_rgb = msg
        
    def depth_callback(self, msg):
        self.latest_depth = msg
        
    def camera_info_callback(self, msg):
        self.latest_camera_info = msg
        
    def detections_callback(self, msg):
        self.latest_detections = msg

    # open the gripper, can be extracted to a helper function of robot class
    def open_gripper(self):
        rospy.loginfo("opening gripper")
        self._gripper.command(1.2)
    # close the gripper, can be extracted to a helper function of robot class
    def close_gripper(self):
        self._gripper.apply_force(0.7)

    # move the arm lift joint to allign with the object height to avoid collision effectively
    def arm_joint_init(self, target_height):
        # when joint value is 0, it is 33cm from actual ground.
        joint_zero_rel_base = 0.33
        joint_values = {'arm_lift_joint': target_height - joint_zero_rel_base}
        self._whole_body.move_to_joint_positions(joint_values)

    def execute_cb(self, goal):
        r = rospy.Rate(1)
        feedback = ManipApproachFeedback()
        result = ManipApproachResult()
        # default to be true
        result.action_success=True

        feedback.feedback = 0  # Not started
        self._as.publish_feedback(feedback)

        direction = goal.direction
        target_point = goal.target_point
        goal_tracker_id = goal.object_id
        goal_object_class = goal.class_name
        

        # feedback.feedback = 1  # Ongoing
        # self._as.publish_feedback(feedback)

        # try:
        #     trans = self.tf_buffer.lookup_transform("base_footprint", target_point.header.frame_id, rospy.Time.now(), rospy.Duration(2))
        #     transformed_point_stamped = tf2_geometry_msgs.do_transform_point(target_point, trans)
        #     angle_to_target = math.atan2(transformed_point_stamped.point.y, transformed_point_stamped.point.x)
        #     # print(angle_to_target)
        #     # rotate the robot to face the target and then another 90 degree make the right side of the robot facing the target
        #     # calc the new posestamp in base_footprint frame(0 + angle_to_target)
        #     goal_pose_rotation = tf2_geometry_msgs.PoseStamped()
        #     goal_pose_rotation.header.frame_id = "base_footprint"
        #     # calc the quanternion from ei ej ek use euler to quaternion
        #     q = quaternion_from_euler(0,0,angle_to_target)
        #     # print(q)
        #     goal_pose_rotation.pose.orientation.w = q[3]
        #     goal_pose_rotation.pose.orientation.z = q[2]
        #     goal_pose_rotation.pose.orientation.y = q[1]
        #     goal_pose_rotation.pose.orientation.x = q[0]
        #     trans2 = self.tf_buffer.lookup_transform("map", "base_footprint", rospy.Time.now(), rospy.Duration(5))
        #     transformed_pose_stamp_map = tf2_geometry_msgs.do_transform_pose(goal_pose_rotation, trans2)
            
        #     goal = MoveBaseGoal()
        #     goal.target_pose = transformed_pose_stamp_map
        #     # TODO: Uncommnet
        #     self.move_base_action_client.send_goal_and_wait(goal)
        # except Exception as e:
        #     rospy.loginfo("Failed to rotate to the object. {}".format(e))
        #     feedback.feedback = 2  # Failure
        #     self._as.publish_feedback(feedback)
        #     result.action_success = False
        #     result.forward_distance = 0
        #     self._as.set_aborted(result)
        #     return
        
        # rospy.sleep(2)
        # try:
        #     trans = self.tf_buffer.lookup_transform("base_footprint", target_point.header.frame_id, rospy.Time.now(), rospy.Duration(2))
        #     transformed_point_stamped = tf2_geometry_msgs.do_transform_point(target_point, trans)
        #     rospy.loginfo(transformed_point_stamped)
        # except Exception as e:
        #     rospy.loginfo("Failed to transform point to footprint. {}".format(e))
        #     feedback.feedback = 2  # Failure
        #     self._as.publish_feedback(feedback)
        #     result.action_success = False
        #     result.forward_distance = 0
        #     self._as.set_aborted(result)
        #     return
        
        
        line_dis = 0

        #if only one instance of target object class, use that msg
        #else check the tracking id, trackign id will change if object moves out of frame
        current_object = None
        objects_detected = self.latest_detections.objects if self.latest_detections else []
        objects_in_class = {} #tracker_id : msg
        for obj in objects_detected:
            object_class = obj.object_class.lower()
            object_class = object_class.replace(" ", "_")
            if object_class == goal_object_class:
                objects_in_class[obj.tracking_id] = obj
        if len(objects_in_class) == 1:
            current_object = list(objects_in_class.values())[0]  # Fixed: was getValue[0]
        else:
            current_object = list(objects_in_class.values())[0]
            
        if current_object is None:
            rospy.logwarn("No valid object found for grasping")
            feedback.feedback = 2  # Failure
            self._as.publish_feedback(feedback)
            result.action_success = False
            result.forward_distance = 0
            self._as.set_aborted(result)
            return

                # Generate point cloud from rgb, depth and mask
        pcd = PointCloudData(        
            raw_rgb=self.latest_rgb,
            raw_depth=self.latest_depth,
            mask=current_object.mask,
            camera_info=self.latest_camera_info,
            nearest_neighbor=500,
            distance_thresh=0.005,
            semght_threshold=10,
            debug=self.debug
        )

        cloudSegments = pcd.getCloudSegments()
        superquadrics = []

        #generate superquadric fits for each segment
        for segment in cloudSegments:
            sq = Superquadric(segment, downsample=30, debug=self.debug)
            superquadrics.append(sq)
        
        pose = Grasps(
            superquadrics, 
            self.latest_camera_info.header.frame_id, 
            object_center = pcd.getCenter(),
            orientation= direction, 
            grasp_width= 0.236, 
            debug= self.debug
        ).getSelectedGrasps()

        if pose is None:
            rospy.logwarn("No valid grasp pose found")
            feedback.feedback = 2  # Failure
            self._as.publish_feedback(feedback)
            result.action_success = False
            result.forward_distance = 0
            self._as.set_aborted(result)
            return

        # Convert the grasp pose to geometry.pose format for HSR
        # Note: You may need to adjust this conversion based on your specific pose format
        # pose = geometry.pose(
        #     x=selected_grasp_pose.x,
        #     y=selected_grasp_pose.y,
        #     z=selected_grasp_pose.z,
        #     ei=selected_grasp_pose.ei,
        #     ej=selected_grasp_pose.ej,
        #     ek=selected_grasp_pose.ek
        # )

        # if direction in ["top", "top2"]:
        #     line_dis = pose.pose.position.z - transformed_point_stamped.point.z - PALM_SIZE
        # elif direction in ["front", "front-vertical"]:  # Fixed typo: was "front-vrtical"
        #     line_dis = transformed_point_stamped.point.x - pose.pose.position.x - PALM_SIZE

        # acually move the robot
        try:
            self.open_gripper()

            # Transforms & debug pubs
            base_pose = self.camera_to_base_transform(pose, direction)
            hand_pose = self.camera_to_hand_transform(pose)
            self.base_grasp_pose_pub.publish(base_pose)
            self.hand_grasp_pose_pub.publish(hand_pose)
            
            # Safety neutral
            self._whole_body.move_to_neutral()

            # ---- Move base first ----
            if base_pose is None:
                raise RuntimeError("base_pose is None")
            mb_goal = MoveBaseGoal()
            mb_goal.target_pose = base_pose
            mb_goal.target_pose.header.stamp = rospy.Time.now()
            self.move_base_action_client.send_goal_and_wait(mb_goal)

            # ---- Then move end effector ----
            if hand_pose is None:
                raise RuntimeError("hand_pose is None")
            pos_h = hand_pose.pose.position
            quat_h = hand_pose.pose.orientation
            self._whole_body.move_end_effector_pose(
                ((pos_h.x, pos_h.y, pos_h.z), (quat_h.x, quat_h.y, quat_h.z, quat_h.w)),
                "hand_palm_link"
            )

            # Final result (set ONCE)
            result.action_success = True
            result.forward_distance = 0.05
            self._as.set_succeeded(result)
            return

        except Exception as e:
            rospy.logwarn("Failed in execute_cb: %s", e)
            result.action_success = False
            result.forward_distance = 0.0
            self._as.set_aborted(result)
            return
        
    def camera_to_base_transform(self, grasp_pose, direction):

        target_frame = "base_footprint"
        try:

            #transform the pose frame
            T = self.tf_buffer.lookup_transform(target_frame, grasp_pose.header.frame_id, rospy.Time.now(), rospy.Duration(2))
            pose = tf2_geometry_msgs.do_transform_pose(grasp_pose, T)
            
            #re-orientate the pose to point at the object
            roll, pitch, yaw = euler_from_quaternion([pose.pose.orientation.x,
                                                pose.pose.orientation.y,
                                                pose.pose.orientation.z,
                                                pose.pose.orientation.w])
            
            if direction == 'front' or direction == None:
                r = R.from_euler('xyz', [roll, pitch, yaw])

                # Flipped orientation (180 deg around y-axis)
                R_orig = r.as_matrix()
                R_flip = R_orig @ R.from_euler('y', np.pi).as_matrix()
                quat_flipped = R.from_matrix(R_flip).as_quat()

                # Compute flipped position: move 20 cm backwards along flipped forward (z) axis
                forward_vector = R_flip[:, 0]  # Z axis of rotation matrix
                delta_pos = -0.6 * forward_vector
                new_pos = np.array([pose.pose.position.x , pose.pose.position.y, pose.pose.position.z]) + delta_pos
                x, y, z = new_pos
            elif direction == 'top':
                r = R.from_euler('xyz', [roll, pitch, yaw])

                # Rotate 180° around the Z-axis instead of Y-axis
                R_orig = r.as_matrix()
                R_flip = R_orig @ R.from_euler('z', np.pi/2).as_matrix()
                quat_flipped = R.from_matrix(R_flip).as_quat()

                # Compute new position: move 20 cm backwards along the rotated X-axis (forward direction)
                forward_vector = R_flip[:, 0]   # robot forward axis (X)
                delta_pos = -0.6 * forward_vector
                new_pos = np.array([
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z
                ]) + delta_pos

                x, y, z = new_pos

            base_pose = PoseStamped()
            base_pose.header.frame_id = "base_footprint"
            base_pose.pose.position.x = x
            base_pose.pose.position.y = y
            base_pose.pose.position.z = 0
            base_pose.pose.orientation.x = quat_flipped[0]
            base_pose.pose.orientation.y = quat_flipped[1]
            base_pose.pose.orientation.z = quat_flipped[2]
            base_pose.pose.orientation.w = quat_flipped[3]
                
            return base_pose
        
        except Exception as e:
            rospy.logwarn(f"camera to base transform {e}")



    def camera_to_hand_transform(self, grasp_pose):
        target_frame = "hand_palm_link"
        try:
            # Transform grasp_pose into the target frame without altering orientation
            T = self.tf_buffer.lookup_transform(
                target_frame,
                grasp_pose.header.frame_id,
                rospy.Time(0),                 # latest transform
                rospy.Duration(2)
            )
            hand_pose = do_transform_pose(grasp_pose, T)
            return hand_pose

        except Exception as e:
            rospy.logwarn(f"Camera to hand transform error: {e}")
            return None


if __name__ == '__main__':
    rospy.init_node('manip_approach')
    server = ManipApproach(rospy.get_name())
    rospy.spin()