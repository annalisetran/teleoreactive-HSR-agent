#! /usr/bin/env python3

import rospy
import actionlib
import hsrb_interface
from hsrb_interface import geometry
import tf2_geometry_msgs
from action_server.msg import ManipGazeAtAction, ManipGazeAtFeedback, ManipGazeAtResult, ManipViewPoseAction, ManipViewPoseFeedback, ManipViewPoseResult
from math import pi
import tf2_ros
import json
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from tf.transformations import quaternion_from_euler
PALM_SIZE = 0.06
class ManipGaze(object):
    def __init__(self):
        # init the robot
        action_name1 = "manip_view_pose"
        action_name2 = "manip_gaze_at"
        action_name3 = "manip_gaze_region"
        self._robot = hsrb_interface.Robot()
        self._whole_body = self._robot.get('whole_body')
        self._gripper = self._robot.get('gripper')
        rospy.loginfo("Robot initialized")
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.loginfo("TF listener initialized")

        self.move_base_action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        if not self.move_base_action_client.wait_for_server(rospy.Duration(10)):
            rospy.logerr("move_base action server is not available")
            rospy.signal_shutdown("move_base action server is not available")
            return
        self._as1 = actionlib.SimpleActionServer(action_name1, ManipViewPoseAction, execute_cb=self.view_pose_execute_cb, auto_start = False)
        self._as1.start()
        rospy.loginfo("Action server view_pose started")

        self._as2 = actionlib.SimpleActionServer(action_name2, ManipGazeAtAction, execute_cb=self.gaze_at_cb, auto_start = False)
        self._as2.start()
        rospy.loginfo("Action server gaze_at started")
        # self._as3 = actionlib.SimpleActionServer(action_name3, ManipViewPoseAction, execute_cb=self.execute_cb, auto_start = False)
        # self._as3.start()

    def joint_status_update_cb(self,x):
        feedback = ManipGazeAtFeedback()
        x = self._whole_body.joint_positions
        feedback.feedback = json.dumps(x)
        self._as2.publish_feedback(feedback)

    def view_pose_execute_cb(self, goal):
        feedback = ManipViewPoseFeedback()
        result = ManipViewPoseResult()
        # default to be true
        result.action_success=True
        feedback.feedback = 1  # Ongoing
        self._as1.publish_feedback(feedback)

        # TODO: these values need to be revised
        arm_lift_joint_var_MAX =100
        arm_lift_joint_var_MIN =-100
        head_tilt_joint_var_MAX = 100
        head_tilt_joint_var_MIN = -100

        arm_lift_joint_var = min (max(goal.arm_lift_joint, arm_lift_joint_var_MIN), arm_lift_joint_var_MAX)
        head_tilt_joint_var = min (max(goal.head_tilt_joint, head_tilt_joint_var_MIN), head_tilt_joint_var_MAX)
        
        try:
            jointValues = {
                "arm_flex_joint": goal.arm_flex_joint,
                "arm_lift_joint": arm_lift_joint_var,
                "arm_roll_joint": -1.63,
                "hand_motor_joint": 0.69,
                "head_pan_joint": goal.head_pan_joint,
                "head_tilt_joint": head_tilt_joint_var,
                "wrist_flex_joint": -1.57,
                "wrist_roll_joint": 1.91
            }
            self._whole_body.move_to_joint_positions(jointValues)
            self._as1.set_succeeded(result)
            feedback.feedback=0
            self._as1.publish_feedback(feedback)
            return
        except Exception as e:
            rospy.loginfo(e)
            result.action_success = False
            feedback.feedback = 2  # Failure
            self._as1.publish_feedback(feedback)
            self._as1.set_aborted(result)
            return


    #TODO: TEST IF THIS MOVES BASE, as it will affect 
    def gaze_at_cb(self, goal):
        result = ManipGazeAtResult()
        # default to be true
        result.action_success=True
        target_point = goal.target_point
        x = rospy.Timer(rospy.Duration(secs=0.2), self.joint_status_update_cb)
        
        try:
            # calc the oretation need to be done so robot front is facing gaze point
            trans = self.tfBuffer.lookup_transform("base_footprint", target_point.header.frame_id, rospy.Time.now(), rospy.Duration(2))
            transformed_point_stamped = tf2_geometry_msgs.do_transform_point(target_point, trans)
            # print(transformed_point_stamped.point)
            angle_to_target = math.atan2(transformed_point_stamped.point.y, transformed_point_stamped.point.x)
            # print(angle_to_target)

            # rotate the robot to face the target and then another 90 degree make the right side of the robot facing the target
            # calc the new posestamp in base_footprint frame(0 + angle_to_target)
            goal_pose_rotation = tf2_geometry_msgs.PoseStamped()
            goal_pose_rotation.header.frame_id = "base_footprint"
            # calc the quanternion from ei ej ek use euler to quaternion
            q = quaternion_from_euler(0,0,angle_to_target+math.pi/2)
            # print(q)
            goal_pose_rotation.pose.orientation.w = q[3]
            goal_pose_rotation.pose.orientation.z = q[2]
            goal_pose_rotation.pose.orientation.y = q[1]
            goal_pose_rotation.pose.orientation.x = q[0]
            trans2 = self.tfBuffer.lookup_transform("map", "base_footprint", rospy.Time.now(), rospy.Duration(2))
            transformed_pose_stamp_map = tf2_geometry_msgs.do_transform_pose(goal_pose_rotation, trans2)
            

            goal = MoveBaseGoal()
            goal.target_pose = transformed_pose_stamp_map
            self.move_base_action_client.send_goal_and_wait(goal)
          
            #p = geometry.Vector3(x = target_point.point.x, y = target_point.point.y, z = target_point.point.z)
            # now the robot's right is facing the target point
            # calc the distance between the robot and the target point, which is the hypotenuse of the triangle
            distance = math.sqrt(transformed_point_stamped.point.x**2 + transformed_point_stamped.point.y**2)
            p = geometry.Vector3(x = 0, y = -distance, z = transformed_point_stamped.point.z)
            self._whole_body.gaze_point(p, "base_footprint")

            rospy.loginfo("Gaze at done")
            result.action_success = True
            self._as2.set_succeeded(result)
            x.shutdown()
            return
        except Exception as e:
            rospy.loginfo(e)
            result.action_success = False
            self._as2.set_aborted(result)
            x.shutdown()
            return
            


        


        
        


if __name__ == '__main__':
    rospy.init_node('manip_gaze')
    server = ManipGaze()
    rospy.spin()
    