#! /usr/bin/env python3

import rospy
import actionlib
import hsrb_interface
from hsrb_interface import geometry
import tf2_geometry_msgs
from action_server.msg import ManipPlacingAction, ManipPlacingFeedback, ManipPlacingResult
from math import pi
import tf2_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import WrenchStamped
PALM_SIZE = 0.06
class ManipPlacing(object):
    def __init__(self, name):
        # init the robot
        self._robot = hsrb_interface.Robot()
        self._whole_body = self._robot.get('whole_body')
        self._gripper = self._robot.get('gripper')
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.wrist_pressure_sub = rospy.Subscriber("hsrb/wrist_wrench/raw", WrenchStamped, self.wristCB)
        self.move_base_action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        if not self.move_base_action_client.wait_for_server(rospy.Duration(10)):
            rospy.logerr("move_base action server is not available")
            rospy.signal_shutdown("move_base action server is not available")
            return
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, ManipPlacingAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo("Action server manip-placing started")

    def wristCB(self, data):
      self.wrist_pressure = data.wrench.force.x
      # pressure on the wrist when grip opens vertically
      self.grip_pressure = data.wrench.force.y

    # open the gripper, can be extracted to a helper function of robot class
    def open_gripper(self):
        rospy.loginfo("opening gripper")
        self._gripper.command(1.2)
    # close the gripper, can be extracted to a helper function of robot class
    def close_gripper(self):
        self._gripper.apply_force(0.7)


    def execute_cb(self, goal):
        rospy.loginfo("executing placing")
        if goal.direction == "table":
            self.table_placing(goal)
        else:
            self.normal_placing(goal)

    def table_placing(self, goal):
        feedback = ManipPlacingFeedback()
        result = ManipPlacingResult()
        target_point = goal.target_point
        buffer = 0.05
        self._whole_body.move_to_neutral()
        rospy.sleep(1)

        # rotate to obj
        try:
            trans = self.tfBuffer.lookup_transform("base_footprint", target_point.header.frame_id, rospy.Time.now(), rospy.Duration(2))
            transformed_point_stamped = tf2_geometry_msgs.do_transform_point(target_point, trans)
            angle_to_target = math.atan2(transformed_point_stamped.point.y, transformed_point_stamped.point.x)
            # print(angle_to_target)
            # rotate the robot to face the target and then another 90 degree make the right side of the robot facing the target
            # calc the new posestamp in base_footprint frame(0 + angle_to_target)
            goal_pose_rotation = tf2_geometry_msgs.PoseStamped()
            goal_pose_rotation.header.frame_id = "base_footprint"
            # calc the quanternion from ei ej ek use euler to quaternion
            q = quaternion_from_euler(0,0,angle_to_target)
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
        except Exception as e:
            rospy.loginfo("Failed to rotate to the object. {}".format(e))
            feedback.feedback = 2  # Failure
            self._as.publish_feedback(feedback)
            result.action_success = False
            #result.forward_distance = 0
            self._as.set_aborted(result)
            return
        
        rospy.sleep(2)

        # update the wrist pressure value for current hand(should hold item). Can be updated to take averages
        try:
            trans = self.tfBuffer.lookup_transform("hand_palm_link",target_point.header.frame_id,rospy.Time.now(), rospy.Duration(2))
            transformed_point_stamped= tf2_geometry_msgs.do_transform_point(target_point, trans)
            rospy.loginfo(transformed_point_stamped)
        except Exception as e:
            rospy.loginfo("Failed transform point to footprint. {}".format(e))
            result.action_success = False
            feedback.feedback = 2  # Failure
            self._as.publish_feedback(feedback)
            self._as.set_aborted(result)
            return
        
        try:
            
            self._whole_body.end_effector_frame = u'hand_palm_link'
            # move to match height, buffer up a little bit to avoid distance. Makesure the buffer is less than level height if in shelf. 
            self._whole_body.move_end_effector_by_line((1,0,0),transformed_point_stamped.point.x+ buffer)

            # move to match horizontally
            self._whole_body.move_end_effector_by_line((0,1,0),transformed_point_stamped.point.y)

            # move to match depth
            self._whole_body.move_end_effector_by_line((0,0,1),transformed_point_stamped.point.z)

            # move down by buffer
            self._whole_body.move_end_effector_by_line((1,0,0), - buffer)
            self.open_gripper()
            
            # move up 
            self._whole_body.move_end_effector_by_line((1,0,0), buffer)
            rospy.sleep(0.5)
            self._whole_body.move_end_effector_by_line((0,0,-1),transformed_point_stamped.point.z)
            
            rospy.sleep(1)
            # move back to original pose
            self._whole_body.move_to_neutral()
            rospy.sleep(1)
            
            result.action_success = True
            self._as.set_succeeded(result)
            feedback.feedback=0
            self._as.publish_feedback(feedback)
            return
            
        except Exception as e:
            rospy.logwarn("Failed moving body. {}".format(e))
            result.action_success = False
            feedback.feedback = 2  # Failure
            self._as.publish_feedback(feedback)
            self._as.set_aborted(result)
            return

    def normal_placing(self, goal):
        feedback = ManipPlacingFeedback()
        result = ManipPlacingResult()
        # default to be true
        result.action_success=True
        target_point = goal.target_point
        direction = goal.direction
        self._whole_body.move_to_neutral()
        # update the wrist pressure value for current hand(should hold item). Can be updated to take averages
        # NOTE: temp solution since sensor is not re
        # holding_wrist_pressure = self.wrist_pressure   
        try:
            trans = self.tfBuffer.lookup_transform("base_footprint",target_point.header.frame_id,rospy.Time.now(), rospy.Duration(2))
            transformed_point_stamped= tf2_geometry_msgs.do_transform_point(target_point, trans)
            rospy.loginfo(transformed_point_stamped)
        except Exception as e:
            rospy.loginfo("Failed transform point to footprint. {}".format(e))
            result.action_success = False
            feedback.feedback = 2  # Failure
            self._as.publish_feedback(feedback)
            self._as.set_aborted(result)
            return

        
        if direction == "top":
            pose = geometry.pose(x=transformed_point_stamped.point.x,
                            y=transformed_point_stamped.point.y,
                            z=min(max(float(transformed_point_stamped.point.z+ 0.08),0.02),1.00),
                            ei=-pi,
                            ej=0,
                            ek=-pi/2)
            line_dis = pose.pos.z - transformed_point_stamped.point.z - PALM_SIZE
            line_traj = (0,0,1)
        elif direction == "front":
            pose = geometry.pose(x=transformed_point_stamped.point.x,
                                y=transformed_point_stamped.point.y,
                                z=min(max(float(transformed_point_stamped.point.z+ 0.1),0.02),1.00),
                                ei=-pi,
                                ej=-pi/2,
                                ek=0)
            line_traj = (-1,0,0)
            line_dis = pose.pos.z - transformed_point_stamped.point.z
        else:
            result.action_success = False
            feedback.feedback = 2  # Failure
            self._as.publish_feedback(feedback)
            self._as.set_aborted(result)
            return
        try:
            # move to that pose
            self._whole_body.end_effector_frame = u'hand_palm_link'
            a = self._whole_body.move_end_effector_pose(pose,
                                                    "base_footprint")
            self._whole_body.move_end_effector_by_line(line_traj,line_dis)
            self.open_gripper()
            # if drop from top do twist
            if direction == "top":
                rospy.loginfo("Got into wrist_rotate function")
                wrist_roll_joint = self._whole_body.joint_positions.get('wrist_roll_joint')
                original_joint_Values = jointValues = {
                        'wrist_roll_joint': wrist_roll_joint
                    }
                if wrist_roll_joint > 0:
                    jointValues = {
                        'wrist_roll_joint': wrist_roll_joint - pi/9,
                    }
                else:
                    jointValues = {
                        'wrist_roll_joint': wrist_roll_joint + pi/9,
                    }
                try:
                    self._whole_body.move_to_joint_positions(jointValues)
                    self._whole_body.move_to_joint_positions(original_joint_Values)
                except:
                    result.action_success = False
                    feedback.feedback = 2  # Failure
                    self._as.publish_feedback(feedback)
                    self._as.set_aborted(result)
                    return
            rospy.sleep(1)
            # move back to original pose
            self._whole_body.move_to_neutral()
            rospy.sleep(1)

            # drop success if pressure difference found or does not have bag(pressure close to initial pressure that does not holding bag)
            # NOTE: temp solution since sensor is not reliable
            # if holding_wrist_pressure - self.wrist_pressure >= 2 or abs(holding_wrist_pressure-self.initial_wrist_pressure) <= 1:
            if True:
                self._as.set_succeeded(result)
                feedback.feedback=0
                self._as.publish_feedback(feedback)
                return 
            result.action_success = False
            feedback.feedback = 2  # Failure
            self._as.publish_feedback(feedback)
            self._as.set_aborted(result)
            return
            
        except Exception as e:
            rospy.logwarn("Failed moving body. {}".format(e))
            result.action_success = False
            feedback.feedback = 2  # Failure
            self._as.publish_feedback(feedback)
            self._as.set_aborted(result)
            return

        


        
        


if __name__ == '__main__':
    rospy.init_node('manip_placing')
    server = ManipPlacing(rospy.get_name())
    rospy.spin()
    