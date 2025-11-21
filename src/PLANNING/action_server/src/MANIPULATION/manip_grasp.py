#! /usr/bin/env python3

import rospy
import actionlib
import hsrb_interface
from hsrb_interface import geometry
import tf2_geometry_msgs
from action_server.msg import ManipGraspAction, ManipGraspFeedback, ManipGraspResult
from math import pi
import tf2_ros
from std_msgs.msg import Bool

PALM_SIZE = 0.06
class ManipGrasp(object):
    def __init__(self, name):
        # init the robot
        self._robot = hsrb_interface.Robot()
        self._whole_body = self._robot.get('whole_body')
        self._gripper = self._robot.get('gripper')
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, ManipGraspAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo("Action server manip-grasp started")

        self.holding_sub = rospy.Subscriber('/holding_object', Bool, self.holding_callback)
        self.holding_object = False


    # open the gripper, can be extracted to a helper function of robot class
    def open_gripper(self):
        rospy.loginfo("opening gripper")
        self._gripper.command(1.2)
    # close the gripper, can be extracted to a helper function of robot class
    def close_gripper(self):
        self._gripper.apply_force(0.7)

    def holding_callback(self, msg):
        if msg.data:
            self.holding_object = True
        else:
            self.holding_object = False

    def execute_cb(self, goal):
        rospy.loginfo("Executing grasp action")
        r = rospy.Rate(1)
        feedback = ManipGraspFeedback()
        result = ManipGraspResult()
        # default to be true
        result.action_success=True
        self.open_gripper()
        line_traj = (0, 0, 1)
        line_dis =  goal.forward_distance
        try:
            self.open_gripper()
            self._whole_body.end_effector_frame = u'hand_palm_link'

            feedback.feedback = 1 # Ongoing
            self._as.publish_feedback(feedback)
            self._whole_body.move_end_effector_by_line(line_traj,line_dis)
            self.close_gripper()

            self._whole_body.move_end_effector_by_line((1,0,0), 0.1)
        except Exception as e:
            rospy.logwarn("Failed moving body. {}".format(e))
            result.action_success = False
            feedback.feedback = 2  # Failure
            self._as.publish_feedback(feedback)
            self._as.set_aborted(result)
            return

        if self.holding_object:
            result.action_success = True
            self._as.set_succeeded(result)
            feedback.feedback=0
            self._as.publish_feedback(feedback)
        else:
            rospy.logwarn("holding_object not registering")
            result.action_success = True
            feedback.feedback = 2 # Failure
            self._as.publish_feedback(feedback)
            self._as.set_aborted(result)
            
        rospy.loginfo("Grasp action completed")
        return
   


if __name__ == '__main__':
    rospy.init_node('manip_grasp')
    server = ManipGrasp(rospy.get_name())
    rospy.spin()
    