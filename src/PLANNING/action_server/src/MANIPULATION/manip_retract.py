#! /usr/bin/env python3

import rospy
import actionlib
import hsrb_interface
from hsrb_interface import geometry
import tf2_geometry_msgs
from action_server.msg import ManipRetractAction, ManipRetractFeedback, ManipRetractResult
from math import pi
import tf2_ros
from geometry_msgs.msg import WrenchStamped, Twist
PALM_SIZE = 0.06
class ManipRetract(object):
    def __init__(self, name):
        # init the robot
        self._robot = hsrb_interface.Robot()
        self._whole_body = self._robot.get('whole_body')
        self._gripper = self._robot.get('gripper')
        self.move_base_pub = rospy.Publisher("hsrb/command_velocity", Twist, queue_size=1)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.wrist_pressure = 0
        rospy.Subscriber("hsrb/wrist_wrench/raw", WrenchStamped, self.wristCB)

        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, ManipRetractAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo("Action server manip-retract started")

        
        # TODO: update the value to share between servers
        self.initial_wrist_pressure = 0

    def wristCB(self, data):
      self.wrist_pressure = data.wrench.force.x
      # pressure on the wrist when grip opens vertically
      self.grip_pressure = data.wrench.force.y

    def move_to_carry_pose(self):
        jointValues = {
            'arm_flex_joint': -0.029939646257455266,
            'arm_roll_joint': 1.2387664454216463e-06,
            'wrist_flex_joint': -1.57,
            'wrist_roll_joint': 0.0
        }
        self._whole_body.move_to_joint_positions(jointValues)

        jointValues = {
            'arm_flex_joint': -0.13,
            'arm_roll_joint': 2.28,
            'wrist_flex_joint': -1.57,
            'wrist_roll_joint': 0.0
        }
        self._whole_body.move_to_joint_positions(jointValues)
        

        t = rospy.Time.now().to_sec() 
        while(True):
            if rospy.Time.now().to_sec()  - t > 1.5:
                break
            t_msg = Twist()
            t_msg.linear.x = -0.05
            self.move_base_pub.publish(t_msg)

        # lower the arm lift
        jointValues = {
            'arm_lift_joint': 0.15,
        }
        self._whole_body.move_to_joint_positions(jointValues)

    def execute_cb(self, goal):
        rospy.loginfo("Executing retract action")
        r = rospy.Rate(1)
        feedback = ManipRetractFeedback()
        result = ManipRetractResult()
        # default to be true
        result.action_success=True
        feedback.feedback = 1 # Ongoing
        self._as.publish_feedback(feedback)
        try:
            # move the tensor up
            # get current joint value
            x = self._whole_body.joint_positions.get('arm_lift_joint')
            x = min(0.6,x+0.1)
            jointValues = {
            'arm_lift_joint': x,
            }
            self._whole_body.move_to_joint_positions(jointValues)

            self._whole_body.move_end_effector_by_line((1,0,0), 0.05)
            rospy.sleep(0.5)
            self._whole_body.move_end_effector_by_line((0,0,-1),0.3)
            rospy.sleep(1)
            # move back to original pose
            self._whole_body.move_to_neutral()
            # self._whole_body.move_to_neutral()
            # TODO: implement weight checking
            if self._gripper.get_distance() >0.4 or self.wrist_pressure - self.initial_wrist_pressure > 2:
                rospy.logwarn("successfully picked up")
            else:
                rospy.logwarn("successfully picked up")
                #return False
            self.move_to_carry_pose()
        except Exception as e:
            rospy.logwarn("Failed moving body. {}".format(e))
            result.action_success = False
            feedback.feedback = 2  # Failure
            self._as.publish_feedback(feedback)
            self._as.set_aborted(result)
            return
        
        result.action_success = True
        self._as.set_succeeded(result)
        feedback.feedback=0
        self._as.publish_feedback(feedback)
        return



        


        
        


if __name__ == '__main__':
    rospy.init_node('manip_retract')
    server = ManipRetract(rospy.get_name())
    rospy.spin()
    