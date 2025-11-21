#! /usr/bin/env python3

# use moveit to plan and execute a pickup action

import rospy
import actionlib
import moveit_commander
import geometry_msgs.msg
from math import pi
import tf
import moveit_msgs
import trajectory_msgs
from copy import deepcopy
class ManipPickup():
    def __init__(self, name):
        self._action_name = name
        # self._as = actionlib.SimpleActionServer(self._action_name, ManipPickupAction, execute_cb=self.execute_cb, auto_start = False)
        # self._as.start()
        # rospy.loginfo("Action server manip-pickup started")

        self._robot = moveit_commander.RobotCommander()
        
        # print all the groups in the robot

        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.arm.allow_replanning(True)
        self.wholebody = moveit_commander.MoveGroupCommander("whole_body")
        self.scene = moveit_commander.PlanningSceneInterface()

        

        rospy.sleep(1)
        

        # move_to_neutral
        rospy.loginfo("step1: move_to_neutral")

        # self.arm.set_named_target("neutral")
        # self.arm.go()

        rospy.logdebug("done")




        

    def execute_cb(self, goal):
        # use moveit to plan and execute a pickup action to pick up the object at 
        pass

    def pickup_demo(self):
        # plan and execute a pickup action to pick up the object at (1,0,0.6) in the base_footprint frame
        position = geometry_msgs.msg.Point(x=1, y=1, z=0.7)
        # this is from the top
        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        orientation = geometry_msgs.msg.Quaternion(*quaternion)
        target_pose = geometry_msgs.msg.Pose(position=position, orientation=orientation)
        

        # Plan to the target pose
        self.arm.set_pose_target(target_pose)
        plan = self.arm.plan()
        print(plan)
        print("Planning")

if __name__ == '__main__':
    rospy.init_node('manip_pickup')
    server = ManipPickup(rospy.get_name())
    rospy.loginfo("Action server manip-pickup started")
    rospy.sleep(3)
    server.pickup_demo()
    rospy.spin()