#! /usr/bin/env python3

import rospy
import actionlib

from action_server.msg import doActionAction

class doAction(object):
    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, doActionAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        rospy.loginfo("executing")
        r = rospy.Rate(1)
        success = True
        
        for i in range(1, goal):
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            self._feedback = i
            self._as.publish_feedback(self._feedback)
        
        if success:
            self._result = True
            rospy.logingo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('doAction')
    server = doAction(rospy.get_name())
    rospy.spin()
    