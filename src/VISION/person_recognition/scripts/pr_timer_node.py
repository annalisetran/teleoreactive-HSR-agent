#!/usr/bin/env python3

# Person Recognition Timer
# Performs functions at specified time interval

# Dependencies:
# World Model - database with schema 2.4 or higher

# Written By: Adam Golding

import rospy
import os
from dbase import DBase
from agent_api import AgentApi

###
#
###
class PRTimer():
    def __init__(self) -> None:
        # parameters
        self.timer_interval = rospy.get_param('~timer_interval')    # the time interval between execution of decay process (in seconds)
        self.db_ini = rospy.get_param('~db_ini')                    # file containing database connection details     

        self.db = DBase(os.path.abspath(self.db_ini))               # database connections
        self.agent_mgr = AgentApi()                                 # agent manager

        # log info
        rospy.loginfo("timer_interval="+str(self.timer_interval))
        rospy.loginfo("db_ini="+self.db_ini)     

        pr_timer = rospy.Timer(rospy.Duration(self.timer_interval), self.timer_callback)
        rospy.loginfo("Person Recognition Timer started")    

        
    def timer_callback(self, event):
        rospy.loginfo("PR Timer executed")
        # check if any people have been re-detected as another agent using face recognition
        self.obj_mgr.update_location_confidence_all_objects(self.db.con_pool, self.std_decay_rate, self.timer_interval)


if __name__ == "__main__":
    try:
        rospy.init_node('pr_timer_node')
        prt = PRTimer()        
        rospy.spin()
    except (KeyboardInterrupt, rospy.ROSInterruptException) as e:
            rospy.logfatal("Error occurred! Stopping the person recognition timer node...")
            rospy.sleep(1)
            print("pr_timer_node terminated")
            
                