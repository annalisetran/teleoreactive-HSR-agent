#!/usr/bin/env python3

# World Model Timer
# Performs functions at specified time interval

# Dependencies:
# World Model - database with schema 2.1.9 or higher

# Written By: Adam Golding

import rospy
import os
from dbase import DBase
from object_api import ObjectApi

###
#
###
class WMTimer():
    def __init__(self) -> None:
        # parameters
        self.decay_timer_interval = rospy.get_param('~decay_timer_interval')    # the time interval between execution of decay process (in seconds)
        self.decay_cycle = rospy.get_param('~decay_cycle')                # the time in seconds for a 100% decay at a standard rate
        self.db_ini = rospy.get_param('~db_ini')                          # file containing database connection details     

        self.db = DBase(os.path.abspath(self.db_ini))                     # database connections
        self.obj_mgr = ObjectApi()

        self.std_decay_rate = 1.0/self.decay_cycle                        # our standard decay rate based on the decay cycle

        # log info
        rospy.loginfo("decay_timer_interval="+str(self.decay_timer_interval))
        rospy.loginfo("decay_cycle="+str(self.decay_cycle))
        rospy.loginfo("db_ini="+self.db_ini)     

        wm_timer = rospy.Timer(rospy.Duration(self.decay_timer_interval), self.timer_callback)
        rospy.loginfo("World Model Timer started")    

        
    def timer_callback(self, event):
        # update the location confidence for all current objects
        rospy.loginfo("WM Timer executed")
        self.obj_mgr.update_location_confidence_all_objects(self.db.con_pool, self.std_decay_rate, self.decay_timer_interval)



if __name__ == "__main__":
    try:
        rospy.init_node('wm_timer_node')
        wmt = WMTimer()        
        rospy.spin()
    except (KeyboardInterrupt, rospy.ROSInterruptException) as e:
            rospy.logfatal("Error occurred! Stopping the world model timer node...")
            rospy.sleep(1)
            print("wm_timer_node terminated")
            
                