#!/usr/bin/env python3

# Scene Manager
# Creates a scene and saves the objects from the Detection msg to the world model database
# The world model database contains tables related to the scene and scene objects

# Dependencies:
# World Model - database with schema 2.1.9 or higher

# Written By: Adam Golding

import rospy
import os
# from std_msgs.msg import String
# from geometry_msgs.msg import Point
# from sensor_msgs.msg import Image
from unsw_vision_msgs.msg import PersonDetection, DetectionList, BoundingBox
from dbase import DBase
from tracker import Tracker


###
#
###
class SceneManager():
    def __init__(self) -> None:
        # parameters
        self.input_topic = rospy.get_param('~input_topic')                  # topic to get the input Detection message from 
        self.frame_id = rospy.get_param('~frame_id')                        # the frame id that will be used in the scene
        self.db_ini = rospy.get_param('~db_ini')                            # file containing database connection details
        self.cost_threshold = rospy.get_param('~cost_threshold')            # the cost threshold used in the Tracker function
        self.step_size = rospy.get_param('~step_size')                      # the step size used in the Tracker function
        self.show_plots = rospy.get_param('~show_plots')                    # flag to indicate whether to show the plots

        # subscribers
        self.image_listener = rospy.Subscriber(self.input_topic, DetectionList, self.process_scene) # subscribe to the input topic

        self.db = DBase(os.path.abspath(self.db_ini))               # database connections
        #self.scene_mgr = SceneApi()

        # Setup Tracker for tracking objects in the World Model
        #initialise the tracker
        self.tracker = Tracker(self.step_size, self.db_ini, self.cost_threshold)

        if (self.show_plots):
            self.tracker.init_plot()

        # log info
        rospy.loginfo("input_topic="+self.input_topic)
        rospy.loginfo("frame_id="+self.frame_id)
        rospy.loginfo("db_ini="+self.db_ini)     
        rospy.loginfo("cost_threshold="+str(self.cost_threshold))  
        rospy.loginfo("step_size="+str(self.step_size)) 
        #rospy.loginfo("show_plots="+str(self.show_plots))
        rospy.loginfo("Scene Manager started")    

        
    def process_scene(self, input_msg):
        # check if there are any objects in the message
        rospy.loginfo(f"Objects in scene: {len(input_msg.objects)}")
        if len(input_msg.objects) > 0:
            # Update predictions
            self.tracker.predict()

            scene_objs = input_msg.objects
            if scene_objs != None:                
                self.tracker.update(input_msg.header.seq, scene_objs,input_msg.image.width,input_msg.image.height) 

            if (self.show_plots):
                self.tracker.plot_world()
                print(self.tracker.print_tracked_objects())

            if self.show_plots:
                self.tracker.plot_world()
                
        # check removal of objects


if __name__ == "__main__":
    try:
        rospy.init_node('scene_manager_node')
        sm = SceneManager()        
        rospy.spin()
    except (KeyboardInterrupt, rospy.ROSInterruptException) as e:
            rospy.logfatal("Error occurred! Stopping the scene manager node...")
            rospy.sleep(1)
            print("scene_manager_node terminated")
            
                