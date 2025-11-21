#! /usr/bin/env python3

# Detect features of an individual person to ensure Blinky follows the same person
# Basic logic
    # Get features of person (ideally from points on skeleton in vision, can get the colour of the pants at their hip + ankle and the colour of their top)
    # Use openCV to get the colour of that radius of point
    # Coninuously check that the person we are following matches those features 
    # If they don't match, find someone who does ... OR automatically go back into the stadium ... OR ask for help and then detect a waving person
    
# How this file relates to controller logic:
    # When picking up the bag, get the features of the person giving us the bag, storing in database
    # When going back into the stadium, check that the person we have in view matches the features stored

import rospy
import sys
import os
sys.path.remove('/home/robocupathome/.local/lib/python3.8/site-packages')
sys.path.append('/home/robocupathome/workspace/robocup_ws/src/world_model/world_model_store/scripts')
from db import Db
import cv2
import numpy
from cv_bridge import CvBridge
# Import vision message structure, this is the skeleton one 
# Skeleton is in PersonDetection.msg, which is what people in PersonDetection.msg stores
from vision_msgs.msg import DetectionList, PersonDetecting
from colorthief import ColorThief
import matplotlib.pyplot as plt

class feature_recognition:
    def __init__(self) -> None:
        self.img_topic = rospy.get_param('~img_topic','/hsrb/head_center_camera/image_raw')     # topic to get the image from       

        self.skeleton_sub = DetectionList.people
        
        
    def find_features(self):
        for person in self.skeleton_sub:
            # Left leg colour
            # This is assuming leg is vertical and not moving in any other plane/direction ... don't think we should assume this though...
            # Also need to check that these array values actually correspond to these points
            left_hip = person.skeleton[11]
            left_knee = person.skeleton[13]
            # Tuple of (x, y) point for midpoint of left leg
            self.left_leg_midpoint = (left_hip.x (left_knee.y + ((left_hip.y - left_knee.y)/2)))
            # Finds a circle around the centre pixel, returns an image 
            self.left_leg_circle = cv2.circle(self.img_topic, self.left_leg_midpoint, 20, (255,0,0), 1)
            # Find the most common colour in this circle and store this value in the database
            
            # Trying to store a cropped section of the original image as a new image/set of data points
            # Need to check this as I feel it is very wrong....
            self.cropped_image = self.img_topic[(self.left_leg_midpoint-20):(self.left_leg_midpoint+20), (self.left_leg_midpoint-20):(self.left_leg_midpoint+20)]
            
            self.dominant_colour = self.cropped_image.get_colour(quality=1)
            plt.imshow([[self.dominant_colour]])
            plt.show()
            
            
            # Make sure to convert final colour found to RGB as openCV uses BGR by default
            
            # Store in database
            
            # Repeat for Right leg, and arms (shoulder and elbow as points) 
            # Right leg colour
            right_hip = person.skeleton[12]
            right_knee = person.skeleton[14]
            self.right_leg_midpoint = right_knee.y + ((right_hip.y - right_knee.y)/2)
            
            # Find a radius of 20 pixels around this pixel
            # Find the most common colour in this circle and store this value in the database
            
            left_shoulder = person.skeleton[5]
            left_elbow = person.skeleton[7]
            self.left_arm_midpoint = left_elbow.y + ((left_shoulder.y - left_elbow.y)/2)
            
            right_shoulder = person.skeleton[6]
            right_elbow = person.skeleton[8]
            self.right_arm_midpoint = right_elbow.y + ((right_shoulder.y - right_elbow.y)/2)


if __name__ == "__main__":
    rospy.init_node('feature_recognition')
    feature_recognition_node = feature_recognition()
    while not rospy.is_shutdown(): 
        feature_recognition_node.find_features()



