#!/usr/bin/env python3
import rospy
import sys
import numpy as np
import math
from typing import List, Tuple
from unsw_vision_msgs.msg import DetectionList, BagPointDetection, PersonDetection, PersonPointing

"""
Detects when someone is pointing.
once someone is pointing, then scans the area for bags.
Once the area is scanned, publishes the location of the 
closest 
"""


class PersonPointingDetector:

    def __init__(self):
        input_topic = rospy.get_param("~input_topic")
        output_topic = rospy.get_param("~output_topic")
        if not input_topic:
            rospy.loginfo("No input topic for PersonPointingDetector")
        if not output_topic:
            rospy.loginfo("No output topic for PersonPointingDetector")
        rospy.loginfo("Initialise person pointing detector")
        self.sub = rospy.Subscriber(input_topic, DetectionList, self.detectionListCallback, queue_size=10)
        self.pub = rospy.Publisher(output_topic, PersonPointing, queue_size=1)
        self.pubHeader = None
        self.POINTING_ERROR = 0.05 # degree to which someone is considered pointing
        self.HIP_DISTANCE = 0.2 # distance which wrist needs to be away from hip
        rospy.loginfo("Person Pointing Detector Parameters Set")
        rospy.loginfo(f"input topic: {input_topic}")
        rospy.loginfo(f"output topic: {output_topic}")
        rospy.loginfo(f"Pointing Error: {self.POINTING_ERROR}")
        rospy.loginfo(f"Hip Distance: {self.HIP_DISTANCE}")
        

    def detectionListCallback(self, msg: DetectionList) -> None:
        self.pubHeader = msg.header
        rospy.loginfo("start1")
        for person in msg.people:
            rospy.loginfo("start")
            self.isPersonPointing(person)
    
    
    def isPersonPointing(self, person: PersonDetection) -> Tuple[bool]:
        """
        Returns a tuple containing two booleans. the first boolean is true if the
        person is pointing, the second boolean is true if the person is pointing with their left hand.
        If a person is pointing, also publishes that they are pointing with their respective hand
        """
        rospy.loginfo("points hereo")
        if len(person.skeleton) < 17:
            rospy.loginfo("points less")
            return False, False
        
        # maybe right arm
        rightShoulder = Vector(person.skeleton[6].x, person.skeleton[6].y, person.skeleton[6].z)
        rightElbow = Vector(person.skeleton[8].x, person.skeleton[8].y, person.skeleton[8].z)
        rightWrist = Vector(person.skeleton[10].x, person.skeleton[10].y, person.skeleton[10].z)
        rightHip = Vector(person.skeleton[12].x, person.skeleton[12].y, person.skeleton[12].z)
        # maybe left arm
        leftShoulder = Vector(person.skeleton[5].x, person.skeleton[5].y, person.skeleton[5].z)
        leftElbow = Vector(person.skeleton[7].x, person.skeleton[7].y, person.skeleton[7].z)
        leftWrist = Vector(person.skeleton[9].x, person.skeleton[9].y, person.skeleton[9].z)
        leftHip = Vector(person.skeleton[11].x, person.skeleton[11].y, person.skeleton[11].z)
        
        leftZeroCheck = leftShoulder.checkZero() or leftElbow.checkZero() or leftWrist.checkZero() or leftHip.checkZero()
        rightZeroCheck = rightShoulder.checkZero() or rightElbow.checkZero() or rightWrist.checkZero() or rightHip.checkZero()
        
        pubMsg = PersonPointing()
        pubMsg.person = person
        pubMsg.header = self.pubHeader
        # makes sure all points are found, then checks distance
        #rospy.loginfo(f'left is {leftWrist.distance(leftHip)}, right distance is {rightWrist.distance(rightHip)}')
        # rospy.loginfo(f'total number of skekton joint is {len(person.skeleton)}')
        if (not leftZeroCheck 
            and shortDistance(leftShoulder, leftWrist, leftElbow) <= self.POINTING_ERROR
            and leftWrist.distance(leftHip) >= self.HIP_DISTANCE):
            pubMsg.left = True
            pubMsg.wrist = leftWrist
            pubMsg.elbow = leftElbow
            self.pub.publish(pubMsg)
            return True, True
        

        if (not rightZeroCheck 
            and shortDistance(rightShoulder, rightWrist, rightElbow) <= self.POINTING_ERROR
            and rightWrist.distance(rightHip) >= self.HIP_DISTANCE):
            pubMsg.left = False
            pubMsg.wrist = rightWrist
            pubMsg.elbow = rightElbow
            self.pub.publish(pubMsg)
            return True, False
        return False, False


# Linear Algebra assistant class
 
class Vector:
    def __init__(self, x, y, z):
        # Constructor
        self.x = x
        self.y = y
        self.z = z
         
    # ADD 2 Vectors
    def __add__(self, v):
        x1, y1, z1 = self.x + v.x, self.y + v.y, self.z + v.z
        return Vector(x1, y1, z1)
 
    # Subtract 2 vectors
    def __sub__(self, v):
        x1, y1, z1 = self.x - v.x, self.y - v.y, self.z - v.z
        return Vector(x1, y1, z1)
 
    # Dot product of 2 vectors
    def __xor__(self, v):
        x1, y1, z1 = self.x * v.x, self.y * v.y, self.z * v.z
        return x1 + y1 + z1
 
    # Cross product of 2 vectors
    def __mul__(self, v):
        x1 = self.y * v.z - self.z * v.y
        y1 = self.z * v.x - self.x * v.z
        z1 = self.x * v.y - self.y * v.x
        return Vector(x1, y1, z1)
 
    # Display Vector
    def __str__(self):
        out = str(self.x)+"i "
        if self.y >= 0:
            out += "+ "
        out += str(self.y)+"j "
        if self.z >= 0:
            out += "+ "
        out += str(self.z)+"k"
        return out
 
    def magnitude(self):
        return math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)
    
    def distance(self, vector):
        return math.sqrt((self.x - vector.x) ** 2 + (self.y - vector.y) ** 2 + (self.z - vector.z) ** 2)
    
    def checkZero(self):
        return (self.x == 0 and self.y == 0) or self.z == 0
 
# calculate shortest dist. from point to line
def shortDistance(line_point1: Vector, line_point2: Vector, point: Vector):
    AB = line_point2 - line_point1
    AC = point - line_point1
    area = (AB * AC).magnitude()
    CD = area / AB.magnitude()
    return CD
            

if __name__ == "__main__":
    try:
        rospy.init_node('PersonPointingDetector', anonymous=True)
        obc = PersonPointingDetector()
        rospy.spin()
    except (KeyboardInterrupt, rospy.ROSInterruptException) as e:
        rospy.logfatal("Error occurred! Stopping the scene manager node...")
        rospy.sleep(1)
        print("scene_manager_node terminated")