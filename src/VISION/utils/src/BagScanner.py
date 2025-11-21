#!/usr/bin/env python3
import rospy
import sys
import numpy as np
import math
from typing import List, Tuple
from unsw_vision_msgs.msg import DetectionList, BagPointDetection, PersonDetection, ObjectDetection, PersonPointing
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Bool
from geometry_msgs.msg import Point

"""
Detects when someone is pointing.
once someone is pointing, then scans the area for bags.
Once the area is scanned, publishes the location of the 
closest bag
"""


class BagScanner:

    def __init__(self):
        detectionIn_topic = rospy.get_param("/BagScanner/in_topic")
        pointingIn_topic = rospy.get_param("/BagScanner/point_topic")
        is_ready_topic = rospy.get_param("/BagScanner/ready_topic")
        detectionOut_topic = rospy.get_param("/BagScanner/out_topic")
        if not detectionIn_topic:
            rospy.loginfo("No detection in topic for BagScanner")
        if not detectionOut_topic:
            rospy.loginfo("No detection output topic for BagScanner")
        rospy.loginfo("initialise bag point detector")
        self.sub = rospy.Subscriber(detectionIn_topic, DetectionList, self.detectionListCallback, queue_size=10)
        self.pointSub = rospy.Subscriber(pointingIn_topic, PersonPointing, self.pointingCallback, queue_size=10)
        self.readySub = rospy.Subscriber(is_ready_topic, Bool, self.readyCallback, queue_size=10)
        self.pub = rospy.Publisher(detectionOut_topic, BagPointDetection, queue_size=1)
        self.move_head_pub = rospy.Publisher('hsrb/head_trajectory_controller/command', JointTrajectory, queue_size=1)
        
        self.ready = False
        self.pointFound = False
        self.closestBag = False
        self.closestBagDistance = False
    
    def readyCallback(self, msg: Bool):
        self.ready = msg.data

    def pointingCallback(self, msg: PersonPointing):
        if not self.ready:
            return
        
        rospy.loginfo("point callback reached!")
        # self.pointSub.unregister()
        self.pointFound = msg
        rospy.loginfo(f'start scan, left bool is {self.pointFound.left}')
        self.doHeadScan()
        rospy.loginfo("head scanned!")
        rospy.sleep(16)
        self.ready = False
        if not self.closestBag:
            rospy.loginfo("Could not find a bag :(")
            toPub: BagPointDetection = BagPointDetection()
            toPub.header = msg.header
            toPub.handleCoordinates = Point()
            toPub.handleCoordinates.z = -1
            toPub.pointingPerson = msg.person
            self.returnHeadPos()
            self.pub.publish(toPub)
        else:
            rospy.loginfo("pointing at a bag!")
            toPub: BagPointDetection = BagPointDetection()
            toPub.header = msg.header
            toPub.handleCoordinates = self.closestBag.position
            toPub.pointingPerson = msg.person
            self.pub.publish(toPub)
            self.returnHeadPos()
            rospy.signal_shutdown("shutdown")
        

    def detectionListCallback(self, msg: DetectionList) -> None:
        if not self.pointFound:
            return
        rospy.loginfo("detected something while scanning")
        self.checkBagIntersection(msg.objects, self.pointFound)
        



    def checkBagIntersection(self, objects: List[ObjectDetection], pointDetection: PersonPointing):
        """
        params: 
            objects: list of detected objects
            pointingPerson: person detected as pointing
            leftPoint: true if person is pointing with their left hand, false if right hand
        """
        for obj in objects:
            rospy.loginfo(f"detected following item: {obj.object_class}")

        # checking if point is close to bags
        for handle in list(filter(lambda obj: obj.object_class == "baghandle", objects)):
            if handle.bbox.x == 0 and handle.bbox.y == 0:
                continue
            if handle.position.x == 0 and handle.position.y == 0:
                continue
            distance = shortDistance(
                Vector(pointDetection.elbow.x, pointDetection.elbow.y, pointDetection.elbow.z),
                Vector(pointDetection.wrist.x, pointDetection.wrist.y, pointDetection.wrist.z),
                Vector(handle.position.x, handle.position.y, handle.position.z),
            )
            rospy.loginfo("calc bag handle distance" + str(distance))
            if not self.closestBag and not self.closestBagDistance:
                self.closestBag = handle
                self.closestBagDistance = distance
            elif distance < self.closestBagDistance:
                self.closestBag = handle
                self.closestBagDistance = distance
    """
    MOVE HEAD
    Topic details:
    - position: length two array
        - index 0: horizontal joint (max 1.75 left, min -3.84)
        - index 1: vertical joint (min -1.57, max 0.52) - positive = up
    """
    def doHeadScan(self):
        
        # headPos = JointTrajectory()
        # headPos.joint_names = ["head_pan_joint", "head_tilt_joint"]
        # point1 = JointTrajectoryPoint()
        # point1.positions = [0, -0.8]
        # point1.time_from_start = rospy.Duration(secs=1)
        # point2 = JointTrajectoryPoint()
        # point2.positions = [1.3, -0.8]
        # point2.time_from_start = rospy.Duration(secs=8)
        # point3 = JointTrajectoryPoint()
        # point3.positions = [-1.3, -0.8]
        # point3.time_from_start = rospy.Duration(secs=16)
        # headPos.points = [point1, point2, point3]
        # self.move_head_pub.publish(headPos)
        
        
        rospy.sleep(1)
        if not self.pointFound.left:
            rospy.loginfo("move head to left(robot perspective)")
            headPos = JointTrajectory()
            headPos.joint_names = ["head_pan_joint", "head_tilt_joint"]
            point1 = JointTrajectoryPoint()
            point1.positions = [0, -0.6]
            point1.time_from_start = rospy.Duration(secs=1)
            point2 = JointTrajectoryPoint()
            point2.positions = [1.3, -0.6]
            point2.time_from_start = rospy.Duration(secs=8)
            point3 = JointTrajectoryPoint()
            point3.positions = [1.3, -0.0]
            point3.time_from_start = rospy.Duration(secs=10)
            point4 = JointTrajectoryPoint()
            point4.positions = [0.0, -0.0]
            point4.time_from_start = rospy.Duration(secs=12)
            headPos.points = [point1, point2, point3,point4]
            self.move_head_pub.publish(headPos)
        else:
            rospy.loginfo("move head to right(robot perspective)")
            headPos = JointTrajectory()
            headPos.joint_names = ["head_pan_joint", "head_tilt_joint"]
            point1 = JointTrajectoryPoint()
            point1.positions = [0, -0.6]
            point1.time_from_start = rospy.Duration(secs=1)
            point2 = JointTrajectoryPoint()
            point2.positions = [-1.3, -0.6]
            point2.time_from_start = rospy.Duration(secs=8)
            point3 = JointTrajectoryPoint()
            point3.positions = [-1.3, -0.0]
            point3.time_from_start = rospy.Duration(secs=10)
            point4 = JointTrajectoryPoint()
            point4.positions = [0.0, -0.0]
            point4.time_from_start = rospy.Duration(secs=12)
            headPos.points = [point1, point2, point3,point4]
            self.move_head_pub.publish(headPos)
        return headPos
    
    def returnHeadPos(self):
        headPos = JointTrajectory()
        headPos.joint_names = ["head_pan_joint", "head_tilt_joint"]
        point1 = JointTrajectoryPoint()
        point1.positions = [0, 0]
        point1.time_from_start = rospy.Duration(1)
        headPos.points = [point1]
        self.move_head_pub.publish(headPos)
        return headPos


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
        return self.x == 0 and self.y == 0 and self.z == 0
 
# calculate shortest dist. from point to line
def shortDistance(line_point1: Vector, line_point2: Vector, point: Vector):
    AB = line_point2 - line_point1
    AC = point - line_point1
    area = (AB * AC).magnitude()
    CD = area / AB.magnitude()
    return CD
            





def main(args):
    obc = BagScanner()
    rospy.init_node('BagScanner', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)