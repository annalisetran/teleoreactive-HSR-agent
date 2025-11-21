#!/usr/bin/env python3
import rospy
import sys
import numpy as np
import math
from typing import List, Tuple
from unsw_vision_msgs.msg import DetectionList, BagPointDetection, PersonDetection, ObjectDetection
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

"""
Detects when someone is pointing.
once someone is pointing, then scans the area for bags.
Once the area is scanned, publishes the location of the 
closest 
"""


class BagPointingDetector:

    def __init__(self):
        detectionIn_topic = rospy.get_param("/BagPointingDetector/in_topic")
        detectionOut_topic = rospy.get_param("/BagPointingDetector/out_topic")
        if not detectionIn_topic:
            rospy.loginfo("No detection in topic for BagHadleIdentify")
        if not detectionOut_topic:
            rospy.loginfo("No detection output topic for BagHadleIdentify")
        rospy.loginfo("initialise bag point detector")
        self.sub = rospy.Subscriber(detectionIn_topic, DetectionList, self.detectionListCallback, queue_size=10)
        self.pub = rospy.Publisher(detectionOut_topic, BagPointDetection, queue_size=1)

        self.POINTING_ERROR = 0.05 # degree to which someone is considered pointing
        self.HIP_DISTANCE = 0.2 # distance which wrist needs to be away from hip

    def detectionListCallback(self, msg: DetectionList) -> None:
        #rospy.loginfo("In callback")
        for person in msg.people:
            rospy.loginfo("In callback for person")
            personPointing = self.isPersonPointing(person)
            if personPointing[0]:
                # rospy.loginfo("person is pointing!!!!")
                bagHandle = self.checkBagIntersection(msg.objects, person, personPointing[1])
                if bagHandle == None:
                    return
                rospy.loginfo("pointing at a bag!")
                toPub: BagPointDetection = BagPointDetection()
                toPub.header = msg.header
                toPub.handleCoordinates = bagHandle.position
                toPub.pointingPerson = person
                self.pub.publish(toPub)
    
    
    def isPersonPointing(self, person: PersonDetection) -> Tuple[bool]:
        """
        Returns a tuple containing two booleans. the first boolean is true if the
        person is pointing, the second boolean is true if the person is pointing with their left hand
        """
        if len(person.skeleton) < 17:
            return False, False
        
        # left arm
        leftShoulder = Vector(person.skeleton[6].x, person.skeleton[6].y, person.skeleton[6].z)
        leftElbow = Vector(person.skeleton[8].x, person.skeleton[8].y, person.skeleton[8].z)
        leftWrist = Vector(person.skeleton[10].x, person.skeleton[10].y, person.skeleton[10].z)
        leftHip = Vector(person.skeleton[12].x, person.skeleton[12].y, person.skeleton[12].z)
        # right arm
        rightShoulder = Vector(person.skeleton[5].x, person.skeleton[5].y, person.skeleton[5].z)
        rightElbow = Vector(person.skeleton[7].x, person.skeleton[7].y, person.skeleton[7].z)
        rightWrist = Vector(person.skeleton[9].x, person.skeleton[9].y, person.skeleton[9].z)
        rightHip = Vector(person.skeleton[11].x, person.skeleton[11].y, person.skeleton[11].z)
        
        leftZeroCheck = leftShoulder.checkZero() or leftElbow.checkZero() or leftWrist.checkZero() or leftHip.checkZero()
        rightZeroCheck = rightShoulder.checkZero() or rightElbow.checkZero() or rightWrist.checkZero() or rightHip.checkZero()
                
        # makes sure all points are found, then checks distance
        if (not leftZeroCheck 
            and shortDistance(leftShoulder, leftWrist, leftElbow) <= self.POINTING_ERROR
            and leftWrist.distance(leftHip) >= self.HIP_DISTANCE):
            return True, True
        

        if (not rightZeroCheck 
            and shortDistance(rightShoulder, rightWrist, rightElbow) <= self.POINTING_ERROR
            and rightWrist.distance(rightHip) >= self.HIP_DISTANCE):
            return True, False
        return False, False


    def checkBagIntersection(self, objects: List[ObjectDetection], pointingPerson: PersonDetection, leftPoint: bool):
        """
        params: 
            objects: list of detected objects
            pointingPerson: person detected as pointing
            leftPoint: true if person is pointing with their left hand, false if right hand
        """
        closestBagDistance = math.inf
        closestBag = None
        # TODO: check divide by zero
        # getting pointing wrist and elbow
        elbow = pointingPerson.skeleton[7]
        wrist = pointingPerson.skeleton[9]
        if leftPoint:
            elbow = pointingPerson.skeleton[8]
            wrist = pointingPerson.skeleton[10]

        # checking if point is close to bags
        for handle in list(filter(lambda obj: obj.object_class == "baghandle", objects)):
            distance = shortDistance(
                Vector(elbow.x, elbow.y, elbow.z),
                Vector(wrist.x, wrist.y, wrist.z),
                Vector(handle.position.x, handle.position.y, handle.position.z),
            )
            if distance < closestBagDistance:
                closestBagDistance = distance
                closestBag = handle
        return closestBag

    # def getPerpendicularDistance(self, A: np.array, B: np.array, C: np.array) -> float:
    #     """
    #     Adapted from https://math.stackexchange.com/questions/1905533/find-perpendicular-distance-from-point-to-line-in-3d
    #     Takes 3 points, where A and B are points on the line, and C is the point to find perpendicular distance to
    #     the line AB from. 
    #     """
    #     d: np.array = (A - B) / np.linalg.norm(A - B)
    #     v: np.array = C - A
    #     t: float = v.dot(d)
    #     P: np.array = B + t * d
    #     return np.linalg.norm(P - A) # returning the distance


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
            

if __name__ == "__main__":
    try:
        rospy.init_node('BagPointingDetector', anonymous=True)
        obc = BagPointingDetector()
        rospy.spin()
    except (KeyboardInterrupt, rospy.ROSInterruptException) as e:
        rospy.logfatal("Error occurred! Stopping the scene manager node...")
        rospy.sleep(1)
        print("BagPointingDetector node terminated")



# def main(args):
#     obc = BagPointingDetector()
#     rospy.init_node('BagPointingDetector', anonymous=True)
#     try:
#         rospy.spin()
#     except KeyboardInterrupt:
#         print("Shutting down")

# if __name__ == '__main__':
#     main(sys.argv)