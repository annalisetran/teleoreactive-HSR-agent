#!/usr/bin/env python3

import rospy

from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import  Marker
from geometry_msgs.msg import Point32
from unsw_vision_msgs.msg import DetectionList
from unsw_vision_msgs.msg import PersonDetection
from unsw_vision_msgs.msg import VisualVector
from std_msgs.msg import ColorRGBA


from unsw_vision_msgs.msg import PersonDetection, VisualVector
from geometry_msgs.msg import Point, Point32, Vector3
import numpy as np
from typing import List, Union
from sklearn.linear_model import LinearRegression

NOSE = 0
EYE_RIGHT = 1
EYE_LEFT = 2
EAR_RIGHT = 3
EAR_LEFT = 4
SHOULDER_RIGHT = 5
SHOULDER_LEFT = 6
ELBOW_RIGHT = 7
ELBOW_LEFT = 8
HAND_RIGHT = 9
HAND_LEFT = 10
HIP_RIGHT = 11
HIP_LEFT = 12
KNEE_RIGHT = 13
KNEE_LEFT = 14
ANKLE_RIGHT = 15
ANKLE_LEFT = 16


HEAD = 4
ARMS = 10
LEGS = 16

COLOUR_RED = 0
COLOUR_GREEN = 1
COLOUR_BLUE = 2
COLOUR_YELLOW = 3


# Tuple of tuple(point1, point2, colour)
KEY_POINTS_RELATION = (
    # Head
    (EYE_LEFT, EAR_LEFT, COLOUR_GREEN),
    (EYE_RIGHT, EAR_RIGHT, COLOUR_GREEN), 
    (EYE_RIGHT, NOSE, COLOUR_GREEN), 
    (EYE_LEFT, NOSE, COLOUR_GREEN), 

    # Neck
    (EAR_LEFT, SHOULDER_LEFT, COLOUR_GREEN),
    (EAR_RIGHT, SHOULDER_RIGHT, COLOUR_GREEN),

    # The two shoulders
    (SHOULDER_LEFT, SHOULDER_RIGHT, COLOUR_BLUE), 

    # Left Arm
    (SHOULDER_LEFT, ELBOW_LEFT, COLOUR_BLUE), 
    (ELBOW_LEFT, HAND_LEFT, COLOUR_BLUE), 

    # Right Arm
    (SHOULDER_RIGHT, ELBOW_RIGHT, COLOUR_BLUE), 
    (ELBOW_RIGHT, HAND_RIGHT, COLOUR_BLUE), 

    # Left Leg
    (HIP_LEFT, KNEE_LEFT, COLOUR_YELLOW),
    (KNEE_LEFT, ANKLE_LEFT, COLOUR_YELLOW),

    # Right Leg
    (HIP_RIGHT, KNEE_RIGHT, COLOUR_YELLOW),
    (KNEE_RIGHT, ANKLE_RIGHT, COLOUR_YELLOW),

    # Torso
    (SHOULDER_LEFT, HIP_LEFT, COLOUR_RED),
    (SHOULDER_RIGHT, HIP_RIGHT, COLOUR_RED),
    (HIP_LEFT, HIP_RIGHT, COLOUR_RED),
    )


class PeopleVisualiser:

    def __init__(self):
        rospy.init_node('debugger_people_visualiser')
        self.visionSub = rospy.Subscriber('/unsw_vision/detections/objects/positions', DetectionList, self.OnDetection)
        self.debuggerPub = rospy.Publisher('/debugger/people', MarkerArray, queue_size=10)
        self.message = None

        self.marker = MarkerArray()

        rospy.Timer(rospy.Duration(secs=0.3), self.DrawPeople)
        print('People Visualiser Running.')




    def WipeMarker(self):
        m = Marker()
        m.action = Marker.DELETEALL

        self.marker.markers.clear()
        self.marker.markers.append(m)
        self.debuggerPub.publish(self.marker)

        self.marker.markers.clear()


    def OnDetection(self, data:DetectionList):
        self.message = data

    def DrawPeople(self, event=None):
        if self.message is None:
            print('[Debug] Visualisation_People: Message not received yet.')
            return

        self.WipeMarker()

        person:PersonDetection
        personID = 0

        # generate pointing but should not be here. Just for demo.


        for person in self.message.people:
            person = pointing_vector_generation(person)
            i = 0
            exist = [True] * 17 # 17 skeletons
            keyPos = [None] * 17 # Coordinates of the 17 skeletons

            # Check if key points exist.
            # If yes, store the coordinates of them.
            for i in range(17):
                s = person.skeleton[i]

                if s.x == 0.0 and s.y == 0.0 and s.z == 0.0:
                    exist[i] = False
                else:
                    keyPos[i] = s
                    self.marker.markers.append(self.CreateKeyPointMarker(i, personID, s))
            
            

            for relation in KEY_POINTS_RELATION:
                if exist[relation[0]] and exist[relation[1]]:
                    m = self.CreateLineStripe(keyPos[relation[0]], keyPos[relation[1]], relation[0], relation[1], personID, relation[2])
                    self.marker.markers.append(m)

            # place a marker use the person gaze direction
            # gaze vecctor has a origin in its type and also a direction
            r = person.right_pointing_vector
            l = person.left_pointing_vector
            # gaze = person.gaze_vector
            # m = self.CreateLineFromVisualVector(r, personID)
            m1 = self.CreateLineFromVisualVector(l, personID)
            # m2 = self.CreateLineFromVisualVector(gaze, personID)
            # self.marker.markers.append(m2)
            self.marker.markers.append(m1)
            


            personID = personID + 1
        rospy.loginfo('[Debug] Visualisation_People: Drawing ' + str(personID) + ' people.')
        self.debuggerPub.publish(self.marker)


    def CreateLineFromVisualVector(self, vector:VisualVector, personID):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.message.header.stamp

        marker.ns = "person" + str(personID)
        marker.id = 1000 + personID

        marker.action = Marker.ADD
        marker.type = Marker.ARROW

        # maker the marker looks like a ray
        marker.scale.x = 0.2
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # marker.pose.position = vector.origin
        # marker.pose.position.z = 0.5
        # marker.pose.position.x = 0
        # marker.pose.position.y = 0
        # marker.pose.position.z = 0

        # convert the vector to oreientation
        # marker.pose.orientation.x = vector.direction.x
        # marker.pose.orientation.y = vector.direction.y
        # marker.pose.orientation.z = vector.direction.z
        

        #just for testing
        # marker.pose.position.z = 0.5

        # start and end point
        marker.points.append(vector.origin)
        # calculate the end point
        end = Point32()
        end.x = vector.origin.x + (vector.direction.x * 5)
        end.y = vector.origin.y + (vector.direction.y * 5)
        end.z = vector.origin.z + (vector.direction.z * 5)
        marker.points.append(end)



        marker.color = self.Colour_Blue()

        return marker

    def CreateLineStripe(self, p0, p1, pointID0, pointID1, personID, colour):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.message.header.stamp

        marker.ns = "person" + str(personID)
        marker.id = 10 * pointID0 + pointID1

        marker.action = Marker.ADD
        marker.type = Marker.LINE_STRIP

        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03

        marker.points.append(p0)
        marker.points.append(p1)

        if colour == COLOUR_BLUE:
            marker.colors.append(self.Colour_Blue())
            marker.colors.append(self.Colour_Blue())
        elif colour == COLOUR_RED:
            marker.colors.append(self.Colour_Red())
            marker.colors.append(self.Colour_Red())
        elif colour == COLOUR_GREEN:
            marker.colors.append(self.Colour_Green())
            marker.colors.append(self.Colour_Green())
        else:
            marker.colors.append(self.Colour_Yellow())
            marker.colors.append(self.Colour_Yellow())


        return marker


    def CreateKeyPointMarker(self, keyPointID, personID, coordinates):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.message.header.stamp

        marker.ns = "person" + str(personID)
        marker.id = keyPointID

        marker.action = Marker.ADD

        marker.scale.x = 0.07 
        marker.scale.y = 0.07
        marker.scale.z = 0.07

        marker.pose.position = coordinates

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0


        
        ## Based on the key point, Change the colour and shape
        if keyPointID <= HEAD:
            marker.type = Marker.SPHERE
            marker.color = self.Colour_Green()
        elif keyPointID <= ARMS:
            marker.type = Marker.CUBE
            marker.color = self.Colour_Blue()
        else:   # Legs
            marker.type = Marker.CYLINDER
            marker.color = self.Colour_Yellow()


        return marker





    def Colour_Red(self):
        c = ColorRGBA()
        c.a = 1.0
        c.r = 1.0
        c.g = 0.0
        c.b = 0.0
        return c
    
    def Colour_Green(self):
        c = ColorRGBA()
        c.a = 1.0
        c.r = 0.0
        c.g = 1.0
        c.b = 0.0
        return c
    
    def Colour_Blue(self):
        c = ColorRGBA()
        c.a = 1.0
        c.r = 0.0
        c.g = 0.0
        c.b = 1.0
        return c
    
    def Colour_Yellow(self):
        c = ColorRGBA()
        c.a = 1.0
        c.r = 1.0
        c.g = 0.5
        c.b = 0.0 
        return c


'''
Function to generate a vector for person pointing to a target, and attach it to the person detection message.
If the pointing is not generated, the pointing vector will be all 0s.
Parameters:
person: PersonDetection
Return:
None
'''
def pointing_vector_generation(person: PersonDetection):
    left_hand = create_pointing_vector(person.skeleton[7], person.skeleton[9], person.skeleton)
    right_hand = create_pointing_vector(person.skeleton[8], person.skeleton[10], person.skeleton)
    person.right_pointing_vector = right_hand
    person.left_pointing_vector = left_hand
    return person


def create_pointing_vector(
    elbow: Point32,
    wrist: Point32,
    skeleton: List[Point32]
):
    pointing_vector = VisualVector()
    pointing_vector.direction = Vector3()
    pointing_vector.origin = Point()
    # Creating origin for the person pointing (Point32 -> Point) at wrist
    pointing_vector.origin.x = wrist.x
    pointing_vector.origin.y = wrist.y
    pointing_vector.origin.z = wrist.z
    # Creating vectors for the person pointing (wrist - elbow)
    # If one of the vectors is 0.0, there was an invalid reading, so no vector should be created
    # If the person's arm is parallel to one of their upper legs, they should not be pointing, angle less than 10 degrees
    # If a person's arm is pointing close to directly down
    down_vector = Vector3()
    down_vector.x = 0.0
    down_vector.y = 0.0
    down_vector.z = -1.0    

    right_leg = vector_from_points(skeleton[11], skeleton[13])
    left_leg = vector_from_points(skeleton[12], skeleton[14]) 

    if any((coord is None or coord == 0.0) for coord in [wrist.x, wrist.y, wrist.z, elbow.x, elbow.y, elbow.z]):
        pointing_vector.direction = Vector3(0.0, 0.0, 0.0)

    else:
        pointing_vector.direction.x = wrist.x - elbow.x
        pointing_vector.direction.y = wrist.y - elbow.y
        pointing_vector.direction.z = wrist.z - elbow.z
        if any((vector_dir is None or vector_dir == 0.0) for vector_dir in [right_leg.x, right_leg.y, right_leg.z, left_leg.x, left_leg.y, left_leg.z]):
            return pointing_vector
        if vector_angle(pointing_vector.direction, down_vector) < 5 or vector_angle(pointing_vector.direction, right_leg) < 5 or vector_angle(pointing_vector.direction, left_leg) < 5:
            pointing_vector.direction = Vector3(0.0, 0.0, 0.0)
        else:
            rospy.loginfo("Valid pointing vector created")
    return pointing_vector

def create_gaze_direction(skeleton: List[Point32]):

    nose, lefteye, righteye, leftear, rightear = (skeleton[0], skeleton[2], skeleton[1], skeleton[4], skeleton[3])      
    nose = point_to_np_array(nose)
    lefteye = point_to_np_array(lefteye) # doesn't need eyes atm.
    righteye = point_to_np_array(righteye)
    leftear = point_to_np_array(leftear)
    rightear = point_to_np_array(rightear)
    points = [nose, lefteye, righteye, leftear, rightear]        

    # Check we have three points
    havenose = point_exists(skeleton[0]) 
    haveleft = point_exists(skeleton[4]) 
    haveright = point_exists(skeleton[3])
    if not(havenose and haveleft and haveright):
        rospy.loginfo(f"Nose visible? {havenose} Left ear? {haveleft} Rightear? {haveright}")
    non_zero_count = 0
    for point in points:
        if not np.all(np.array(point) == 0):
            non_zero_count += 1
    if non_zero_count < 3:
        rospy.loginfo("not enough points to do gaze.")
        return Vector3(0.0,0.0,0.0)

    # take two ears and a nose/eye. TODO can be other combination, can be more robust
    leftear2ear = rightear - leftear
    leftear2nose = nose - rightear
    floorPlaneNormal = np.cross(leftear2ear, leftear2nose) # this normal gets the "vertical" of the head

    # now take eye 2 eye
    # lefteye2eye = righteye - lefteye
    # face plane's normal, made by spanning the "vertical" vector and the "horizontal" vector:
    facePlaneNormal = np.cross(leftear2ear, floorPlaneNormal)

    #finally make sure it's going forward not backward. 
    # expect ear to nose to be going somewhat forward:
    ear2nose = nose-leftear
    if np.dot(ear2nose, facePlaneNormal) < 0:
        # need to flip the vector
        # consider unsw_person_recognition.py for face directions.
        facePlaneNormal = -1 * facePlaneNormal

    # normalise
    facePlaneNormal = facePlaneNormal / np.linalg.norm(facePlaneNormal)
    # create and return
    gaze_vector = Vector3()
    gaze_vector.x = facePlaneNormal[0]
    gaze_vector.y = facePlaneNormal[1]
    gaze_vector.z = facePlaneNormal[2]
    return gaze_vector

def create_gaze_direction_2(skeleton: List[Point32]):
    v = Vector3()
    nose, lefteye, righteye, leftear, rightear = (skeleton[0], skeleton[2], skeleton[1], skeleton[4], skeleton[3])      

    # only get valid points.
    points = list(filter(lambda x: (point_exists(x)) , [nose, lefteye, righteye, leftear, rightear]))
    floorPlaneNormal = np.array([0,0,1])
    if len(points) < 3:            
        rospy.loginfo("Not enough points to create floor plane for gaze, using default")
    else:    
        points_np = list(map(lambda x: (point_to_np_array(x)) , points))
        #get the floor plane.
        x_vals = np.array([p[0] for p in points_np]) # 'function' object is not subscriptable.
        y_vals = np.array([p[1] for p in points_np])
        z_vals = np.array([p[2] for p in points_np])
        X = np.column_stack((x_vals, y_vals))
        model = LinearRegression()
        model.fit(X, z_vals)
        A, B = model.coef_
        C = -1
        D = model.intercept_
        floorPlaneNormal = np.array([A,B,C])
    # tweak floor plane normal to point upwards:
    if np.dot(floorPlaneNormal, np.array([0,0,1])) < 0:
        floorPlaneNormal = floorPlaneNormal * -1

    # line across face.
    line_across_face = np.array([1.0,0.0,0.0]) # meaningless vector
    if point_exists(lefteye) and point_exists(righteye):
        line_across_face = point_to_np_array(lefteye) - point_to_np_array(righteye)
    elif point_exists(leftear) and point_exists(rightear):
        line_across_face = point_to_np_array(leftear) - point_to_np_array(rightear)
    else:
        rospy.loginfo("Unable to create line across face.")

    # gaze vector:
    v = np.cross(line_across_face, floorPlaneNormal)

    #check and tweak direction (backward to forward)
    ear2nose = None
    if point_exists(leftear):
        ear2nose = point_to_np_array(nose)-point_to_np_array(leftear) #TODO fix this to stop calling so much.
    elif point_exists(rightear):
        ear2nose = point_to_np_array(nose)-point_to_np_array(rightear)
    else:
        rospy.loginfo("unable to find a forwardish vector.")
    if ear2nose is not None:
        if np.dot(ear2nose, v) < 0:
            # need to flip the vector
            # consider unsw_person_recognition.py for face directions.
            v = -1 * v

    if (v == np.array([0.0,0.0,0.0])).all():
        rospy.loginfo("Gaze vector is all 0s")
    else:
        # normalise
        v = v / np.linalg.norm(v)
    # put into message
    gaze_vector = Vector3()
    gaze_vector.x = v[0]
    gaze_vector.y = v[1]
    gaze_vector.z = v[2]
    return gaze_vector

    
def create_gaze_origin(skeleton: List[Point32]):
    # use eyes midpoint
    point = Point()
    if point_exists(skeleton[2]) and point_exists(skeleton[1]):
        point.x =(skeleton[2].x + skeleton[1].x)/2
        point.y =(skeleton[2].y + skeleton[1].y)/2
        point.z =(skeleton[2].z + skeleton[1].z)/2
        return point
    # use ears midpoint
    if point_exists(skeleton[4]) and point_exists(skeleton[3]):
        point.x =(skeleton[4].x + skeleton[3].x)/2
        point.y =(skeleton[4].y + skeleton[3].y)/2
        point.z =(skeleton[4].z + skeleton[3].z)/2
        return point
    return skeleton[0]


def point_to_np_array(point: Point32):
    return np.array([point.x, point.y, point.z])
    
def point_exists(point: Point32):
    return Point.x != 0 or Point.y != 0 or Point.z != 0

def vector_angle(
    vector_a: Vector3,
    vector_b: Vector3,
):
    # cos-1(a.b/|a||b|)
    # unit_a = vector_a / np.linalg.norm(vector_a)
    unit_a_factor = np.linalg.norm(np.array([vector_a.x, vector_a.y, vector_a.z]))
    unit_b_factor = np.linalg.norm(np.array([vector_b.x, vector_b.y, vector_b.z]))
    unit_a = np.array([vector_a.x / unit_a_factor, vector_a.y / unit_a_factor, vector_a.z / unit_a_factor])
    unit_b = np.array([vector_b.x / unit_b_factor, vector_b.y / unit_b_factor, vector_b.z / unit_b_factor])
    # unit_a.x = vector_a.x / unit_a_factor
    # unit_a.y = vector_a.y / unit_a_factor
    # unit_a.z = vector_a.z / unit_a_factor
    # unit_b.x = vector_b.x / unit_b_factor
    # unit_b.y = vector_b.y / unit_b_factor
    # unit_b.z = vector_b.z / unit_b_factor
    
    angle = np.degrees(np.arccos(np.clip(np.dot(unit_a, unit_b), -1.0, 1.0)))

    return angle

def vector_from_points(
    point_a: Union[Point32, Point], # from
    point_b: Union[Point32, Point] # to
):
    vector = Vector3()
    vector.x = point_b.x - point_a.x
    vector.y = point_b.y - point_a.y
    vector.z = point_b.z- point_a.z
    return vector

if __name__ == '__main__':

    visualiser = PeopleVisualiser()
    rospy.spin()
        




    
