#!/usr/bin/env python3

# Person vector generation
# Help function to generate a vectors for person pointing and gazing.
# Dependencies:
# unsw_vision_msgs/VisualVector.msg
# unsw_vision_msgs/PersonDetection.msg
# Written By: Zijie Li, 


import rospy
from unsw_vision_msgs.msg import PersonDetection, VisualVector
from geometry_msgs.msg import Point, Point32, Vector3
import numpy as np
from typing import List, Union
from sklearn.linear_model import LinearRegression

'''
Function to generate a vector for person gaze to a target, and attach it to the person detection message.
If the gaze is not generated, the gaze vector will be all 0s.
Parameters:
person: PersonDetection
Return:
None
'''
def gaze_vector_generation(person: PersonDetection):
    # NOTE:REMEMBER TO HANDLE THE CASE WHEN SOME SKELTON ARE NOT DETECTED OR 3D POINTS ARE NOT AVAILABLE
    # NOTE:USUALLY IT IS ALL 0s WHEN NOT DETECTED OR NOT AVAILABLE
    vec = VisualVector()
    vec.origin = create_gaze_origin(person.skeleton)
    
    person.gaze_vector = vec
    # Not enough points, returns empty vector direction
    if not point_exists(vec.origin):
        return person
    
    # Only one side of the face detected
    if (point_exists(person.skeleton[0]) and point_exists(person.skeleton[4]) and point_exists(person.skeleton[2]) and not point_exists(person.skeleton[1]) and not point_exists(person.skeleton[3])):
        vec.direction = create_gaze_direction_one_side(person.skeleton[4], person.skeleton[2], person.skeleton[0])
    elif (point_exists(person.skeleton[0]) and point_exists(person.skeleton[1]) and point_exists(person.skeleton[3]) and not point_exists(person.skeleton[4]) and not point_exists(person.skeleton[2])):
        vec.direction = create_gaze_direction_one_side(person.skeleton[3], person.skeleton[1], person.skeleton[0])
    else:
        vec.direction = create_gaze_direction_2(person.skeleton)
    
    return person

def gaze_vector_generation_2(person: PersonDetection):
    vec = VisualVector()
    vec.origin = create_gaze_origin(person.skeleton)
    person.gaze_vector = vec

    # Not enough points, returns empty vector direction
    if not point_exists(vec.origin):
        return person
    
    # Only one side of the face detected
    if (point_exists(person.skeleton[0]) and point_exists(person.skeleton[4]) and point_exists(person.skeleton[2]) and (not point_exists(person.skeleton[1]) or not point_exists(person.skeleton[3]))):
        vec.direction = create_gaze_direction_one_side(person.skeleton[4], person.skeleton[2], person.skeleton[0])
    elif (point_exists(person.skeleton[0]) and point_exists(person.skeleton[1]) and point_exists(person.skeleton[3]) and (not point_exists(person.skeleton[4]) or not point_exists(person.skeleton[2]))):
        vec.direction = create_gaze_direction_one_side(person.skeleton[3], person.skeleton[1], person.skeleton[0])
    else:
        vec.direction = create_gaze_direction_2(person.skeleton)
    
    return person

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


# def test_gaze_vector_generation():
#     person = PersonDetection()
#     person.skleton = [] ## TODO: this is just a placeholder, you need to put some data in it. Need to grab from actual robot
#     gaze_vector_generation(person)

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


    eye2eye = righteye - lefteye
    eye2nose = nose - lefteye
    earnosenormal = np.cross(eye2eye, eye2nose)
    earnosenormal = earnosenormal / np.linalg.norm(earnosenormal)

    # create and return
    gaze_vector = Vector3()
    gaze_vector.x = (facePlaneNormal[0] + earnosenormal[0])/2
    gaze_vector.y = (facePlaneNormal[1] + earnosenormal[1])/2
    gaze_vector.z = (facePlaneNormal[2] + earnosenormal[2])/2
    return gaze_vector

'''
This function gets the gaze vector only using the position of one ear, one eye and the nose.
The resulting vector is then returned.
Params: ear,eye,nose
Return: gaze_vector
'''
def create_gaze_direction_one_side(ear: Point32, eye: Point32, nose: Point32):
    # get eye to nose and eye to ear vector and cross product
    ear = point_to_np_array(ear)
    eye = point_to_np_array(eye)
    nose = point_to_np_array(nose)
    nose_eye = nose - eye
    ear_eye = eye - ear
    perpendicular = -1 * np.cross(nose_eye, ear_eye)
    # normalise
    if np.linalg.norm(perpendicular) != 0:
        perpendicular = perpendicular / np.linalg.norm(perpendicular)

    # create vector and return
    gaze_vector = Vector3()
    gaze_vector.x = perpendicular[0]
    gaze_vector.y = perpendicular[1]
    gaze_vector.z = perpendicular[2]
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
    gaze_vector.z = v[2] + 0.3 # add a little bit to make it look more natural.
    return gaze_vector


'''
This function checks if specific points in the skeleton are found or not. If either the eyes or ears are found,
they are returned. Otherwise the value of the nose is returned.
Params: skeleton
Returns: point
'''    
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

'''
Helper function to convert a point to an np array
It returns the np array
Params: point
Returns: array
'''
def point_to_np_array(point: Point32):
    return np.array([point.x, point.y, point.z])

'''
Helper function that checks if the point exists
If a point does not exist, then the x, y and z values will all be 0
Params: point
Returns: boolean
'''
def point_exists(point: Point32):
    return point.x != 0 or point.y != 0 or point.z != 0

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
