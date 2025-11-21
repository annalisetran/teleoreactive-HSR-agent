#!/usr/bin/env python3

import rospy
import sys
from unsw_vision_msgs.msg import DetectionList, ObjectDetection
from diagnostic_msgs.msg import KeyValue
import numpy as np
from scipy.spatial import ConvexHull
# from pyrr import Matrix44

'''  
OLD
           +
      E --F
     /   /|   
    A--B/ |    (G is bottom left hidden)
    |  | /H
    C--D/

-
    
'''
'''  
           +
      3 --7
     /   /|   
    1--5/ |    (2 is bottom left hidden). x is <------->
    |  | /6
    0--4/

-
    
'''

TAKEN_THRESHOLD = 0.99
@DeprecationWarning
def rotateVector(vector, rotation): #vector must be shape 1x3, will get reshaped.
    # x = vector[0]
    # y = vector[1]
    # z = vector[2]
    vector = vector.reshape((3,1))
    r = rotation[0]
    p = rotation[1]
    yaw = rotation[2]

    # assume associate x with r, y with p and z with yaw
    # x y z represent a vector, not a point.
    xtheta = r # TODO figure out theta from roll. Need to be in radians.
    ytheta = p
    ztheta = yaw
    rollMatrix = np.array([
        [1, 0, 0],
        [0, np.cos(xtheta), 0],
        [0, np.sin(xtheta), np.cos(xtheta)],
    ])
    pitchMatrix = np.array([
        [np.cos(ytheta), 0, np.sin(ytheta)],
        [0, 1, 0],
        [-np.sin(ytheta), 0, np.cos(ytheta)],
    ])
    yawMatrix = np.array([
        [np.cos(ztheta), -np.sin(ztheta), 0],
        [np.sin(ztheta), np.cos(ztheta), 0],
        [0, 0, 1],
    ])

    v = np.matmul(np.matmul(np.matmul(rollMatrix, vector), pitchMatrix), yawMatrix)
    return v
@DeprecationWarning
def getRotationMatrix(rotation):
    r = rotation[0]
    p = rotation[1]
    yaw = rotation[2]

    # assume associate x with r, y with p and z with yaw
    # x y z represent a vector, not a point.
    xtheta = r # TODO figure out theta from roll. Need to be in radians.
    ytheta = p
    ztheta = yaw
    rollMatrix = np.array([
        [1, 0, 0],
        [0, np.cos(xtheta), 0],
        [0, np.sin(xtheta), np.cos(xtheta)],
    ])
    pitchMatrix = np.array([
        [np.cos(ytheta), 0, np.sin(ytheta)],
        [0, 1, 0],
        [-np.sin(ytheta), 0, np.cos(ytheta)],
    ])
    yawMatrix = np.array([
        [np.cos(ztheta), -np.sin(ztheta), 0],
        [np.sin(ztheta), np.cos(ztheta), 0],
        [0, 0, 1],
    ])

    m = np.matmul(np.matmul(rollMatrix, pitchMatrix), yawMatrix)
    return m
@DeprecationWarning
def get_vertices(center, dimensions, rotation_matrix):
    """Calculate the 8 vertices (ordered using the diagram at the top) of a rectangular prism given its center, dimensions, and rotation."""
    hx, hy, hz = dimensions / 2  # Half dimensions

    # Define the 8 vertices relative to the center
    vertices = np.array([
        [-hx, -hy, -hz], [-hx, -hy, hz], [-hx, hy, -hz], [-hx, hy, hz],
        [hx, -hy, -hz], [hx, -hy, hz], [hx, hy, -hz], [hx, hy, hz]
    ])

    # Apply rotation and translation
    rotated_vertices = np.dot(vertices, rotation_matrix.T) + center
    return rotated_vertices

def get_vertices(center, dimensions):
    """Calculate the 8 vertices (ordered using the diagram at the top) of a rectangular prism given its center, dimensions, and rotation."""
    hx, hy, hz = dimensions / 2  # Half dimensions

    # Define the 8 vertices relative to the center
    vertices = np.array([
        [-hx, -hy, -hz], [-hx, -hy, hz], [-hx, hy, -hz], [-hx, hy, hz],
        [hx, -hy, -hz], [hx, -hy, hz], [hx, hy, -hz], [hx, hy, hz]
    ])

    return vertices + center

# By ChatGPT and checked.
@DeprecationWarning
def get_planes(vertices):
    """Given 8 vertices (ordered by the diagram at the top) of a rectangular prism, return 6 plane equations (Ax + By + Cz + D = 0)."""
    planes = []
    # Define the 6 faces by only 3 vertices that form each face
    face_indices = [
        [0, 1, 3],  # -x face
        [4, 5, 7],  # +x face
        [0, 1, 5],  # -y face
        [2, 3, 7],  # +y face
        [0, 2, 4],  # -z face
        [1, 3, 5]   # +z face
    ]
    
    for face in face_indices:
        # Select three vertices on the face
        v0, v1, v2 = vertices[face[0]], vertices[face[1]], vertices[face[2]]
        
        # Compute two vectors on the plane
        vec1 = v1 - v0
        vec2 = v2 - v0

        # Compute the normal vector using the cross product
        normal = np.cross(vec1, vec2)
        normal = normal / np.linalg.norm(normal)  # Normalize the normal vector

        # Calculate D for the plane equation Ax + By + Cz + D = 0
        D = -np.dot(normal, v0)

        # Append the plane equation as [A, B, C, D]
        planes.append(np.append(normal, D))
    
    return np.array(planes)

@DeprecationWarning
def intersect_planes(planes):
    """Find vertices that satisfy all plane inequalities and form the intersection polyhedron."""
    # written by Chat GPT
    vertices = []

    # Try combinations of three planes to find intersection points
    for i in range(len(planes)):
        for j in range(i + 1, len(planes)):
            for k in range(j + 1, len(planes)):
                normals = planes[[i, j, k], :3]  # select plane i, j and k, and only select A B and C values.
                if np.linalg.matrix_rank(normals) == 3:  # Planes are not parallel if their equations are linearly independent.
                    # Solve for intersection point of these three planes
                    D = -planes[[i, j, k], 3]
                    intersection = np.linalg.solve(normals, D)
                    
                    # Check if the intersection point satisfies all planes (i.e., inside the polyhedron)
                    if all(np.dot(plane[:3], intersection) + plane[3] <= 1e-6 for plane in planes):
                        vertices.append(intersection)
    
    return np.array(vertices)

# def seatEmptyValidation(object: ObjectDetection, detectionList: DetectionList):
@DeprecationWarning
def seatEmptyValidation(object , detectionList):
    chairbox = object.bbox3d
    centreObjC = chairbox.center
    centreChair = np.array([centreObjC.position.x, centreObjC.position.y, centreObjC.position.z])
    orientationChair = np.array([centreObjC.quaternion.x, centreObjC.quaternion.y, centreObjC.quaternion.z])
    sizeChair = np.array([chairbox.size.x, chairbox.size.y, chairbox.size.z]) 
    
    verticesChair = get_vertices(centreChair, sizeChair, orientationChair)
    planesChair = get_planes(verticesChair)
    volumeChair = chairbox.size.x * chairbox.size.y * chairbox.size.z

    isTaken = False
    people = detectionList.people
    for person in people:
        box = person.bbox3d
        center = np.array([person.position.x, person.position.y, person.position.z])
        orientation = np.array([box.centreP.orientation.x, box.centreP.orientation.y, box.centreP.orientation.z]) #NOTE ignoring w.
        size = box.size
        vertices = get_vertices(center, np.array([size.x, size.y, size.z]), getRotationMatrix(orientation))
        #rotated_vertices = np.dot(vertices, getRotationMatrix(orientation))
        # Get planes defining both chair and person
        planesPerson = get_planes(vertices)
        all_planes = np.vstack((planesPerson, planesChair))

         # Find intersection vertices that satisfy all plane constraints
        intersection_vertices = intersect_planes(all_planes)

        # If we have enough points to define a volume
        if len(intersection_vertices) >= 4:
            hull = ConvexHull(intersection_vertices)
            if hull.volume / volumeChair > 1 or hull.volume/volumeChair < 0:
                rospy.loginfo("ERROR: HULL VOLUME INVALID") #NOTE this is a logic check and can eventually be removed.
            if hull.volume / volumeChair > TAKEN_THRESHOLD:
                return True
        # otherwise, assume the person and chair box did not intersect. Check for the next person.
    return isTaken

def seatEmptyValidationSimple(object: ObjectDetection, detectionList: DetectionList):
    #def seatEmptyValidationSimple(object, detectionList):
    chairbox = object.bbox3d
    centreObjC = chairbox.center
    centreChair = np.array([centreObjC.position.x, centreObjC.position.y, centreObjC.position.z])
    sizeChair = np.array([chairbox.size.x, chairbox.size.y, chairbox.size.z]) 
    chairVertices = get_vertices(centreChair, sizeChair)
    vertex0Chair = chairVertices[0]
    volumeChair = chairbox.size.x * chairbox.size.y * chairbox.size.z
    isTaken = False
    if volumeChair == 0:
        return isTaken
    
    people = detectionList.people
    for person in people:
        box = person.bbox3d
        center = np.array([box.center.position.x, box.center.position.y, box.center.position.z])
        size = np.array([box.size.x, box.size.y, box.size.z]) 
        vertices = get_vertices(center, size)
        vertex0 = vertices[0]
        # (0, 5)  (3, 9) => 2 from 5-3.
        # (0, 5) (6, 7) => 0 from 5-6
        # (0,5) (2,4) => 2 from 4-2  min right - max left.
        # (0,5) (-1, 1) => 1 from 1-0
        # (0,5) ( -2, -1) => 
        intersectingW = max(min(chairVertices[4][0], vertices[4][0]) - max(chairVertices[0][0], vertices[0][0]), 0)
        intersectingD = max(min(chairVertices[2][1], vertices[2][1]) - max(chairVertices[0][1], vertices[0][1]), 0) # Problem, always 0
        intersectingH = max(min(chairVertices[1][2], vertices[1][2]) - max(chairVertices[0][2], vertices[0][2]), 0)
        print(f'chair pos y {chairVertices[2][1]} chair neg y {chairVertices[0][1]} person pos y {vertices[2][1]} person neg y {vertices[0][1]}')

    

        # If we have enough points to define a volume
        intersectV = intersectingD * intersectingH * intersectingW
        print(f'Intersect Width {intersectingW}, Intersect Depth {intersectingD}, Intersect Height {intersectingH}')
        print(f'Volume chair {volumeChair}')
        print(f'Intersecting volume: {intersectV}')
        if intersectV / volumeChair > 1:
            rospy.loginfo("ERROR: HULL VOLUME INVALID") #NOTE this is a logic check and can eventually be removed.
        if intersectV / volumeChair > TAKEN_THRESHOLD:
            return True
        # otherwise, assume the person and chair box did not intersect. Check for the next person.
    return isTaken

##### JUST FOR MANUAL TESTING ########################################

class Position:
    def __init__(self, x=1, y=1, z=1):
        self.x = x
        self.y = y
        self.z = z


class Orientation:
    def __init__(self, x=1, y=1, z=1):
        self.x = x
        self.y = y
        self.z = z


class Size:
    def __init__(self, x=1, y=1, z=1):
        self.x = x
        self.y = y
        self.z = z


class Center:
    def __init__(self, position=None, orientation=None):
        self.position = position if position else Position()
        self.orientation = orientation if orientation else Orientation()


class BBox3D:
    def __init__(self, center=None, size=None):
        self.center = center if center else Center()
        self.size = size if size else Size()


class myObj:
    def __init__(self, bbox3d=None):
        self.bbox3d = bbox3d if bbox3d else BBox3D()

class DetectionList:
    def __init__(self, list=None):
        self.people = list


if __name__ == '__main__':
    # Custom chair
    custom_center = Center(Position(1, 1, 1), Orientation(0, 0, 0))
    custom_size = Size(1, 1, 1)
    custom_bbox3d = BBox3D(custom_center, custom_size)
    chair = myObj(custom_bbox3d)
    # p1
    custom_center = Center(Position(0, 0, 0), Orientation(0, 0, 0))
    custom_size = Size(1.1, 1.1, 1.1)
    custom_bbox3d = BBox3D(custom_center, custom_size)
    person1 = myObj(custom_bbox3d)
    # p1
    custom_center = Center(Position(0, 0, 0), Orientation(0, 0, 0))
    custom_size = Size(1, 1, 1)
    custom_bbox3d = BBox3D(custom_center, custom_size)
    person1 = myObj(custom_bbox3d)
    # p2
    custom_center = Center(Position(1, 1, 1), Orientation(0, 0, 0))
    custom_size = Size(1, 1, 1)
    custom_bbox3d = BBox3D(custom_center, custom_size)
    person2 = myObj(custom_bbox3d)

    print(seatEmptyValidationSimple(chair, DetectionList(list = [person1, person2])))