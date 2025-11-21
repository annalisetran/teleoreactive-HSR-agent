#! /usr/bin/env python3
# import doors
# see if any doors are in front of where the robot - /amcl_pose /hsrb_base_pose
# check expected distance
# measure distance shown on lidar

import rospy
from math import atan2, pi, tan, sqrt
from utility.msg import DBDoor, DoorList
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Int8
from nav_msgs.msg import Odometry
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_multiply

DOOR_LENGTH = 1.5
ACCURACY = 0.05


class DoorChecker:
    def __init__(self) -> None:
        self.doors_sub = rospy.Subscriber('door_list', DoorList, callback=self.doorListCallback, queue_size=10)
        self.laser_scan_sub = rospy.Subscriber('/hsrb/base_scan', LaserScan, callback=self.laserScanCallBack, queue_size=10)
        # self.amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback=self.amclCallBack, queue_size=10)
        self.pose_sub = rospy.Subscriber('/hsrb/odom', Odometry, callback=self.poseCallBack, queue_size= 10)
        self.open_pub = rospy.Publisher('/door_open_close', String, queue_size=10)
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.door_list = None
        self.pose = None
        self.scan = None
        self.amcl_pose = None

        self.wrist_sub = rospy.Subscriber('/start_task', Int8, callback=self.wrist_callback, queue_size=10)
        self.start = False
    def wrist_callback(self, msg:Int8):
        self.start = True

    def doorListCallback(self, msg: DoorList):
        # print("got door")
        self.door_list = msg
    
    # called when recieving the odom message
    def poseCallBack(self, msg: Odometry):
        
        self.pose = msg.pose.pose

    # called when recieving laser scan message
    def laserScanCallBack(self, msg: LaserScan):
        self.scan = msg.ranges[len(msg.ranges) // 2]

    # def amclCallBack(self, msg: PoseWithCovarianceStamped):
    #     self.amcl_pose = msg.pose.pose

    def checkDoors(self):
        # print("checking doors")
        if (not self.start):
            print("waiting for press the wrist")
            return
        if self.door_list == None or self.pose == None: 
            print("no pose or no door")
            return
        try:
            
            # print(f"ogPose x: {self.pose.position.x}, y: {self.pose.position.y}, z: {self.pose.position.z}")
            # print(f"og orient, x: {self.pose.orientation.x}, y: {self.pose.orientation.y}, z: {self.pose.orientation.z}, w: {self.pose.orientation.w}")
            trans = self.buffer.lookup_transform('map', 'odom', rospy.Time(0))
            # print(trans)
            
            self.pose.position.x += trans.transform.translation.x
            self.pose.position.y += trans.transform.translation.y
            self.pose.position.z += trans.transform.translation.z

            # quaternion_multiply uses two arrays of numbers instead of a quaternion object
            orient_arr = [self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w]
            rotation_arr = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
            trans_arr = quaternion_multiply(rotation_arr, orient_arr)
            self.pose.orientation = Quaternion(x = trans_arr[0], y=trans_arr[1], z=trans_arr[2], w=trans_arr[3])
            print(f"transform Pose x: {self.pose.position.x}, y: {self.pose.position.y}, z: {self.pose.position.z}")
            # print(f"transform  orient, x: {self.pose.orientation.x}, y: {self.pose.orientation.y}, z: {self.pose.orientation.z}, w: {self.pose.orientation.w}")
            closestDoor = self.findInterceptingDoor()
            if closestDoor == None:
                print(" no closest door :(")
                return
                
            print("actually checking doors")
            distance = self.twoPointDistance(self.pose.position.x, self.pose.position.y, closestDoor[1][0], closestDoor[1][1])
            # rospy.loginfo("dist: ",distance," scan: ",self.scan)
            if (self.scan < distance - ACCURACY):
                rospy.loginfo(f"DOOR CLOSED: {closestDoor[0].name}")
            else:
                rospy.loginfo(f"DOOR OPEN: {closestDoor[0].name}")
                self.open_pub.publish(closestDoor[0].name)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException):
            print("could not find transform")
    
    # returns an array where the first element is the name of the door, and the second
    # element is the point of interception with the lidar and the door.
    def findInterceptingDoor(self) -> DBDoor:
        for door in self.door_list.doors:
            euler = self.orientToEuler(door.pose.orientation)
            rotatedYaw = euler[2] - pi/2
            robotYaw = self.orientToEuler(self.pose.orientation)[2]
            interception = self.findInterception(
                self.pose.position.x, self.pose.position.y, robotYaw, 
                door.pose.position.x, door.pose.position.y, rotatedYaw
            )
            if self.twoPointDistance(interception[0], interception[1], door.pose.position.x, door.pose.position.y) < DOOR_LENGTH / 2:
                return [door, interception]
        return None
            
    def twoPointDistance(self, x1, y1, x2, y2):
        return sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
    
    def findInterception(self, x1, y1, yaw1, x2, y2, yaw2):
        x = ((tan(yaw1) * x1 - tan(yaw2) * x2) + y2 - y1) / (tan(yaw1) - tan(yaw2))
        y = tan(yaw1) * (x - x1) + y1
        return [x, y]

    def orientToEuler(self, orientation):
        return euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    

    def printPose(self, pose):
        print("Pose: x: ",pose.position.x,", y: ",pose.position.y," z: ",pose.position.z)
        print("rotation: x: ",pose.orientation.x,", y: ",pose.orientation.y," z: ",pose.orientation.z," w: ",pose.orientation.w)
    
    def doorOpen(self):
        self.open_pub.publish("open")
    
if __name__ == "__main__":
    rospy.init_node('DoorChecker')
    doorCheckerNode = DoorChecker()
    # doorCheckerNode.listener.waitForTransform("/odom", "/map", rospy.Time(), rospy.Duration(4))
    while not rospy.is_shutdown(): 
        
        doorCheckerNode.checkDoors()
        rospy.sleep(2)
