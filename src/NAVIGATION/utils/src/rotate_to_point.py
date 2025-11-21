#! /usr/bin/env python
import json
import math
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Int8
from geometry_msgs.msg import PoseStamped, Point, Twist
from nav_msgs.msg import Odometry
import sys
import os

import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from manip_srv.srv import rotate_to_point

module_path = os.environ.get("UNSW_WS")
sys.path.append(module_path + "/WORLD_MODEL/world_model/world_model_kb/scripts")
from regions_api import RegionsApi
from object_api import ObjectApi

class Rotate_to_point:

    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.past_pose = None
        self.pose_in_map = None
        self.odom_sub = rospy.Subscriber('/hsrb/odom', Odometry, callback=self.odom_callback, queue_size=1)
        rospy.Service('/unsw/nav_utils/rotate_to_point', rotate_to_point, self.rotate_to_point)

    def odom_callback(self, msg: Odometry):
            self.past_pose = msg.pose.pose
                
            myPoseStamp = PoseStamped()
            myPoseStamp.header = msg.header
            myPoseStamp.pose = msg.pose.pose
            print("callback")
            if self.pose_in_map == None:
                try:
                    trans = self.tfBuffer.lookup_transform("map",msg.header.frame_id,rospy.Time.now(), rospy.Duration(2))
                    self.pose_in_map = tf2_geometry_msgs.do_transform_pose(myPoseStamp, trans)
                except Exception as e:
                    rospy.loginfo(f"Rotate to point door check failed to do transformation {e}")
    
    def rotate_to_point(self, data):
        if self.pose_in_map == None: return False
        x = data.point.x - self.pose_in_map.position.x
        y = data.point.y - self.pose_in_map.position.y

        vector_angle = math.atan(y, x)
        (roll, pitch, current_yaw) = euler_from_quaternion(self.pose_in_map.orientation)
        angles = [current_yaw - vector_angle, (current_yaw-360) - vector_angle, current_yaw - (vector_angle-360)]
        difference = min(map(abs, angles))
        if (difference < 0.4):
            # service that makes robot face door
            return True
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        angular_difference = filter((abs(x)==difference), angles)[0]
        r=rospy.Rate(4)
        speed = Twist()
        i = 0
        while not rospy.is_shutdown():
            if (difference <= 0.4): return True
            if (difference < 0.8):
                speed.angular.z = difference * (angular_difference/abs(angular_difference))
            else: speed.angular.z = 1 * (angular_difference/abs(angular_difference))
            speed.linear.x, speed.linear.y, speed.linear.z = 0
            pub.publish(speed)
            difference = difference - abs(speed.angular.z)
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('Rotate_to_point')
    Rotate_to_point()
    while not rospy.is_shutdown():
        rospy.sleep(1)