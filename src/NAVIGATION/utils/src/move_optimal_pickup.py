#!/usr/bin/env python3

import rospy
import message_filters
import sys
sys.path.append("/home/robocupathome/workspace/robocup_ws/src/world_model/world_model_kb/scripts")
from dbase import DBase
from object_api import ObjectApi

from unsw_vision_msgs.msg import DetectionList
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, PointStamped, Transform
import json
import numpy as np

import tf2_ros
import math

from geometry_msgs.msg import PoseStamped, PointStamped, PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalID

class moveOptimal:
    def __init__(self):
        self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        self.movePub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.object_class = rospy.get_param("/move_optimal_pickup/object_class")
        self.optimalPos = None
        self.not_update_attempt_num = 0
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        laser_topic = rospy.get_param("/move_optimal_pickup/laser_topic")
        detection_topic = rospy.get_param("/move_optimal_pickup/detection_topic")
        
        # SETUP SUBSCRIBERS AND CALLBACKS
        laser_sub = message_filters.Subscriber(laser_topic, LaserScan)
        detection_sub = message_filters.Subscriber(detection_topic,DetectionList)
        ts = message_filters.ApproximateTimeSynchronizer([laser_sub,detection_sub],10,0.2)
        ts.registerCallback(self.mergecallback)
        
        

        
        object_loc = Point()
        print(self.object_class)
        obj_api = ObjectApi()
        db = DBase("/home/robocupathome/workspace/robocup_ws/src/world_model/world_model_kb/scripts/database.ini")
        obj = obj_api.get_object_by_class(db.con_pool,self.object_class)[0]
        object_loc.x = obj["loc_x"]
        object_loc.y = obj["loc_y"]
        
        object_loc.x = 0
        object_loc.y = 0
        # self.move(object_loc, 'base_range_sensor_link')
        self.move(object_loc, 'map')
        
    def move(self, waypoint: Point, frame_id: str):
        rospy.sleep(0.3)
        msg = GoalID()
        msg.stamp = rospy.Time.now()
        msg.id = "-1"
        self.cancel_pub.publish(msg)
        
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = frame_id
        goal.pose.position.x = 1.3
        goal.pose.position.y = -0.73
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1
        self.movePub.publish(goal)
        
        print(f"I should move to {goal}")
        
    def do_transformation(self, header, point:Point, to_frame):
        point_stamped = PointStamped()
        point_stamped.header = header
        point_stamped.point = point
        try:
            transformed_point_stamped = self.tfBuffer.transform(point_stamped,to_frame)
        except Exception as e:
            rospy.loginfo(f"Optimal pickup failed to do transformation {e}")
            return None
        return transformed_point_stamped.point
    def mergecallback(self, laser: LaserScan, detection_list: DetectionList):
        
        filtered_objects = [obj for obj in detection_list.objects if obj.name == self.object_class]
        if len(filtered_objects) == 0:
            rospy.loginfo("I don't have the object in my view")
            return
        # transform the object to robot frametyE
        object_pos = self.do_transformation(DetectionList.header, filtered_objects[0].position)
        if object_pos == None:
            return

        closest_edge_point = Point()
        # any point half meters away from the objects will be igonored
        closest_dist_sqaured = 0.25
        for i, range_value in enumerate(laser.ranges):
            angle = laser.angle_min + i*laser.angle_increment
            edge_point = Point()
            edge_point.x = range_value * math.cos(angle)
            edge_point.y = range_value * math.sin(angle)
            distsquared = (edge_point.x - object_pos.x)**2 + (edge_point.y - object_pos.y)
            if distsquared < closest_dist_sqaured:
                closest_dist_sqaured = distsquared
                closest_edge_point = edge_point
            
        # If we have a closest edge point, find new optimal point, otherwise abort
        if closest_dist_sqaured == 0.25:
            rospy.loginfo("No closest point of edge is in the view")
            return
        
        optimal = Point()
        # optimal point is 40cm away from edge. TODO: make this into parameter
        optimal.x = edge_point.x + (edge_point.x - object_pos.x)/math.sqrt(closest_dist_sqaured)*0.4
        optimal.y = edge_point.y + (edge_point.y - object_pos.y)/math.sqrt(closest_dist_sqaured)*0.4
        
        if self.optimalPos == None:
            self.optimalPos = optimal
            self.not_update_attempt_num += 1
        else:
            # only update if big change, and trigger new goal
            if ((optimal.x - self.optimalPos.x)**2 + (optimal.y - self.optimalPos.y)**2 >= 0.1*0.1):
                self.optimalPos = optimal
                # drive to new goal
                self.not_update_attempt_num = 0
                #self.do_transformation()
        if self.not_update_attempt_num == 16:
            rospy.loginfo("Im finished")  
        

def main(args):
  rospy.init_node('move_optimal_pickup', anonymous=True)
  obc = moveOptimal()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
