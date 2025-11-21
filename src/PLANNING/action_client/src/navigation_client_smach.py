#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import tf2_ros
import actionlib

import math
import json
import random

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String, Bool
from unsw_action_msg.msg import UNSWActionMsg, UNSWActionResult
from nav_msgs.msg import OccupancyGrid

import re
import sys
import os
module_path = os.environ.get("UNSW_WS")
sys.path.append(module_path + "/WORLD_MODEL/world_model/world_model_kb/scripts")
from dbase import DBase
from regions_api import RegionsApi

class WaitForAction(smach.State) :
    def __init__(self) :
        smach.State.__init__(self, outcomes=['goto_goal_action', 'goto_region_action', 'waiting'], input_keys=['data'], output_keys=['data'])
        action_subcriber = rospy.Subscriber("/actions", UNSWActionMsg , self.action_cb, queue_size=1)
        self.action = None
        self.data = None

    def action_cb(self, msg: UNSWActionMsg):
        self.data = json.loads(f"{msg.data}")
        if msg.action_name == "goto_goal" :
            self.action = "goto_goal_action"
        elif msg.action_name == "goto_region":
            self.action = "goto_region_action"
        elif msg.action_name == "follow_person":
            self.action = "follow_person_action"

    def execute(self, ud):
        ud.data = self.data
        if self.action is not None:
            action_to_return = self.action
            self.action = None  # Reset the action to prevent re-triggering
            return action_to_return
        
        return 'waiting'  # Default outcome if no action is set


class GotoGoalAction(smach.State):
    def __init__(self) :
        smach.State.__init__(self, outcomes=['success', 'failure'], input_keys=['data'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        if not self.client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Timeout waiting for /move_base/move action server")
            assert False, "Timeout waiting for action server"

    def execute(self, ud):

        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = ud.data['GoalPose']['header']['frame_id']
        goal.target_pose.pose.position.x = ud.data['GoalPose']['pose']['position']['x']
        goal.target_pose.pose.position.y = ud.data['GoalPose']['pose']['position']['y']

        goal.target_pose.pose.orientation.x = ud.data['GoalPose']['pose']['orientation']['x']
        goal.target_pose.pose.orientation.y = ud.data['GoalPose']['pose']['orientation']['y']
        goal.target_pose.pose.orientation.z = ud.data['GoalPose']['pose']['orientation']['z']
        goal.target_pose.pose.orientation.w = ud.data['GoalPose']['pose']['orientation']['w']
        self.client.send_goal(goal)

        result = self.client.wait_for_result()
        if result:
            return 'success'
        else:
            return 'failure'
    
class GotoGoalActionSucces(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['success'])
        self.results_pub = rospy.Publisher("actions/result", UNSWActionResult, queue_size=1)
    
    def execute(self, ud):
        result = UNSWActionResult()
        result.action_name = "goto_goal"
        result.action_result = True
        self.results_pub.publish(result)
        return 'success'
    
class GotoGoalActionFailure(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['failure'])
        self.results_pub = rospy.Publisher("actions/result", UNSWActionResult, queue_size=1)
    
    def execute(self, ud):
        result = UNSWActionResult()
        result.action_name = "goto_goal"
        result.action_result = False
        self.results_pub.publish(result)
        return 'failure'
    
class GotoRegionAction(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['success','failure'], input_keys=['data'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        if not self.client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Timeout waiting for /move_base/move action server")
            assert False, "Timeout waiting for action server"
        
        module_path = os.environ.get("UNSW_WS")
        self.db = DBase(module_path + "/WORLD_MODEL/world_model/world_model_kb/scripts/" + "database.ini")
        self.region_api = RegionsApi()
        
        # Subscribe to occupancy grid for collision checking
        self.occupancy_grid = None
        self.grid_sub = rospy.Subscriber("/map", OccupancyGrid, self.grid_callback, queue_size=1)
        
        # Wait for occupancy grid
        rospy.loginfo("Waiting for occupancy grid...")
        while self.occupancy_grid is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("Occupancy grid received!")
    
    def grid_callback(self, msg):
        self.occupancy_grid = msg
    
    def world_to_grid(self, x, y):
        """Convert world coordinates to grid coordinates"""
        grid_x = int((x - self.occupancy_grid.info.origin.position.x) / self.occupancy_grid.info.resolution)
        grid_y = int((y - self.occupancy_grid.info.origin.position.y) / self.occupancy_grid.info.resolution)
        return grid_x, grid_y
    
    def is_navigable(self, x, y):
        """Check if a point is navigable (not occupied, with clearance)"""
        if self.occupancy_grid is None:
            rospy.logwarn("No occupancy grid available, assuming point is navigable")
            return True
            
        # Convert world coordinates to grid coordinates
        grid_x, grid_y = self.world_to_grid(x, y)
        
        # Check if within grid bounds
        if (grid_x < 0 or grid_x >= self.occupancy_grid.info.width or 
            grid_y < 0 or grid_y >= self.occupancy_grid.info.height):
            return False
        
        # Check for obstacles in a small radius around the point (robot clearance)
        clearance_radius = 3  # grid cells (adjust based on robot size)
        for dx in range(-clearance_radius, clearance_radius + 1):
            for dy in range(-clearance_radius, clearance_radius + 1):
                gx, gy = grid_x + dx, grid_y + dy
                if (0 <= gx < self.occupancy_grid.info.width and 
                    0 <= gy < self.occupancy_grid.info.height):
                    # Check if cell is occupied (typically > 50 means occupied)
                    cell_value = self.occupancy_grid.data[gy * self.occupancy_grid.info.width + gx]
                    if cell_value > 50 or cell_value == -1:  # Occupied or unknown
                        return False
        return True
    
    def point_in_polygon(self, x, y, polygon_coords):
        """Ray casting algorithm to check if point is inside polygon"""
        n = len(polygon_coords)
        inside = False
        p1x, p1y = polygon_coords[0]
        for i in range(1, n + 1):
            p2x, p2y = polygon_coords[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside
    
    def random_point_in_polygon(self, polygon_coords):
        """Generate a random point inside the polygon using bounding box sampling"""
        min_x = min(x for x, y in polygon_coords)
        max_x = max(x for x, y in polygon_coords)
        min_y = min(y for x, y in polygon_coords)
        max_y = max(y for x, y in polygon_coords)
        
        max_attempts = 100
        for _ in range(max_attempts):
            x = random.uniform(min_x, max_x)
            y = random.uniform(min_y, max_y)
            if self.point_in_polygon(x, y, polygon_coords):
                return (x, y)
        
        # Fallback: return centroid if random sampling fails
        center_x = sum(x for x, y in polygon_coords) / len(polygon_coords)
        center_y = sum(y for x, y in polygon_coords) / len(polygon_coords)
        return (center_x, center_y)
    
    def get_navigable_point_in_region(self, polygon_coords):
        """Find a navigable point within the region"""
        # Calculate centroid
        n = len(polygon_coords)
        center_x = sum(x for x, y in polygon_coords) / n
        center_y = sum(y for x, y in polygon_coords) / n
        
        # Generate candidate points in order of preference
        candidates = []
        
        # 1. Try center first
        candidates.append((center_x, center_y))
        
        # 2. Try points at different distances from center
        for radius in [0.3, 0.6, 1.0, 1.5, 2.0]:  # meters
            for angle in [0, 45, 90, 135, 180, 225, 270, 315]:  # degrees
                x = center_x + radius * math.cos(math.radians(angle))
                y = center_y + radius * math.sin(math.radians(angle))
                if self.point_in_polygon(x, y, polygon_coords):
                    candidates.append((x, y))
        
        # 3. Try random points within polygon as fallback
        for _ in range(15):
            x, y = self.random_point_in_polygon(polygon_coords)
            candidates.append((x, y))
        
        # Test each candidate point
        for i, (x, y) in enumerate(candidates):
            if self.is_navigable(x, y):
                if i == 0:
                    rospy.loginfo(f"Using region center: ({x:.2f}, {y:.2f})")
                elif i < 25:  # Systematic search points
                    rospy.loginfo(f"Center blocked, using nearby point: ({x:.2f}, {y:.2f})")
                else:  # Random points
                    rospy.loginfo(f"Using random navigable point: ({x:.2f}, {y:.2f})")
                return (x, y)
        
        rospy.logerr("No navigable point found in region!")
        return None
    
    def execute(self, ud):
        region_name = ud.data['RegionName']
        rospy.loginfo(f"Navigating to region: {region_name}")

        # Look up region name in database
        try:
            region_data = self.region_api.get_region_by_label(self.db.con_pool, region_name)

            rospy.loginfo(f"region_data type: {type(region_data)}")
            rospy.loginfo(f"region_data repr: {repr(region_data)}")

            if region_data is None:
                rospy.logerr(f"Region {region_name} not found in database")
                return 'failure'
                
            region = region_data
            polygon = region['polygon']
            
            # Debug: Check what type polygon is
            rospy.loginfo(f"Polygon type: {type(polygon)}, value: {polygon}")

            # Parse polygon coordinates - handle both string and list formats
            if isinstance(polygon, str):
                coords = re.findall(r'\((-?\d+\.?\d*),(-?\d+\.?\d*)\)', polygon)
                coords = [(float(x), float(y)) for x, y in coords]
            elif isinstance(polygon, list):
                # If it's already a list, just use it
                coords = polygon
            else:
                rospy.logerr(f"Unexpected polygon format: {type(polygon)}")
                return 'failure'
        
            if len(coords) < 3:
                rospy.logerr(f"Invalid polygon for region {region_name}")
                return 'failure'

            # Find a navigable point within the region
            target_point = self.get_navigable_point_in_region(coords)
            
            if target_point is None:
                rospy.logerr(f"Could not find navigable point in region {region_name}")
                return 'failure'

            # Send goal to move_base
            goal = MoveBaseGoal()
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.pose.position.x = target_point[0]
            goal.target_pose.pose.position.y = target_point[1]
            goal.target_pose.pose.position.z = 0.0
            goal.target_pose.pose.orientation.x = 0.0
            goal.target_pose.pose.orientation.y = 0.0
            goal.target_pose.pose.orientation.z = 0.0
            goal.target_pose.pose.orientation.w = 1.0
            
            rospy.loginfo(f"Sending goal to: ({target_point[0]:.2f}, {target_point[1]:.2f})")
            self.client.send_goal(goal)
            
            # Wait for result with timeout
            result = self.client.wait_for_result(timeout=rospy.Duration(60.0))  # 60 second timeout
            
            if result:
                state = self.client.get_state()
                if state == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo(f"Successfully reached region {region_name}")
                    return 'success'
                else:
                    rospy.logwarn(f"Navigation failed with state: {state}")
                    return 'failure'
            else:
                rospy.logwarn("Navigation timed out")
                self.client.cancel_goal()
                return 'failure'
                
        except Exception as e:
            rospy.logerr(f"Error in GotoRegionAction: {e}")
            return 'failure'

class GotoRegionActionSuccess(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])
        self.results_pub = rospy.Publisher("actions/result", UNSWActionResult, queue_size=1)
    
    def execute(self, ud):
        result = UNSWActionResult()
        result.action_name = "goto_region"
        result.action_result = True
        self.results_pub.publish(result)
        return 'success'

class GotoRegionActionFailure(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['failure'])
        self.results_pub = rospy.Publisher("actions/result", UNSWActionResult, queue_size=1)
    
    def execute(self, ud):
        result = UNSWActionResult()
        result.action_name = "goto_region"
        result.action_result = False
        self.results_pub.publish(result)
        return 'failure'


def main():
    rospy.init_node('navigation_client_smach')
    sm = smach.StateMachine(outcomes = ["action_success"])

    with sm:
        smach.StateMachine.add('WAIT_FOR_ACTION', 
                               WaitForAction(), 
                               transitions={'goto_goal_action':'GOTO_GOAL_ACTION',
                                            'goto_region_action':'GOTO_REGION_ACTION',
                                            'waiting' : 'WAIT_FOR_ACTION'
                                            })
        smach.StateMachine.add('GOTO_GOAL_ACTION',
                               GotoGoalAction(),
                               transitions={
                                   'success':'GOTO_GOAL_ACTION_SUCCESS',
                                   'failure' : 'GOTO_GOAL_ACTION_FAILURE'
                               })
        smach.StateMachine.add('GOTO_GOAL_ACTION_SUCCESS',
                               GotoGoalActionSucces(),
                               transitions={
                                   'success' : 'WAIT_FOR_ACTION'
                               })
        smach.StateMachine.add('GOTO_GOAL_ACTION_FAILURE',
                               GotoGoalActionFailure(),
                               transitions={
                                   'failure' : 'WAIT_FOR_ACTION'
                               })
        smach.StateMachine.add('GOTO_REGION_ACTION',
                               GotoRegionAction(),
                               transitions={
                                   'success':'GOTO_REGION_ACTION_SUCCESS',
                                   'failure' : 'GOTO_REGION_ACTION_FAILURE'
                               })
        smach.StateMachine.add('GOTO_REGION_ACTION_SUCCESS',
                               GotoRegionActionSuccess(),
                               transitions={
                                   'success' : 'WAIT_FOR_ACTION'
                               })
        smach.StateMachine.add('GOTO_REGION_ACTION_FAILURE',
                               GotoRegionActionFailure(),
                               transitions={
                                   'failure' : 'WAIT_FOR_ACTION'
                               })

    sis = smach_ros.IntrospectionServer('server_name',sm,'/SM_ROOT')
    sis.start()

    outcome = sm.execute()
    rospy.logwarn(f"State machine finished with outcome: {outcome}")
    rospy.spin()
    sis.stop()

if __name__== "__main__":
    main()