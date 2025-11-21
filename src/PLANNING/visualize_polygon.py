#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import sys
import os

# Add database path
module_path = os.environ.get("UNSW_WS")
sys.path.append(module_path + "/WORLD_MODEL/world_model/world_model_kb/scripts")
from dbase import DBase
from regions_api import RegionsApi
import re


class PolygonVisualizer:
    def __init__(self):
        rospy.init_node('polygon_visualizer')
        
        # Publisher for visualization markers
        self.marker_pub = rospy.Publisher('/polygon_markers', Marker, queue_size=10)
        
        # Database setup
        self.regions_api = RegionsApi()
        module_path = os.environ.get("UNSW_WS")
        self.db = DBase(module_path + "/WORLD_MODEL/world_model/world_model_kb/scripts/database.ini")
        
        rospy.sleep(1.0)  # Wait for publisher to be ready
        
    def create_polygon_marker(self, polygon_coords, marker_id=0, color=None, label=""):
        """
        Create a LINE_STRIP marker to visualize a polygon
        
        Args:
            polygon_coords: List of (x, y) tuples representing polygon vertices
            marker_id: Unique ID for the marker
            color: ColorRGBA object (default: green with transparency)
            label: Text label for the polygon
        """
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "polygons"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Scale (line width)
        marker.scale.x = 0.05  # Line width in meters
        
        # Color
        if color is None:
            marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.8)  # Green with transparency
        else:
            marker.color = color
        
        # Add points to create the polygon outline
        for x, y in polygon_coords:
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.0
            marker.points.append(point)
        
        # Close the polygon by adding the first point again
        if len(polygon_coords) > 0:
            first_point = Point()
            first_point.x = polygon_coords[0][0]
            first_point.y = polygon_coords[0][1]
            first_point.z = 0.0
            marker.points.append(first_point)
        
        return marker
    
    def create_filled_polygon_marker(self, polygon_coords, marker_id=0, color=None):
        """
        Create a TRIANGLE_LIST marker to visualize a filled polygon
        This uses a simple fan triangulation from the first vertex
        """
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "polygons_filled"
        marker.id = marker_id
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        
        # Scale
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        
        # Color (more transparent for filled)
        if color is None:
            marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.3)  # Green with more transparency
        else:
            marker.color = color
        
        # Triangulate polygon using fan method (works for convex polygons)
        if len(polygon_coords) >= 3:
            for i in range(1, len(polygon_coords) - 1):
                # First vertex
                p1 = Point()
                p1.x = polygon_coords[0][0]
                p1.y = polygon_coords[0][1]
                p1.z = 0.0
                marker.points.append(p1)
                
                # Current vertex
                p2 = Point()
                p2.x = polygon_coords[i][0]
                p2.y = polygon_coords[i][1]
                p2.z = 0.0
                marker.points.append(p2)
                
                # Next vertex
                p3 = Point()
                p3.x = polygon_coords[i + 1][0]
                p3.y = polygon_coords[i + 1][1]
                p3.z = 0.0
                marker.points.append(p3)
        
        return marker
    
    def create_text_marker(self, position, text, marker_id=0):
        """Create a text marker to label the polygon"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "polygon_labels"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = 0.5  # Hover above ground
        
        marker.scale.z = 0.3  # Text height
        
        marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)  # White
        marker.text = text
        
        return marker
    
    def visualize_all_regions(self):
        """Fetch all regions from database and visualize them"""
        rospy.loginfo("Fetching regions from database...")
        
        regions_data = self.regions_api.get_regions(self.db.con_pool)
        
        if regions_data is None or len(regions_data) == 0:
            rospy.logwarn("No regions found in database")
            return
        
        regions = regions_data[0][0]  # Unwrap the result
        
        if regions is None:
            rospy.logwarn("No regions data")
            return
        
        rospy.loginfo(f"Found {len(regions)} regions")
        
        # Define colors for different regions
        colors = [
            ColorRGBA(1.0, 0.0, 0.0, 0.8),  # Red
            ColorRGBA(0.0, 1.0, 0.0, 0.8),  # Green
            ColorRGBA(0.0, 0.0, 1.0, 0.8),  # Blue
            ColorRGBA(1.0, 1.0, 0.0, 0.8),  # Yellow
            ColorRGBA(1.0, 0.0, 1.0, 0.8),  # Magenta
            ColorRGBA(0.0, 1.0, 1.0, 0.8),  # Cyan
        ]
        
        for idx, region in enumerate(regions):
            region_id = region['region_id']
            label = region['label']
            polygon_str = region['polygon']
            
            rospy.loginfo(f"Visualizing region {region_id}: {label}")
            rospy.loginfo(f"Polygon string: {polygon_str}")
            
            # Parse polygon coordinates
            coords = re.findall(r'\((-?\d+\.?\d*),(-?\d+\.?\d*)\)', polygon_str)
            coords = [(float(x), float(y)) for x, y in coords]
            
            if len(coords) < 3:
                rospy.logwarn(f"Invalid polygon for region {label}: need at least 3 points")
                continue
            
            rospy.loginfo(f"Parsed {len(coords)} coordinates: {coords}")
            
            # Choose color
            color = colors[idx % len(colors)]
            filled_color = ColorRGBA(color.r, color.g, color.b, 0.3)
            
            # Create and publish filled polygon
            filled_marker = self.create_filled_polygon_marker(coords, marker_id=idx*3, color=filled_color)
            self.marker_pub.publish(filled_marker)
            
            # Create and publish outline
            outline_marker = self.create_polygon_marker(coords, marker_id=idx*3+1, color=color, label=label)
            self.marker_pub.publish(outline_marker)
            
            # Calculate centroid for label placement
            centroid_x = sum(x for x, y in coords) / len(coords)
            centroid_y = sum(y for x, y in coords) / len(coords)
            
            # Create and publish text label
            text_marker = self.create_text_marker((centroid_x, centroid_y), label, marker_id=idx*3+2)
            self.marker_pub.publish(text_marker)
            
            rospy.sleep(0.1)  # Small delay between markers
        
        rospy.loginfo("All regions visualized!")
    
    def visualize_region_by_name(self, region_name):
        """Visualize a specific region by name"""
        rospy.loginfo(f"Fetching region: {region_name}")
        
        region_data = self.regions_api.get_region_by_label(self.db.con_pool, region_name)
        
        if region_data is None:
            rospy.logerr(f"Region {region_name} not found")
            return
        
        region = region_data
        polygon_str = region['polygon']
        
        # Parse polygon coordinates
        coords = re.findall(r'\((-?\d+\.?\d*),(-?\d+\.?\d*)\)', polygon_str)
        coords = [(float(x), float(y)) for x, y in coords]
        
        if len(coords) < 3:
            rospy.logerr(f"Invalid polygon for region {region_name}")
            return
        
        rospy.loginfo(f"Visualizing {len(coords)} coordinates")
        
        # Create markers
        color = ColorRGBA(0.0, 1.0, 0.0, 0.8)  # Green
        filled_color = ColorRGBA(0.0, 1.0, 0.0, 0.3)
        
        filled_marker = self.create_filled_polygon_marker(coords, marker_id=0, color=filled_color)
        outline_marker = self.create_polygon_marker(coords, marker_id=1, color=color)
        
        # Calculate centroid
        centroid_x = sum(x for x, y in coords) / len(coords)
        centroid_y = sum(y for x, y in coords) / len(coords)
        text_marker = self.create_text_marker((centroid_x, centroid_y), region_name, marker_id=2)
        
        # Publish markers
        self.marker_pub.publish(filled_marker)
        self.marker_pub.publish(outline_marker)
        self.marker_pub.publish(text_marker)
        
        rospy.loginfo(f"Region {region_name} visualized!")


def main():
    visualizer = PolygonVisualizer()
    
    # Get command line arguments
    if len(sys.argv) > 1:
        region_name = sys.argv[1]
        rospy.loginfo(f"Visualizing specific region: {region_name}")
        visualizer.visualize_region_by_name(region_name)
    else:
        rospy.loginfo("Visualizing all regions")
        visualizer.visualize_all_regions()
    
    # Keep publishing the markers
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        # Re-publish to keep markers visible
        if len(sys.argv) > 1:
            visualizer.visualize_region_by_name(sys.argv[1])
        else:
            visualizer.visualize_all_regions()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass