#!/usr/bin/env python
'''
View the regions overlaid on the occupancy grid that are saved in the database
Use via the launch file in launch
'''

import rospy
import json
import numpy as np
import cv2

from regions_api import RegionsApi
from db import Db

import nav_msgs
from nav_msgs.msg import OccupancyGrid

class RegionsViewer:
    def __init__(self):
        rospy.init_node("regions_viewer")
        map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.__last_map_msg = None

        self.__db_config_path = rospy.get_param("~database_config_path")
        self.__db = Db(self.__db_config_path)
        self.__regions_api = RegionsApi()

    def start(self):
        pass

    def map_cb(self, map_msg):
        self.__last_map_msg = map_msg


    def draw_regions(self):
        if self.__last_map_msg is None:
            raise Exception("No map received!")

        map_img = self.map_msg_to_image(self.__last_map_msg)
        polygons = self.get_regions_polygons()
        self.plot_polygons(map_img, polygons)

        cv2.imshow("map", map_img)
        cv2.waitKey(1)

    def map_msg_to_image(self, map_msg):
        width = map_msg.info.width
        height = map_msg.info.height
        self.__resolution = map_msg.info.resolution
        self.__origin = np.zeros(2)
        self.__origin[0] = map_msg.info.origin.position.x
        self.__origin[1] = map_msg.info.origin.position.y

        map_img = np.zeros((height, width, 3), np.float32)
        
        map_img = map_img.reshape((-1, 3))
        for (grid_value, i) in zip(map_msg.data, range(0, len(map_msg.data))):
            if (grid_value == -1):
                map_img[i] = (175, 175, 175)
            elif (grid_value == 100):
                map_img[i] = (0, 0, 0)
            elif (grid_value == 0):
                map_img[i] = (255, 255, 255)

        map_img = map_img.reshape((height, width, 3))
        return map_img

    def get_regions_polygons(self):
        db_resp = self.__regions_api.get_regions(self.__db.con_pool)
        rows = db_resp[0][0]
        regions = []

        for region in rows:
            # We get a string ((x1, y1), (x2, y2), ...,)
            # We want to convert this to a numpy array
            outer_brackets_striped = region["polygon"][2:-2]
            outer_brackets_striped = outer_brackets_striped.replace("(", "")
            outer_brackets_striped = outer_brackets_striped.replace(")", "")
            polygon_mat = np.fromstring(outer_brackets_striped, sep=",")
            ##print(polygon_mat)
            polygon_mat = polygon_mat.reshape((-1, 2))
            polygon_img_coords = self.real_coords_to_box_coords(polygon_mat)
            #print(polygon_img_coords.shape)
            regions.append(polygon_img_coords)
        return regions

    def plot_polygons(self, img, polygons):
        for polygon in polygons:
            for i in range(len(polygon) - 1):
                a = tuple(polygon[i].tolist())
                b = tuple(polygon[i+1].tolist())
                cv2.line(img, a, b, (255, 0, 0), 2)
        return img

    def real_coords_to_box_coords(self, coords):
        return ((coords - self.__origin) / self.__resolution).astype(int)


if __name__ == "__main__":
    viewer = RegionsViewer()
    viewer.start()
    r = rospy.Rate(1)

    while (not rospy.is_shutdown()):
        rospy.sleep(1)
        viewer.draw_regions()
        '''
        try:
            viewer.draw_regions()
        except Exception as e:
            print(e)
            raise e
        '''

        r.sleep()
