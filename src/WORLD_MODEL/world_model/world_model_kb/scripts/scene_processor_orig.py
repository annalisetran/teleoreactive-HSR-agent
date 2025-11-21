#!/usr/bin/env python3

import rospy
from tracker import Tracker
from db import Db
from scene_api import SceneApi
from dateutil import parser
import datetime
import math
import time

class SceneProcessor:

    def __init__(self, dt):
        self.dt = dt
        rospy.init_node('scene_processor')
        db_config_path = rospy.get_param("~database_config_path", "/home/robocup/.ros/world_model_kb/database.ini")
        self.db = Db(db_config_path)               # database connections
        self.scn = SceneApi()        # apis for scene/scene objects
        #initialise the tracker
        self.tracker = Tracker(self.dt, db_config_path, 2)

        self.show_plots = True

        if (self.show_plots):
            self.tracker.init_plot()

    def tick(self):
        # Update predictions
        self.tracker.predict()

        # Apply measurement updates if available
        # get list of unprocessed scenes
        processed_list = []
        scenes_to_process = self.scn.get_unprocessed_scenes(self.db.con_pool)

        if scenes_to_process is not None:
            last_scene_time = None

            for scene in scenes_to_process:
                if (rospy.is_shutdown()):
                    break
                print("Processing scene: {}".format(scene['scene_id']))
                scene_objs = self.scn.get_objects_in_scene(self.db.con_pool, scene['scene_id'])
                scene_time = parser.parse(scene['created'])

                if scene_objs != None:
                    self.tracker.update(scene_objs) 
                self.scn.update_scene_to_processed(self.db.con_pool, scene['scene_id'])

                if (self.show_plots):
                    self.tracker.plot_world()
                    print(self.tracker.print_tracked_objects())

        if self.show_plots:
            self.tracker.plot_world()


    def tick_simulated(self):
        # Update predictions
        self.tracker.predict()

        # Apply measurement updates if available
        # get list of unprocessed scenes
        processed_list = []
        scenes_to_process = self.scn.get_unprocessed_scenes(self.db.con_pool)

        if scenes_to_process is not None:
            last_scene_time = None

            for scene in scenes_to_process:
                if (rospy.is_shutdown()):
                    break
                print("Processing scene: {}".format(scene['scene_id']))
                scene_objs = self.scn.get_objects_in_scene(self.db.con_pool, scene['scene_id'])
                scene_time = parser.parse(scene['created'])
                # TODO: Remove this code after testing
                # To allow for replaying from a snapshot of the scene database we keep applying predicts until we catchup with the timestamp from the measurement
                if not last_scene_time is None:
                    time_diff = (scene_time - last_scene_time).total_seconds() # s
                    num_ticks_needed = int(math.floor(time_diff/self.dt))
                    for _ in range(0, num_ticks_needed):
                        self.tracker.predict()
                        time.sleep(dt)
                last_scene_time = scene_time
                # End remove area

                if scene_objs != None:
                    self.tracker.update(scene_objs) 
                self.scn.update_scene_to_processed(self.db.con_pool, scene['scene_id'])

                if (self.show_plots):
                    self.tracker.plot_world()
                    time.sleep(dt)

        if self.show_plots:
            self.tracker.plot_world()
            print(self.tracker.print_tracked_objects())

if __name__ == "__main__":
    dt = 0.1 # s
    scene_proc = SceneProcessor(dt)
    r = rospy.Rate(1.0/dt) # 10 hz

    while not rospy.is_shutdown():
        scene_proc.tick()
        r.sleep()




