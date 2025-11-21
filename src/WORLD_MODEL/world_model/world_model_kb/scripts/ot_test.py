from tracker import Tracker
from db import Db

from object_api import ObjectApi
from scene_api import SceneApi

db = Db()               # database connections
scn = SceneApi()        # apis for scene/scene objects 

#initialise the tracker
tracker = Tracker(2.5)

while True:
    # get list of unprocessed scenes
    processed_list = []
    scenes_to_process = scn.get_unprocessed_scenes(db.con_pool)

    if scenes_to_process == None:
        break

    for scene in scenes_to_process:
        scene_objs = scn.get_objects_in_scene(db.con_pool, scene['scene_id'])
        if scene_objs != None:      # scene contains objects
            tracker.update(scene_objs) 
        scn.update_scene_to_processed(db.con_pool,scene['scene_id'])

print("finished processing...")

