#!/usr/bin/env python3
# Version 1.1.1
# written by Adam Golding

import os

from dbase import DBase
from robot_state_api import RobotStateApi
from object_class_api import ObjectClassApi
from object_api import ObjectApi
#from scene_api import SceneApi
from regions_api import RegionsApi
from agent_api import AgentApi


#db = Db(os.path.abspath("/home/oli/.ros/world_model_kb/database.ini"))               # database connections
db = DBase(os.path.abspath("/home/adam/workspace/vision_ws/src/world_model/world_model_kb/scripts/database.ini"))      # database connections
rs = RobotStateApi()    # apis to robot state
objcls = ObjectClassApi()  # apis to object class
obj = ObjectApi()       # apis for object
#scn = SceneApi()        #apis for scene/scene objects 
regions = RegionsApi()
agent = AgentApi()


# data without object relationship
_obj = {}   
_obj["object_class_id"] = 16
_obj["class_confidence"] = 0.87
_obj["associated_text"] = None
_obj["frame_id"] = "map"
_obj["bbox_x"] = 30
_obj["bbox_y"] = 530
_obj["bbox_width"] = 180
_obj["bbox_height"] = 70
_obj["bbox_cols"] = 150
_obj["bbox_rows"] = 70
_obj["loc_x"] = 97.3
_obj["loc_y"] = 88.7
_obj["loc_z"] = 0.0
_obj["loc_confidence"] = 1.0
_obj["colour"] = None
_obj["texture"] = None
_obj["size_id"] = None
_obj["shape_id"] = None
_obj["img_path"] = None
_obj["pcd_path"] = None


id = obj.insert_object(db.con_pool, _obj)
print("New object_id = ", id)

# agent.update_agent_attribute(db.con_pool, 34, 1, 'sprite')

# fav_drink = agent.get_agent_attribute(db.con_pool, 34, 1)

# print(f'Favoutite Drink: {fav_drink}')


# test = regions.get_poi_by_name(db.con_pool, 'inspection point')
# print(test)

# test = regions.get_poi_list_inc_desc(db.con_pool)
# print(test)

# test = obj.get_objects_of_class_list(db.con_pool, 'door')
# print(test)



# region_id = regions.insert_region(db.con_pool, "square", [0, 0, 1, 1], [0, 1, 1, 0], region_id=34)

# print("Inserted region with id: {}".format(region_id))

# regions.find_point_label(db.con_pool, 0.5, 0.5)


# print("\n\nFind Generic Object Class by Label 1")
# this_object_class = objcls.get_base_object_class_by_label(db.con_pool, 'can')
# print(this_object_class)

'''
#### robot state queries

# pass the connection pool object to the method

print()
print("Robot State")
# get the current robot state
robot_state_cur =  rs.get_robot_state_current(db.con_pool)
print(robot_state_cur)

print("Insert Robot State")

new_data = '[{"frame_id": "frame 1","base_pose_x":5, "base_pose_y":6, "base_pose_z":0, "base_pose_w":1.7, "region_id":0, base_r":0,"base_p":0,"base_yw":0, \
        "head_pose_x":4,"head_pose_y":6,"head_pose_z":0, "head_pose_w":1.0, "head_r":0,"head_p":0,"head_yw":0, \
        "driveable_state":1,"arm_state":1,"gripper_state":0,"shoulder_state":0,"holding_object_id":1}]'
id = rs.insert_robot_state(db.con_pool, new_data)
print("New robot_state_id = ", id)

print("List of Robot State")
robot_state_hist = rs.get_robot_state_history(db.con_pool)
print(robot_state_hist)


### object_class queries

print("\n\nFind Generic Object Class by Label 1")
this_object_class = objcls.get_generic_object_class_by_label(db.con_pool, 'can')
print(this_object_class)

print("\nList of Object Class")
all_object_class = objcls.get_object_class_list(db.con_pool)
print(all_object_class)

print("\n\nObject Class")
this_object_class = objcls.get_object_class_by_id(db.con_pool, 3)
print(this_object_class)

print("\n\nInsert Object Class")
new_data = '[{"description": "Chilli Lime Chips", "pc_path": "/point_cloud/object/pc8.txt", "image_path": "/images/object/img8.jpg" , "decay_weight": 0.9, "object_class_label": [{"object_label" : "chips"}]}]'
id = objcls.insert_object_class(db.con_pool, new_data)
print("New object_class_id = ", id)

print("\n\nInsert Object Class")
new_data = '[{"description": "Cat", "pc_path": "/point_cloud/object/pc8.txt", "image_path": "/images/object/img8.jpg" , "decay_weight": 0.9, "object_class_label": [{"object_label" : "cat"}]}]'
id = objcls.insert_object_class(db.con_pool, new_data)
print("New object_class_id = ", id)

print("\n\nInsert Object Class")
new_data = '[{"description": "Can of beer", "pc_path": "/point_cloud/object/pc8.txt", "image_path": "/images/object/img8.jpg" , "decay_weight": 0.9, "object_class_label": [{"object_label" : "Can", "object_label": "beer"}]}]'
id = objcls.insert_object_class(db.con_pool, new_data)
print("New object_class_id = ", id)


print("\n\nFind Object Class by Desc")
this_object_class = objcls.get_object_class_by_desc(db.con_pool, 'Can of coke')
print(this_object_class)

print("\n\nFind Object Class by Label 1")
this_object_class = objcls.get_object_class_by_label(db.con_pool, ('box','coke'))
print(this_object_class)


print("\n\nFind Object Class by Label 2")
this_object_class = objcls.get_object_class_by_label(db.con_pool, ('can','coke'))
print(this_object_class)

print("\n\nFind Object Class by Label 3")
this_object_class = objcls.get_object_class_by_label(db.con_pool, ('solo',))
print(this_object_class)

print("\n\nFind Object Class Ids by Label 1")
this_object_class = objcls.get_object_class_id_by_label(db.con_pool, ('box','coke'))
print(this_object_class)

print("\n\nFind Object Class Ids by Label 2")
this_object_class = objcls.get_object_class_id_by_label(db.con_pool, ('can','coke'))
print(this_object_class)

print("\n\nFind Object Class Ids by Label 3")
this_object_class = objcls.get_object_class_id_by_label(db.con_pool, ('can',))
print(this_object_class)

print("\n\nInsert Object Label - New")
new_data = "grass"
id = objcls.insert_object_label(db.con_pool, new_data)
print("New object_label_id = ", id)

print("\n\nInsert Object Label - Existing")
new_data = 'beer'
id = objcls.insert_object_label(db.con_pool, new_data)
print("New object_label_id = ", id)
'''
### object queries
'''
print("\nList of Objects")
object_list = obj.get_object_list(db.con_pool)
print(object_list)

print()
print("Insert Object")

# data with object relationship
new_data = '[{"object_class_id": 16, "description": "White plate", "associated_text": null, "frame_id": "frame 1", "pose_x": 3, "pose_y": 4, "pose_z": 1, "pose_w": 1.5, \
        "orient_r": 0, "orient_p": 0, "orient_yw": 0, "region_id": 3, \
        "colour": "0,65,180", "colour_low": "0,65,180", "colour_high": "0,65,180","shape_id": null, "bounding_box": null, "size_id": 3, \
        "image_path": "/images/object/img5.jpg", \
        "pc_path": "/point_cloud/object/pc5.txt", "features": null, "class_confidence": 0.87, "loc_confidence": 0.9, \
        "orient_confidence": 0.0, "size_confidence": 0.7, "colour_confidence": 0.95, \
        "object_relationship": [{"related_id" : 1, "object_relationship_type_id" : 2}]}]'

id = obj.insert_object(db.con_pool, new_data)
print("New object_id = ", id)

# data without object relationship
new_data = '[{"object_class_id": 16, "description": "Big white plate", "associated_text": null, "frame_id": "frame 1", "pose_x": 3, "pose_y": 4, "pose_z": 1, "pose_w": 1.9, \
        "orient_r": 0, "orient_p": 0, "orient_yw": 0, "region_id": 3, \
        "colour": "0,65,180", "colour_low": "0,65,180", "colour_high": "0,65,180", "shape_id": null, "bounding_box": null, "size_id": 3, "image_path": "/images/object/img5.jpg", \
        "pc_path": "/point_cloud/object/pc5.txt", "features": null, "class_confidence": 0.87, "loc_confidence": 0.9}]'

id = obj.insert_object(db.con_pool, new_data)
print("New object_id = ", id)

new_data = '[{"object_class_id": 16, "description": "Small white plate", "associated_text": null, "frame_id": "frame 1", "pose_x": 4, "pose_y": 8, "pose_z": 2, "pose_w": 1.1, \
        "orient_r": 0, "orient_p": 0, "orient_yw": 0, "region_id": 1, \
        "colour": "10,100,190", "colour_low": "10,100,190", "colour_high": "10,100,190", "shape_id": null, "bounding_box": null, "size_id": 2, "image_path": "/images/object/img9.jpg", \
        "pc_path": "/point_cloud/object/pc9.txt", "features": null, "class_confidence": 0.9, "loc_confidence": 0.99}]'

id = obj.insert_object(db.con_pool, new_data)
print("New object_id = ", id)

print("\n\nObject")
this_object = obj.get_object_by_id(db.con_pool, 2)
print(this_object)

print("\n\nSearch by Description")
this_object = obj.get_object_by_desc(db.con_pool, 'plate')
print(this_object)

print("\n\nSearch by Labels")
this_object = obj.get_object_by_label(db.con_pool, ('apple',))
print(this_object)

print("\n\nSearch by Location")
this_object = obj.get_object_by_loc(db.con_pool, 'Kitchen')
print(this_object)
'''
# scene data

new_data = '[{"frame_id" : "frame 1", "base_pose_x":7, "base_pose_y":7, "base_pose_z":0, "base_pose_w":1.7, "base_r":0,"base_p":0,"base_yw":0, \
        "head_pose_x":4,"head_pose_y":6,"head_pose_z":0, "head_pose_w":1.0, "head_r":0,"head_p":0,"head_yw":0, \
        "driveable_state":1,"arm_state":1,"gripper_state":0,"shoulder_state":0,"holding_object_id":1}]'
rs_id = rs.insert_robot_state(db.con_pool, new_data)

scn_id = scn.insert_scene(db.con_pool, rs_id)
scene_obj = '[{"lables": "[(\'plate\', 0.8)]", "description": "Small white plate", "associated_text": null, "frame_id" : "frame 1", "pose_x": 4, "pose_y": 8, "pose_z": 2, "pose_w":1.1, \
        "orient_r": 0, "orient_p": 0, "orient_yw": 0, "loc_desc": "Kitchen", \
        "colour": "10,100,190", "colour_low": "10,100,190", "colour_high": "10,100,190", "bounding_box": null, "image_path": "/images/object/img9.jpg", \
        "features": null, "fully_observed": false}]'
scn.insert_scene_object(db.con_pool, scene_obj, scn_id)

scene_obj = '[{"labels": "[(\'bowl\', 0.8), (\'plate\', 0.9)]", "description": "Small yellow plate", "associated_text": null, "frame_id": "frame 1", "pose_x": 4, "pose_y": 8, "pose_z": 2, "pose_w":1.1, \
        "orient_r": 0, "orient_p": 0, "orient_yw": 0, "loc_desc": "Kitchen", \
        "colour": "10,100,100", "colour_low": "10,100,100", "colour_high": "10,100,100", "bounding_box": null, "image_path": "/images/object/img10.jpg", \
        "features": null, "fully_observed": true}]'
scn.insert_scene_object(db.con_pool, scene_obj, scn_id)

print("\n\nGet Current Scene")
cur_scene = scn.get_current_scene(db.con_pool)
print(cur_scene)


print("\n\nGet Current Scene Objects")
cur_scene = scn.get_objects_in_current_scene(db.con_pool)
print(cur_scene)


print("\n\nGet Objects in Scene")
cur_scene = scn.get_objects_in_scene(db.con_pool,1)
print(cur_scene)
