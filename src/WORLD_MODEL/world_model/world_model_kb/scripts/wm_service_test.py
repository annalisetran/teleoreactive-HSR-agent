#!/usr/bin/env python

from world_model_kb_msgs.srv import Insert, Get, InsertResponse, GetResponse, GetRequest, InsertRequest
from world_model_kb_msgs.msg import Details
from diagnostic_msgs.msg import KeyValue

import rospy
import json

class WmClient:
    def __init__(self):
        rospy.init_node('world_model_test')

        rospy.loginfo("Setting up world_model_test")

        rospy.wait_for_service("/world_model/get_object")
        self.__get_object = rospy.ServiceProxy("/world_model/get_object", Get)

        rospy.wait_for_service("/world_model/get_object_list")
        self.__get_object_list = rospy.ServiceProxy("/world_model/get_object_list", Get)

        rospy.wait_for_service("/world_model/insert_object")
        self.__insert_object = rospy.ServiceProxy("/world_model/insert_object", Insert)
        """

        rospy.wait_for_service("/world_model/insert_object_def")
        self.__insert_object_def = rospy.ServiceProxy("/world_model/insert_object_def", Insert)

        rospy.wait_for_service("/world_model/get_object_def_list")
        self.__get_object_def_list = rospy.ServiceProxy("/world_model/get_object_def_list", Get)
        
        rospy.wait_for_service("/world_model/insert_robot_state")
        self.__insert_robot_state = rospy.ServiceProxy("/world_model/insert_robot_state", Insert)

        rospy.wait_for_service("/world_model/get_robot_state_current")
        self.__get_robot_state = rospy.ServiceProxy("/world_model/get_robot_state_current", Get)
        """
        rospy.wait_for_service("/world_model/insert_scene")
        self.__insert_scene = rospy.ServiceProxy("/world_model/insert_scene", Insert)

        rospy.wait_for_service("/world_model/insert_scene_object")
        self.__insert_scene_object = rospy.ServiceProxy("/world_model/insert_scene_object", Insert)

        rospy.wait_for_service("/world_model/insert_region")
        self.__insert_region = rospy.ServiceProxy("/world_model/insert_region", Insert)

        rospy.wait_for_service("/world_model/find_point_region")
        self.__find_point_region = rospy.ServiceProxy("/world_model/find_point_region", Get)

        rospy.loginfo("Finished setting up world_model_test")

    def run(self):
        print("Running tests")
        request_dic = {"label": "my room", "points_x": [0, 0, 1, 1], "points_y": [0, 1, 1, 0], "neighbors": None, "region_id": 32}
        req = InsertRequest()
        req.insert.json = json.dumps(request_dic)
        resp = self.__insert_region(req)
        print("insert region {} returned {}".format(req, resp))

        request_dic = {"x": 5, "y": 5}
        req = GetRequest()
        req.criteria.json = json.dumps(request_dic)
        resp = self.__find_point_region(req)
        print("Lookup location {} recieve region {}".format(req, resp))


        request_dic = {"object_id": 3}
        req = GetRequest()
        req.criteria.json = json.dumps(request_dic)

        resp = self.__get_object(req)
        resp_dic = json.loads(resp.matches[0].json)
        rospy.loginfo("Database returned item with description: {}".format(resp_dic["description"]))

        req = GetRequest()
        resp = self.__get_object_list(req)
        resp_dic = json.loads(resp.matches[0].json)
        rospy.loginfo("Database returned item with description: {}".format(resp_dic["description"]))
        
        # Lets try an insert
        new_obj_dic = {"object_class_id": 16, "description": "Big white plate", "associated_text": None, "loc_x": 3, "loc_y": 4, "loc_z": 1, \
                "orient_r": 0, "orient_p": 0, "orient_yw": 0, "loc_desc": "Living room", \
                "colour": "0,65,180", "shape_id": None, "bounding_box": None, "size_id": 3, "image_path": "/images/object/img5.jpg", \
                "pc_path": "/point_cloud/object/pc5.txt", "features": None, "class_confidence": 0.87, "loc_confidence": 0.9, \
                "orient_confidence": 0.0, "size_confidence": 0.7, "colour_confidence": 0.95, \
                "object_relationship": [{"related_id" : 1, "object_relationship_type_id" : 2}]}

        req = InsertRequest()
        print(json.dumps(new_obj_dic))
        req.insert.json = json.dumps(new_obj_dic)
        print(req)
        resp = self.__insert_object(req)
        print("New object_id = {}".format(resp.inserted_id))

        # Insert a new scene
        print("Inserting new scene")
        scene_insert_dic = {Details.ROBOT_STATE_ID: 1}
        req = InsertRequest()
        req.insert.json = json.dumps(scene_insert_dic)
        print(req)
        resp = self.__insert_scene(req)
        scene_id = resp.inserted_id
        print(resp)

        print("Inserting object into the new scene")
        new_scene_obj_dic = {"object_category": "Book", "description": "Small white plate", "associated_text": None, "pose_x": 4, "pose_y": 8, "pose_z": 2, "pose_w":1.1, \
                "orient_r": 0, "orient_p": 0, "orient_yw": 0, "loc_desc": "Kitchen", \
                "colour": "10,100,190", "colour_low": "10,100,190", "colour_high": "10,100,190", "bounding_box": None, "image_path": "/images/object/img9.jpg", \
                "features": None, "fully_observed": False, "class_confidence": 0.9}
        new_scene_obj_dic.update({"scene_id": scene_id})
        req = InsertRequest()
        req.insert.json = json.dumps(new_scene_obj_dic)
        print(req)
        resp = self.__insert_scene_object(req)
        print(resp)

        




        

    """
        resp = self.__get_object_list()
        # Print the obj id and colour for each object
        for obj in resp.matches:
            obj_id = [x.value for x in obj.details if x.key == "object_id"]
            colour = [x.value for x in obj.details if x.key == "colour"]
            print("obj_id: {} colour: {}".format(obj_id, colour))

        # Get a specific object
        req = GetRequest()
        req.criteria.details.append(KeyValue(key="object_id", value=str(2)))
        resp = self.__get_object(req)
        print(resp)

        # Insert an object into the database
        req = InsertRequest()
        new_data = {"base_x": 123, "base_y": 2, "base_z": 3, \
                "base_r": 0, "base_p": 0, "base_yw": 0.1, \
                "head_x": 0, "head_y": 1, "head_z": 2, \
                "head_r": 0.2, "head_p": 0.1, "head_yw": 0.1, \
                "driveable_state": 1, "arm_state": 0, "shoulder_state": 0,
                "holding_object_id": 2}
        req.insert.details = self.__dictToKeyValueList(new_data)
        resp = self.__insert_robot_state(req)
        print(resp)

        print(self.__get_robot_state())

        '''
        new_data = {"object_class_id": 16, "description": "Big white plate", "associated_text": None, "loc_x": 3, "loc_y": 4, "loc_z": 1, \
                "orient_r": 0, "orient_p": 0, "orient_yw": 0, "loc_desc": "Living room", \
                "colour": "0,65,180", "shape_id": None, "bounding_box": None, "size_id": 3, "image_path": "/images/object/img5.jpg", \
                "pc_path": "/point_cloud/object/pc5.txt", "features": None, "class_confidence": 0.87, "loc_confidence": 0.9, \
                "orient_confidence": 0.0, "size_confidence": 0.7, "colour_confidence": 0.95, \
                }
                #"object_relationship": [{"related_id" : 1, "object_relationship_type_id" : 2}]}

        #data_dic = json.loads(new_data)

        print(new_data)
        req.insert.details = self.__dictToKeyValueList(new_data)
        resp = self.__insert_object(req)
    
        print(self.__get_object_def_list())
        '''

    """

    def __dictToKeyValueList(self, dic):
        l = []
        for key in dic:
            # We can have NULL's in our table!
            if dic[key] is None:
                continue
            # Note ROS msgs need to be ascii strings!
            ascii_key = str(key)
            ascii_val = str(dic[key])
            l.append(KeyValue(key=ascii_key, value=ascii_val))
        return l

if __name__ == "__main__":
    client = WmClient()
    client.run()
        

