#!/usr/bin/env python

from world_model_kb_msgs.srv import Insert, Get, InsertResponse, GetResponse
from world_model_kb_msgs.msg import Details
from diagnostic_msgs.msg import KeyValue

from db import Db
from robot_state_api import RobotStateApi
from object_def_api import ObjectDefApi
from object_api import ObjectApi
from scene_api import SceneApi
from object_class_api import ObjectClassApi
from regions_api import RegionsApi

import rospy
import json

class WmServer:
    def __init__(self):
        rospy.init_node('world_model_kb')

        self.__getParams()

        rospy.loginfo("Setting up world_model_kb...")
        # Setup connection to DB
        self.__db = Db(self.__db_config_path)
        rospy.loginfo("Connected to DB!")

        self.__robot_state = RobotStateApi()
        self.__obj_def = ObjectDefApi()
        self.__obj = ObjectApi()
        self.__scene_api = SceneApi()
        self.__obj_class_api = ObjectClassApi()
        self.__regions_api = RegionsApi()
        rospy.loginfo("\tConstructed all DB API's")


        # Setup services
        service_ns = "/world_model"
        # Map of service topic -> service type and handler
        service_map = {
                "get_object": [Get, self.handleGetObject],
                "get_object_def": [Get, self.handleGetObjectDef],
                "get_object_def_list": [Get, self.handleGetObjectDefList],
                "get_object_list": [Get, self.handleGetObjectList],
                "get_robot_state_history": [Get, self.handleGetRobotStateHistory],
                "get_robot_state_current": [Get, self.handleGetRobotStateCurrent],
                "insert_object": [Insert, self.handleInsertObject],
                "insert_object_def": [Insert, self.handleInsertObjectDef],
                "insert_robot_state": [Insert, self.handleInsertRobotState],
                "insert_scene": [Insert, self.handleInsertScene],
                "insert_scene_object": [Insert, self.handleInsertSceneObject],
                "insert_region": [Insert, self.handleInsertRegion],
                "find_point_region": [Get, self.handleFindPointRegion],
                "get_regions": [Get, self.handleGetRegions]
                }

        # Create a list of services from the map
        self.__services = []
        for key in service_map:
            service_name = service_ns + "/" + key
            handler = service_map[key][1]
            service_type = service_map[key][0]
            self.__services.append(rospy.Service(service_name, service_type, handler))
            rospy.loginfo("\tSetup service " + service_name)

        rospy.loginfo("\tSetup all services... world_model_kb ready!")
        
    def __del__(self):
        pass


    def start(self):
        pass

    def handleFindPointRegion(self, request):
        data_json = json.loads(request.criteria.json)
        x = data_json["x"]
        y = data_json["y"]

        return self.__handleGetList(request, lambda pool, x=x, y=y: self.__regions_api.find_point_label(pool, x, y))

    def handleInsertRegion(self, request):
        data_json = json.loads(request.insert.json)
        label = data_json["label"]
        points_x = data_json["points_x"]
        points_y = data_json["points_y"]
        neighbors = data_json["neighbors"]
        region_id = data_json["region_id"]

        region_id = self.__regions_api.insert_region(self.__db.con_pool, label, points_x, points_y, neighbors=neighbors, region_id=region_id)

        # Generate response
        response = InsertResponse()
        response.status = response.SUCCESS
        response.inserted_id = int(region_id)
        if region_id == -1:
            # DB failed to insert item
            response.status = response.FAIL

        return response

    def handleGetRegions(self, request):
        return self.__handleGetList(request, self.__regions_api.get_regions)


    '''
        Insert a new scene into the DB
        Parameters:
            request: world_model_kb_msgs.srv.InsertRequest object
                     request.insert.json contains a json formatted string containing "robot_state_id: value"
            returns: world_model_kb_msgs.srv.InsertResponse object
                if the insert was successful 
                    inserted_id is the scene_id
                else if the insert failed
                    inserted_id is -1        
    '''
    def handleInsertScene(self, request):
        data_json = json.loads(request.insert.json)
        robot_state_id = data_json[Details.ROBOT_STATE_ID]
        scene_id = self.__scene_api.insert_scene(self.__db.con_pool, robot_state_id)

        # Generate response
        response = InsertResponse()
        response.status = response.SUCCESS
        response.inserted_id = int(scene_id)
        if scene_id == -1:
            # DB failed to insert item
            response.status = response.FAIL

        return response


    '''
        Insert a new scene object into the DB
        Parameters:
            request: world_model_kb_msgs.srv.InsertRequest object
                     request.insert.json contains a json formatted string containing:
                        "scene_id: value"
                      if request.insert.json contains a "object_category" and/or "object_sub_category" will lookup the object_class_id for you

            returns: world_model_kb_msgs.srv.InsertResponse object
                if the insert was successful 
                    inserted_id is the scene_object_id
                else if the insert failed
                    inserted_id is -1        
    '''
    def handleInsertSceneObject(self, request):
        data_json = json.loads(request.insert.json)
        scene_id = data_json[Details.SCENE_ID]

        """ No longer need to lookup the class id
        # Lookup the object_class_id
        obj_class_id = None
        obj_cat = data_json.get("object_category")
        obj_sub_cat = data_json.get("object_sub_category")
        if not (obj_cat is None):
            obj_class = {}
            if obj_sub_cat is None:
                # Lookup by class
                obj_class = self.__obj_class_api.get_object_class_by_cat(
                        self.__db.con_pool,
                        obj_cat)[0][0][0]
            else:
                # Lookup by class + sub-class
                obj_class = self.__obj_class_api.get_object_class_by_cat_sub(
                        self.__db.con_pool, obj_cat, obj_sub_cat)[0][0][0]
            #print(obj_class)
            obj_class_id = obj_class.get("object_class_id")

            data_json.update({"object_class_id": obj_class_id})
            print("Lookup class {} sub class {} gives id: {}".format(obj_cat, obj_sub_cat, obj_class_id))
        """

        # Do the insert
        inserted_id = self.__scene_api.insert_scene_object(self.__db.con_pool, json.dumps([data_json]), scene_id)

        # Generate response
        response = InsertResponse()
        response.status = response.SUCCESS
        response.inserted_id = int(inserted_id)
        if inserted_id == -1:
            # DB failed to insert item
            response.status = response.FAIL

        return response



    """"
    Handlers for each reported service
    Delagate to generalised functions
    """


    def handleGetObject(self, request):
        return self.__handleGet(request, Details.OBJ_ID, self.__obj.get_object_by_id)

    def handleGetObjectList(self, request):
        return self.__handleGetList(request, self.__obj.get_object_list)

    def handleGetObjectDefList(self, request):
        return self.__handleGetList(request, self.__obj_def.get_object_def_list)

    def handleGetObjectDef(self, request):
        return self.__handleGet(request, Details.OBJ_DEF_ID, self.__obj_def.get_object_def)

    def handleGetRobotStateHistory(self, request):
        return self.__handleGetList(request, self.__robot_state.get_robot_state_history)
        
    def handleGetRobotStateCurrent(self, request):
        return self.__handleGetList(request, self.__robot_state.get_robot_state_current)

    def handleInsertObject(self, request):
        return self.__handleInsert(request, self.__obj.insert_object)

    def handleInsertObjectDef(self, request):
        return self.__handleInsert(request, self.__obj_def.insert_object_def)

    def handleInsertRobotState(self, request):
        return self.__handleInsert(request, self.__robot_state.insert_robot_state)


    """
    General functions to handle inserts and gets
    """

    def __handleInsert(self, request, db_api_handle):
        """
        params: 
            db_api_handle: callable with args (connection_pool, json_string)
        """

        response = InsertResponse()

        data_json = request.insert.json
        # For some reason the db api wants lists of json objects.
        data_json_list = [json.loads(data_json)]
        inserted_id = db_api_handle(self.__db.con_pool, json.dumps(data_json_list))

        response.status = response.SUCCESS
        response.inserted_id = int(inserted_id)
        if inserted_id == -1:
            # DB failed to insert item
            response.status = response.FAIL

        return response


    def __handleGetList(self, request, db_api_handle):
        """
        params: 
            db_api_handle: callable with args (connection_pool)
        """
        resp = GetResponse()

        try:
            db_resp = db_api_handle(self.__db.con_pool)
            # ugh...
            rows = db_resp[0][0]
            for row in rows:
                resp.matches.append(Details(json=json.dumps(row)))

            resp.status = resp.SUCCESS
        except Exception as e:
            resp.status = resp.FAIL
            resp.error_msg = e.message
            rospy.logwarn(e)
        finally:
            return resp


    def __handleGet(self, request, id_field, db_api_handle):
        """
        Get a single entry from the db with id = request[id_field]
        params: 
            db_api_handle: callable with args (connection_pool, object_id)
        """

        # Strip object id out of request
        req_json = json.loads(str(request.criteria.json))
        
        obj_id = req_json[id_field]

        resp = GetResponse()

        try:
            db_resp = db_api_handle(self.__db.con_pool, obj_id)
            # We want just the first match
            if db_resp[0][0] is None:
                resp.status = resp.FAIL
                resp.error_msg = "{} {} does not exist".format(id_field, obj_id)
            else:
                resp.matches.append(Details(json=json.dumps(db_resp[0][0][0])))
                resp.status = resp.SUCCESS
        except Exception as e:
            resp.status = resp.FAIL
            resp.error_msg = e.message
            rospy.logwarn(e)
        finally:
            return resp

    def __getParams(self):
        self.__db_config_path = rospy.get_param("~database_config_path")

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

    def __keyValueListToDict(self, key_values):
        dic = {}
        for key_val in key_values:
            dic[key_val.key] = key_val.value
        return dic
        
if __name__ == "__main__":
    server = WmServer()
    server.start()
    rospy.spin()
    
