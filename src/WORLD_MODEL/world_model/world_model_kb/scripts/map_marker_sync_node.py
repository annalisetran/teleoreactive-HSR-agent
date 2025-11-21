#!/usr/bin/env python
# from db import Db
# from robot_state_api import RobotStateApi
# from object_def_api import ObjectDefApi
# from object_api import ObjectApi
# from scene_api import SceneApi
# from object_class_api import ObjectClassApi

# import map_markers
# import map_markers.srv

# import rospy
# import json
# import tf

# class MapMarker:
#     def __init__(self):
#         self.__add_marker = rospy.ServiceProxy('/map_marker/add_map_marker', map_markers.srv.AddMarker)
#         self.__remove_marker = rospy.ServiceProxy('/map_marker/remove_map_marker', map_markers.srv.RemoveMarker)

#         self.__db_config_path = rospy.get_param("~database_config_path")
#         self.__db = Db(self.__db_config_path)
#         self.__obj = ObjectApi()
#         rospy.loginfo("Connected to DB!")

#         self.__known_markers = set()

#         rospy.loginfo("Waiting for map marker services")
#         self.__add_marker.wait_for_service()
#         self.__remove_marker.wait_for_service()
#         rospy.loginfo("map marker services are up!")

#     def tick(self):
#         obj_list = self.__obj.get_object_list(self.__db.con_pool)
#         if obj_list is None:
#             return

#         #obj_list = json.loads(obj_list_json)
#         curr_names = set()

#         # Add markers from WM
#         if obj_list is None or obj_list[0][0] is None:
#             return
#         for obj in obj_list[0][0]:
#             obj_dict = obj
#             add_req = map_markers.srv.AddMarkerRequest()
#             name = ""
#             try:
#                 name = obj_dict.get("name")
#                 if name is None or not name:
#                     # We dont have a name - make one!
#                     #class = 
#                     name = str(obj_dict["object_id"])

#                 #rospy.loginfo("Adding marker {}".format(name))
#                 add_req.name = name
#                 add_req.pose.pose.position.x = obj_dict["pose_x"]
#                 add_req.pose.pose.position.y = obj_dict["pose_y"]
#                 add_req.pose.pose.position.z = obj_dict["pose_z"]
#                 quat = tf.transformations.quaternion_from_euler(0, 0, 0)
#                 add_req.pose.pose.orientation.x = quat[0]
#                 add_req.pose.pose.orientation.y = quat[1]
#                 add_req.pose.pose.orientation.z = quat[2]
#                 add_req.pose.pose.orientation.w = quat[3]

#                 add_req.pose.header.frame_id = obj_dict["frame_id"]
#                 add_req.pose.header.stamp = rospy.Time.now()
#             except Exception as e:
#                 rospy.logwarn(type(e).__name__ + ": " + str(e))
#                 continue

#             self.__add_marker(add_req)
#             self.__known_markers.add(name)
#             curr_names.add(name)

#         # Remove markers not in WM anymore

#         for name in self.__known_markers:
#             if not name in curr_names:
#                 remove_req = map_markers.srv.RemoveMarker()
#                 remove_req.name = name
#                 self.__remove_marker(remove_req)

# if __name__ == "__main__":
#     rospy.init_node("world_model_map_marker_sync")
#     markers = MapMarker()

#     r = rospy.Rate(5) # 5 hz
#     while not rospy.is_shutdown():
#         markers.tick()
#         r.sleep()
