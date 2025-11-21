#!/usr/bin/env python3

import rospy
import math
import smach
import smach.state
import smach_ros
import tf2_ros
import tf2_geometry_msgs
import actionlib
import tf
from cv_bridge import CvBridge
from std_msgs.msg import String, Int8, Bool
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PointStamped, Point, Pose, Quaternion, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from manip_srv.srv import approach_object, grasp_object, placing_object, center_object_move, view_pose
from unsw_vision_msgs.msg import DetectionList
from unsw_vision_msgs.srv import ViewObjects

# import os
# import sys
# module_path = os.environ.get("UNSW_WS")
# sys.path.append(module_path + "/VISION/utils/src")
# from ObjectFeatureExtractor import ObjectFeatureExtractor 

TABLE_VIEW_POSE = Pose(Point(-7.109, -8.67, 0.000),Quaternion(0.000, 0.000, -0.719, 0.695))
TABLE_PICK_UP_POSE = Pose(Point(-6.66, -7.738, 0.0),Quaternion(0, 0, -0.999923, 0.0124))
CABINET_POSE = Pose(Point(-6.396, -8.413, 0), Quaternion(0,0,-0.7, 0.714))

class Talker:
    def __init__(self):
        tts_topic = rospy.get_param('/store_groceries_controller/tts_topic')
        self.talker = rospy.Publisher(tts_topic, String, queue_size=1)
    
    def talk(self, sentence):
        rospy.loginfo(f"saying: {sentence}")
        self.talker.publish(sentence)

class Start(smach.State):
    def __init__(self, myTalker, tfBuffer):
        smach.State.__init__(self, outcomes=['task_started', 'waiting'])
        self.myTalker = myTalker
        start_topic = rospy.get_param("/store_groceries_controller/start_topic")
        self.task_started = False
        rospy.loginfo("init wrist sub callback")
        self.wrist_sub = rospy.Subscriber(start_topic, Int8, callback=self.wrist_callback, queue_size=1)

    def wrist_callback(self, msg: Int8):
        rospy.logwarn("Task started!")
        self.task_started = True

    def execute(self, userdata):
        rospy.loginfo("Waiting for start")
        if self.task_started:
            self.myTalker.talk("Storing Groceries task started. Waiting for the door to be opened.")
            return 'task_started'
        return 'waiting'
    
class DoorCheck(smach.State):
    def __init__(self, myTalker):
        smach.State.__init__(self, outcomes=['door_open', 'door_closed'])
        self.myTalker = myTalker
        self.door_open = False
        
        self.door_sub = rospy.Subscriber('/door_open_close', String, callback=self.doors_callback, queue_size=10)
        
    def doors_callback(self, msg: String):
        if msg.data == 'open':
            self.door_open = True

    def execute(self, userdata):
        if self.door_open:
            self.myTalker.talk("I detected the door opening.")
            return 'door_open'
        if not self.door_open:
            return 'door_closed'
        
class MoveToTable(smach.State):
    def __init__(self, myTalker):
        smach.State.__init__(self, outcomes=['move_to_table_success', 'move_to_table_fail'])

        self.myTalker = myTalker
        self.cli = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        
        # if nothing heard back from the server in 5 seconds then kill the node
        if not self.cli.wait_for_server(rospy.Duration(5.0)):
            rospy.loginfo("Timeout waiting for /move_base/move action server")
            assert False, "Timeout waiting for action server"

        self.table_view_pose = TABLE_VIEW_POSE
        
    def execute(self, ud):
        self.myTalker.talk("I moving to the table now.")

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"

        pose.pose = self.table_view_pose

        goal = MoveBaseGoal()
        goal.target_pose = pose

        self.cli.send_goal_and_wait(goal)

        navigation_state = self.cli.get_state()
        if navigation_state == GoalStatus.SUCCEEDED:
            self.myTalker.talk("I have reached the table")
            return 'move_to_table_success'
        if navigation_state == GoalStatus.ABORTED or navigation_state == GoalStatus.REJECTED:
            return 'move_to_table_fail'

class ViewTable(smach.State):
    def __init__(self, myTalker, tfBuffer):
        smach.State.__init__(self, outcomes=['object_identified'],
                             output_keys=['object', 'num_attempts'])
        
        self.myTalker = myTalker
        self.tfBuffer = tfBuffer
        
        self.bridge = CvBridge()
        rospy.wait_for_service("/unsw_vision/view_objects", timeout=15)
        self.view_objects_service = rospy.ServiceProxy("/unsw_vision/view_objects", ViewObjects)

        rospy.wait_for_service("/unsw/manip/view_pose", timeout=15)
        self.view_pose_service = rospy.ServiceProxy("/unsw/manip/view_pose", view_pose)

        self.object_class = None
        self.object_id = None

    def execute(self, ud):
        # move to view joint state
        arm_lift_joint = 0.48
        head_tilt_joint = -0.28
        self.view_pose_service(arm_lift_joint, head_tilt_joint)


        view_result = self.view_objects_service(0)

        table_objects = view_result.objects
        #cv_img = self.bridge.imgmsg_to_cv2(view_result.objects.img, desired_encoding='bgr8')
        self.myTalker.talk("I'm deciding which object to put away.")

        # tf to camera frame and choose the closest x value
        closest_distance = 400
        closest_object = table_objects[0]
        for i, object in enumerate(table_objects):
            # tf to camera frame
            object_pose_stamped = PoseStamped()
            object_pose_stamped.header.seq = 0
            object_pose_stamped.header.time = rospy.Time.now()
            object_pose_stamped.header.frame_id = "map"

            object_pose_stamped.pose.position = object.pose.position
            
            quat = tf.transformations.quaternion_from_euler(object.bbox3d.center.orientation)
            object_pose_stamped.pose.orientation = Quaternion(*quat) 

            try:
                trans = self.tfBuffer.lookup_transform("map","head_rgbd_sensor_link",rospy.Time.now(), rospy.Duration(2))
                object_pose_stamped = tf2_geometry_msgs.do_transform_pose(object_pose_stamped, trans)
            except Exception as e:
                rospy.loginfo(f"Failed transform point to head_rgbd_sensor_link. {e}")
                continue
            
            # check if closest_distance is less than other ones
            if object_pose_stamped.position.x < closest_distance:
                closest_distance = object_pose_stamped.position.x
                closest_object = object

        ud.object = closest_object

        # TODO: get features of object out 2d bounding box
        # get the 2d bounding box
        bbox = closest_object.bbox2d
        #cropped_obj = cv_img[bbox.y:bbox.y+bbox.height, bbox.x:bbox.x+bbox.width]

        feature = 1
        ud.num_attempts = 0
        rospy.logwarn("finished")
        return 'object_identitified'
    
class PickUp(smach.State):
    def __init__(self, myTalker):
        smach.State.__init__(self, outcomes=['pick_up_success', 'pick_up_fail'],
                             input_keys=['object', 'num_attempts'],
                             output_keys=['num_attempts'])
        
        self.myTalker = myTalker

        self.cli = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        # if nothing heard back from the server in 5 seconds then kill the node
        if not self.cli.wait_for_server(rospy.Duration(5.0)):
            rospy.loginfo("Timeout waiting for /move_base/move action server")
            assert False, "Timeout waiting for action server"
        
        rospy.wait_for_service('unsw/manip/approach_object', timeout=15)
        self.approach_service = rospy.ServiceProxy('unsw/manip/approach_object', approach_object)

        rospy.wait_for_service('/unsw/manip/hand_plane_move/center_object', timeout=15)
        self.center_service = rospy.ServiceProxy('unsw/manip/hand_plane_move/center_object', center_object_move)

        rospy.wait_for_service('unsw/manip/grasp_object', timeout=15)
        self.grasp_object_service = rospy.ServiceProxy('unsw/manip/grasp_object', grasp_object)
        
        self.pick_up_pose = TABLE_PICK_UP_POSE
    def execute(self, ud):
        # nav to pickup- position
        pose = PoseStamped()
        pose.header.seq = 0
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose = self.pick_up_pose
        goal = MoveBaseGoal()
        goal.target_pose = pose
        result = self.cli.send_goal_and_wait(goal)

        if self.cli.get_state() == GoalStatus.ABORTED:
            return 'pick_up_fail'

        self.myTalker.talk("I am now picking up the object")

        point_stamped = PointStamped((0, rospy.Time.now(), 'map'),(ud.object.position))

        approach_result = self.approach_service('front', point_stamped)
        if not approach_result.success:
            ud.num_attempts += 1
            return 'pick_up_fail'
        
        # hand centering - TODO check when this returns success!!
        center_result = self.center_service(ud.object.object_class)
        if not center_result.success:
            ud.num_attempts += 1
            return 'pick_up_fail'
        
        # grasping based off forward distance
        grasp_result = self.grasp_object_service(approach_result.forward_distance)
        if grasp_result.success:
            ud.num_attempts = 0
            return 'pick_up_success'
        else:
            ud.num_attempts += 1
            return 'pick_up_fail'

class PickUpReset(smach.State):
    def __init__(self, myTalker):
        smach.State.__init__(self, outcomes=['reset', 'new_object'],
                             input_keys=['num_attempts'],
                             output_keys=['num_attempts'])
        
        self.myTalker = myTalker
    
    def execute(self, ud):
        if ud.num_attempts <= 1:
            self.myTalker.talk("let me try again")
            return 'reset'
        else:
            self.myTalker.talk("I am going to choose something else")
            return 'new-object'
        
class MoveToCabinet(smach.State):
    def __init__(self, myTalker):
        smach.State.__init__(self,  outcomes=['nav_fail', 'nav_success'])
        
        self.myTalker = myTalker

        self.cli = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        if not self.cli.wait_for_server(rospy.Duration(5.0)):
            rospy.loginfo("Timeout waiting for /move_base/move action server")
            assert False, "Timeout waiting for action server"

        self.cabinet_pick_up_pose = CABINET_POSE

    def execute(self, ud):
        self.myTalker.talk("I am moving to the cabinet now")

        pose = PoseStamped()
        pose.header.seq = 0
        pose.header.stamp = rospy.Time.now()
        pose.header.frame = "map"
        pose.pose = self.cabinet_pick_up_pose

        goal = MoveBaseGoal()
        goal.target_pose = pose

        self.cli.send_goal_and_wait(goal)

        navigation_state = self.cli.get_state()
        if navigation_state == GoalStatus.SUCCEEDED:
            self.myTalker.talk("I have reached the cabinet")
            return 'nav_success'
        if navigation_state == GoalStatus.ABORTED or navigation_state == GoalStatus.REJECTED:
            return 'nav_fail'   

class ViewCabinet(smach.State):
    def __init__(self, myTalker):
        smach.State.__init__(self, outcomes=['scanning', 'nearby_object_identified'],
                             output_keys=['nearby_object', 'num_attempts', 'object_shelf'])

        rospy.wait_for_service('unsw_vision/view_objects', timeout=15)
        self.view_objects_service = rospy.ServiceProxy('unsw_vision/view_objects', ViewObjects)

        rospy.wait_for_service('unsw/manip/view_pose', timeout=15)
        self.view_pose_service = rospy.ServiceProxy('unsw/manip/view_pose', view_pose)

    # helper to return object_base
    def object_base(self, bbox3d):
        object_base_position = bbox3d.center
        object_base_position.center.position.z = object_base_position.center.position.z - 1/2 * bbox3d.size.z
        return object_base_position

    # helper to find similar z values in dictionary
    def similar_z_keys(self, z, z_keys, tolerance):
        for key in z_keys:
            if math.isclose(z, key, abs_tol=tolerance):
                return key
        return None
    
    def create_shelves(self, objects_list):
        # group similar base measurements into shelves
        shelf_dict = {}
        unique_z_values = []
        tolerance = 0.10 # TODO change if needed!!!
        for object in objects_list:
            object_base_val = self.object_base(object.bbox3d)
            similar_z = self.similar_z_keys(self.object_base.position.z, unique_z_values, tolerance)
            if similar_z is not None:
                # add to existing shelf
                shelf_index = unique_z_values.index(similar_z)
                shelf_dict[shelf_index].append(object)
            else:
                # add new z value and create new shelf index
                unique_z_values.append(self.object_base.position.z)
                new_index = len(unique_z_values) - 1
                shelf_dict[new_index] = [object]

        sorted_z_values = sorted(unique_z_values)
        ordered_shelf_dict = {}
        for new_i, z in enumerate(sorted_z_values):
            old_i = unique_z_values.index(z)
            ordered_shelf_dict[new_i] = shelf_dict[old_i]

        return ordered_shelf_dict 
    
    # helper to decide what category an object is in
    def object_category(self, object):
        object_categories = {}
        object_categories['fruit'] = ['strawberry', 'orange', 'broccoli', 'carrot']
        object_categories['dishes'] = ['cup', 'fork', 'knife', 'spoon', 'bowl']
        object_categories['food'] = ['bottle', 'cup'] # pea soup can is a cup < 0.5 confidence
        object_categories['drinks'] = ['bottle'] # big cola > 0.8 confidence
        object_categories['snacks'] = ['bottle'] # pringles < 0.6

        object_category = []
        for i, category in enumerate(object_categories):
            if object.object_class in category:
                object_category.append(object_categories.keys()[i])
        
        #TODO add feature extraction
        return object_category[0]
        return object_category

    def execute(self, ud):
        arm_lift_joint = 0.44
        head_tilt_joint_up = -0.11
        head_tilt_joint_down = -0.30

        # move head to look up
        self.view_pose_service(arm_lift_joint, head_tilt_joint_up)
        objects_list = self.view_objects_service().objects

        # move head to look down
        self.view_pose_service(arm_lift_joint, head_tilt_joint_down)
        objects_list += self.view_objects_service().objects

        cupboard_dict  = self.create_shelves(objects_list)

        # find shelf with similar objects 
        category = self.object_category(ud.object)

        object_shelf = None
        for shelf_num, shelf_objects in enumerate(cupboard_dict):
            for shelf_object in shelf_objects:
                shelf_category = self.object_category(shelf_object)
                if shelf_category == category:
                    object_shelf = shelf_num
                    self.myTalker.talker(f"The object should be placed on shelf number {object_shelf}")
                    ud.nearby_object = shelf_object
                    ud.object_shelf = object_shelf
                    return 'nearby_object_identified'
        
        if object_shelf == None:
            return 'scanning'

class PlaceObject(smach.State):
    def __init__(self, myTalker, tfBuffer):
        smach.State.__init__(self, outcomes=['place_fail', 'place_success'],
                             input_keys=['nearby_object', 'object'])

        self.myTalker = myTalker

        rospy.wait_for_service('unsw/manip/placing_down/table', timeout=15)
        self.placing_service = rospy.ServiceProxy('unsw/manip/placing_down/table', placing_object)

        self.tfBuffer = tfBuffer

    def execute(self, ud):
        # transform nearby object to base_link
        nearby_object_point_stamped = PointStamped((0,rospy.Time.now(), 'map'),ud.nearby_object.position)

        try:
            trans = self.tfBuffer.lookup_transform("map","base_link",rospy.Time.now(), rospy.Duration(2))
            nearby_object_pose_stamped = tf2_geometry_msgs.do_transform_point(nearby_object_point_stamped, trans)
            is_point_transformed = True
        except Exception as e:
            rospy.loginfo("Failed transform point to head_rgbd_sensor_link. {}".format(e))
            return 'place_fail'

        # robot is center 0
        place_point_stamped = nearby_object_point_stamped
        if nearby_object_point_stamped.point.y > 0:
            place_point_stamped.point.y = nearby_object_point_stamped - 0.2
        if nearby_object_point_stamped <= 0:
            place_point_stamped.point.y = nearby_object_point_stamped + 0.2

        placing_result = self.placing_service(place_point_stamped, "front")
        if placing_result.success:
            return 'place_success'
        else:
            self.myTalker.talk("Let me try a different object")
            return 'place_fail'
    
class PlaceHelp(smach.State):
    def __init__(self, myTalker):
        smach.State.__init__(self, outcomes=['object_given'], input_keys=['object_shelf'])
        self.myTalker = myTalker

        self.open_pub = rospy.Publisher('/open_grasp', String, queue_size=1)

    def execute(self, ud):
        self.myTalker.talk("I couldn't place the object. I will release in 10 seconds")
        rospy.sleep(13)
        self.open_pub.publish('open')
        return 'object_given'

    
def main():
    rospy.init_node('store_groceries_smach')

    myTalker=Talker()
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    #myObjectFeatureExtractor = ObjectFeatureExtractor()
    sm = smach.StateMachine(outcomes = ["task_success"])

    with sm:
        smach.StateMachine.add('START', 
                               Start(myTalker, tfBuffer), 
                               transitions={'task_started':'DOOR_CHECK', 
                                            'waiting': 'START'})
        smach.StateMachine.add('DOOR_CHECK',
                               DoorCheck(myTalker),
                               transitions={'door_open':'MOVE_TO_TABLE',
                                            'door_closed':'DOOR_CHECK'})
        smach.StateMachine.add('MOVE_TO_TABLE',
                               MoveToTable(myTalker),
                               transitions={'move_to_table_fail':'MOVE_TO_TABLE',
                                            'move_to_table_success':'VIEW_TABLE'})
        smach.StateMachine.add('VIEW_TABLE',
                               ViewTable(myTalker, tfBuffer),
                               transitions={'object_identified':'PICK_UP'})
        smach.StateMachine.add('PICK_UP',
                               PickUp(myTalker),
                               transitions={'pick_up_fail':'PICK_UP_RESET',
                                            'pick_up_success':'MOVE_TO_CABINET'})
        smach.StateMachine.add('PICK_UP_RESET',
                               PickUpReset(myTalker),
                               transitions={'new_object':'VIEW_TABLE',
                                            'reset':'MOVE_TO_TABLE'})
        smach.StateMachine.add('MOVE_TO_CABINET',
                               MoveToCabinet(myTalker),
                               transitions={'nav_fail':'MOVE_TO_CABINET',
                                            'nav_success':'MOVE_TO_CABINET'})
        smach.StateMachine.add('VIEW_CABINET', 
                               ViewCabinet(myTalker),
                               transitions={'scanning':'VIEW_CABINET',
                                           'nearby_object_identified':'PLACE_OBJECT'})
        smach.StateMachine.add('PLACE_OBJECT',
                               PlaceObject(myTalker, tfBuffer),
                               transitions={'place_fail':'PLACE_HELP',
                                            'place_success':'MOVE_TO_TABLE'})
        smach.StateMachine.add('PLACE_HELP',
                               PlaceHelp(myTalker),
                               transitions={'object_given':'MOVE_TO_TABLE'})

        # Create and start the introspection server
        sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
        sis.start()
        
        outcome = sm.execute()

        rospy.spin()
        sis.stop()

if __name__ == '__main__':
    main()

