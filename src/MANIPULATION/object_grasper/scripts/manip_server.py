#!usr/bin/env python3

import sys
import rospy
import actionlib
import geometry_msgs.msg
import control_msgs.msg
import trajectory_msgs.msg


from std_msgs.msg import String

from manip_srv.srv import hand_plane_move, center_object_move, wrist_rotate
from time import sleep

from unsw_vision_msgs.msg import DetectionList, ObjectDetection 


# change to launch file param or use camInfo node later
IMG_HEIGHT = 640
IMG_WIDTH = 640
# the value allow to be away from the center
CENTRE_OFFSET = 10
class ManipServer():

    def __init__(self):
        # class of the object needs to be centred
        self.tracking_obj = ""
        
        # 1 indicates needs move up to center, -1 indicates needs move down to center, 0 indicates no need to move, None indicates not yet set or obj is not seen
        # for horizontal, right is positive, left is negative
        self.vertical_move = None
        self.horizontal_move = None
        self.object_bbox_width = 0
        self.object_bbox_height = 0
        rospy.wait_for_service('/unsw/manip/hand_plane_move')
        rospy.wait_for_service('/unsw/manip/wrist_rotate')
        
        
        rospy.Service("/unsw/manip/hand_plane_move/center_object", center_object_move, self.tuneToCenter)

        pass
    
    '''
    Function to set the self.trackingObj to the object that needs to be centred
    '''    
    def tuneToCenter(self,data):
        # if trakcing obj is being set(not empty) and trakcing has not been started, starts it
        step = 0.03
        rospy.loginfo("receive")
        self.tracking_obj = data.object_class
        x = rospy.Subscriber("/unsw_vision/handcam/detections/objects", DetectionList, self.visionCB)
        end_effector_move = rospy.ServiceProxy('/unsw/manip/hand_plane_move', hand_plane_move)
        lastTimeSeen = rospy.get_time()
        while self.vertical_move != 0 or self.horizontal_move != 0:
            rospy.loginfo("move")
            if self.vertical_move == 1:
                rospy.loginfo("move up")
                end_effector_move("up",step)
                rospy.loginfo("move up done")
                lastTimeSeen = rospy.get_time()
            elif self.vertical_move == -1:
                rospy.loginfo("move down")
                end_effector_move("down",step)
                rospy.loginfo("move down done")
                lastTimeSeen = rospy.get_time()
            if self.horizontal_move == 1:
                rospy.loginfo("move right")
                end_effector_move("right",step)
                rospy.loginfo("move right done")
                lastTimeSeen = rospy.get_time()
            elif self.horizontal_move == -1:
                rospy.loginfo("move left")
                end_effector_move("left",step)
                rospy.loginfo("move left done")
                lastTimeSeen = rospy.get_time()
            else:
                # if here, both of them are None
                # if the object is not seen for more than 5 seconds, return False
                rospy.loginfo("Object not seen cannot move")
                if rospy.get_time() - lastTimeSeen > 10:
                    x.unregister()
                    return False
            rospy.sleep(0.5)
        x.unregister()
        rospy.loginfo("Object is centered")
        self.wrist_rotate_helper()
        end_effector_move("up",0.045)
        return True
            
    def wrist_rotate_helper(self):
        if self.object_bbox_width == None or self.object_bbox_height == None:
            rospy.loginfo("Object not seen, will not rotate wrist")
            return False

        wrist_rotate_srv = rospy.ServiceProxy('/unsw/manip/wrist_rotate', wrist_rotate)
        if self.object_bbox_width > self.object_bbox_height * 1.5:
            wrist_rotate_srv()
        return True

    
    def visionCB(self,data):
        # if cannot find the object in the frame, set both the vertical and horizontal move to None

        obj_position_x = None
        obj_position_y = None
        for obj_detection in data.objects:
            if obj_detection.object_class == self.tracking_obj:
                obj_position_x = obj_detection.bbox.x + obj_detection.bbox.width//2
                obj_position_y = obj_detection.bbox.y + obj_detection.bbox.height//2
                self.object_bbox_width = obj_detection.bbox.width
                self.object_bbox_height = obj_detection.bbox.height
        
        if obj_position_x == None or obj_position_y == None:
            self.vertical_move = None
            self.horizontal_move = None
            rospy.loginfo("Object not found in the frame")
        else:
            # check condition for weather it is centered. If it is CENTER_OFFSET away from center
            # considered it as centered, otherwise, set the move value
            if abs(obj_position_x - IMG_WIDTH//2) < CENTRE_OFFSET:
                self.horizontal_move = 0
            else:
                # if the object is on the right side of the center(has a higher pixcel value), move right
                self.horizontal_move = 1 if obj_position_x > IMG_WIDTH//2 else -1
            if abs(obj_position_y - IMG_HEIGHT//2) < CENTRE_OFFSET:
                self.vertical_move = 0
            else:
                # if the object is below the center(has a higher pixcel value), move down
                self.vertical_move = -1 if obj_position_y > IMG_HEIGHT//2 else 1
    
    

                
        
if __name__=='__main__':
    rospy.init_node('manip_server')  
    ManipServer()
     
    rospy.spin()
    # main()        