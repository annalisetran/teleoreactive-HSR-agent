import rospy
import cv2
import numpy as np
import base64
import sys

from cv_bridge import CvBridge

from unsw_vision_msgs.msg import DetectionList
from unsw_vision_msgs.srv import FindObjectFrame
# from unsw_vision_msgs.msg import DetectionList, ObjectFrame

from std_msgs.msg import String

class ObjectFramesDebug:
    def __init__(self):
        rospy.wait_for_service('/unsw_vision/object_frame_srv')
        self.findObjectFrame = rospy.ServiceProxy('/unsw_vision/object_frame_srv', FindObjectFrame)

        self.sub = rospy.Subscriber('/unsw_vision/detections/objects', DetectionList, self.detectionListCallback, queue_size=10)
        
    
    def detectionListCallback(self, detectionList: DetectionList):
        # What do i want to do?
        # recieve image and list of objects
        if len(detectionList.objects) == 0:
            return
        objectDetected = detectionList.objects[0]
        try:
            self.findObjectFrame(objectDetected.object_class, objectDetected.tracking_id)
        except rospy.ServiceException as e:
            rospy.loginfo(f"Service did not process request: {e}")

           
def main(args):
    rospy.init_node('ObjectFramesDebug', anonymous=True)
    object_frames_debug = ObjectFramesDebug()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)