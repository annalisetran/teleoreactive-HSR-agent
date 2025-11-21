#!/usr/bin/env python3
import rospy
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
from unsw_vision_msgs.msg import DetectionList,ObjectDetection, BoundingBox
from sensor_msgs.msg import Image
from matplotlib import pyplot as plt

class BagHandleDetect:

    def __init__(self): 

        rospy.loginfo("initialise baghandle detector")
        # if not detectionIn_topic:
        #     rospy.loginfo("No detection in topic for BagHadleIdentify")
        # if not detectionOut_topic:
        #     rospy.loginfo("No detection output topic for BagHadleIdentify")
        
        #TODO: paraterised following line
        self.count = 0
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_rect_color", Image, self.save)

    
    def save(self, o_img):
        self.count+=1
        if self.count % 10 == 0:
            #do the save
            cv_img = self.bridge.imgmsg_to_cv2(o_img, desired_encoding='bgr8')
            rospy.loginfo("saving it to")
            rospy.loginfo(f"/home/blinky/compimg/item11_{self.count//10}.jpg")
            cv2.imwrite(f"/home/blinky/compimg/item11_{self.count//10}.jpg", cv_img)

def main(args):
    rospy.init_node('ImageSaver', anonymous=True)
    obc = BagHandleDetect()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)