#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

def publish_image():
    rospy.init_node('dummy_image_publisher')
    bridge = CvBridge()

    # Replace with your image path
    image_path = '/home/betty/workspace/src/objectTraining/GeneralObjectDetectionModel/evaluateModel/UnseenImages/Bottle&Can.jpg'
    image_msg = cv2.imread(image_path)
    #cv2.imshow('img', image_msg)

    if image_msg is None:
        rospy.logerr("Image not found or failed to load.")
        return

    # Use CvBridge to convert OpenCV image to ROS message
    cv_bridge = CvBridge()

    # Create a publisher
    image_pub = rospy.Publisher('camera/dummy_image', Image, queue_size=10)

    # Publish the image
    
    rate = rospy.Rate(2)  # 2 Hz
    while not rospy.is_shutdown():
        try:
            ros_image = bridge.cv2_to_imgmsg(image_msg, encoding="bgr8")
            image_pub.publish(ros_image)
            rospy.loginfo("Published image.")
        except Exception as e:
            rospy.logerr(f"Failed to convert or publish image: {e}")
        rate.sleep()

if __name__ == '__main__':
    publish_image()