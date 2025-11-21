#!/usr/bin/env python3

import rospy
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

class ImagePublisher:
    def __init__(self, image_dir, topic):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher(topic, Image, queue_size=10)
        self.image_dir = image_dir

    def publish_images(self):
        print(f"Publishing images from {self.image_dir}")
        for dirpath, dirnames, filenames in os.walk(self.image_dir):
            print(f"Found directory: {dirpath} with {len(filenames)} images")
            for filename in sorted(filenames):
                if filename.endswith(".jpg") or filename.endswith(".png"):
                    img_path = os.path.join(dirpath, filename)
                    img = cv2.imread(img_path)
                    try:
                        self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
                        print(f"Publishing {img_path}")
                    except CvBridgeError as e:
                        print(e)

if __name__ == '__main__':
    rospy.init_node('image_publisher', anonymous=True)
    image_dir = "/home/adam/tmp/PKU-Reid-Dataset/PKUv1a_128x48/"
    topic = "/test/image_topic"
    img_pub = ImagePublisher(image_dir, topic)
    rate = rospy.Rate(3)  # 1 Hz
    while not rospy.is_shutdown():
        img_pub.publish_images()
        rate.sleep()