#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class BrightnessChecker:
    def __init__(self):
        rospy.init_node('brightness_checker', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/hsrb/hand_camera/image_raw', Image, self.image_callback)
        rospy.loginfo("Brightness Checker Node Started. Subscribed to /hsrb/hand_camera/image_raw")

    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)
            return

        # Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Calculate average brightness
        croped_size = 140

        x1 = int(data.width / 2 - croped_size / 2)
        y1 = int(data.height / 2 - croped_size / 2)
        
        print(data.width)
        gray_cropped = gray[y1:y1 + croped_size, x1:x1 + croped_size]
        avg_brightness = np.mean(gray)
        ave_brightness_croped = np.mean(gray_cropped)
        image_with_box = cv_image.copy()
        cv2.rectangle(image_with_box, (x1, y1), (x1 + croped_size, y1 + croped_size), (0, 255, 0), 2)
        
        cv2.imshow("Original with BBox", image_with_box)
        cv2.imshow("ori",gray)
        cv2.imshow("cropped",gray_cropped)

        cv2.waitKey(1)
        rospy.loginfo("Average Brightness: {:.2f}".format(avg_brightness))
        rospy.loginfo("Average Brightness gray : {:.2f}".format(ave_brightness_croped))

if __name__ == '__main__':
    try:
        BrightnessChecker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
