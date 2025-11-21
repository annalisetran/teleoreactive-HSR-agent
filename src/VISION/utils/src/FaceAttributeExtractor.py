#!/usr/bin/env python3

import rospy
import numpy as np
from deepface import DeepFace
import cv2

# img = cv2.imread("/home/robocupathome/Downloads/img1.jpeg")
# print(img)
# objs = DeepFace.analyze(img_path= img, 
#         actions = ['age', 'gender']
# )
# print(objs)


'''
# Request
sensor_msgs/Image img
---
# Response
string age
string gender
'''

'''
A service that extract features(age and gender) for given person, as a service.
'''

class FaceAttributeExtractor:
    def __init__(self) -> None:
        pass
    
    def feature_extract(img)
    
    
if __name__ == "__main__":
    rospy.init_node('face_attribute_extractor')
    obj = FaceAttributeExtractor()
    