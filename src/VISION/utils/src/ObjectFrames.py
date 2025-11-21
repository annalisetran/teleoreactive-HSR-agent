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

class ObjectFrames:
    def __init__(self):
        rospy.loginfo("Object Frames: Node started")
        rospy.Service('/unsw_vision/object_frame_srv', FindObjectFrame, self.findObjectFrame)
        # self.pub = rospy.Publisher('/unsw_vision/object_frame', ObjectFrame, queue_size=10)
        self.imagePub = rospy.Publisher('/unsw_vision/object_frame/image', String, queue_size=10)
        self.classPub = rospy.Publisher('/unsw_vision/object_frame/object_class', String, queue_size=10)

        self.bridge = CvBridge()
    
    def findObjectFrame(self, data):
        rospy.loginfo(f"Finding Object Frame -> id: {data.tracking_id}; class: {data.object_class}")

        detectionList = rospy.wait_for_message('/unsw_vision/detections/objects', DetectionList)
        matchedObjects = list(filter(lambda detectedObject: detectedObject.tracking_id == data.tracking_id, detectionList.objects))
        if len(matchedObjects) == 0:
            rospy.loginfo("Object Frames: Object not found")
            matchedObjects = list(filter(lambda detectedObject: detectedObject.object_class == data.object_class, detectionList.objects))
            if len(matchedObjects) == 0:
                return False
        objectDetected = matchedObjects[0]
        rospy.loginfo("Object frames: Object detected")

        bbox = objectDetected.bbox
        cv_image = self.bridge.imgmsg_to_cv2(detectionList.image, desired_encoding='bgr8')
        
        # cropped_image = cv_image[int(bbox.y):int(bbox.y + bbox.height), int(bbox.x):int(bbox.x + bbox.width)]

        blurred_image = cv2.GaussianBlur(cv_image, (41, 41), 0)
        blurred_image[int(bbox.y):int(bbox.y + bbox.height), int(bbox.x):int(bbox.x + bbox.width)] = cv_image[int(bbox.y):int(bbox.y + bbox.height), int(bbox.x):int(bbox.x + bbox.width)]

        outlined_image = cv2.rectangle(blurred_image, (bbox.x, bbox.y), (bbox.x + bbox.width, bbox.y + bbox.height), (22, 61, 214), 2)

        _, im_arr = cv2.imencode('.jpg', outlined_image)
        im_bytes = im_arr.tobytes()
        im_b64 = base64.b64encode(im_bytes).decode('utf-8')
        im_b64 = "data:image/png;base64," + im_b64

        self.imagePub.publish(im_b64)
        self.classPub.publish(objectDetected.object_class)
        self.detectionList = None
        return True


           
def main(args):
    rospy.init_node('ObjectFrames', anonymous=True)
    object_frames = ObjectFrames()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)