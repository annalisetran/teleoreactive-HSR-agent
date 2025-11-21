#!/usr/bin/env python3

import rospy
from unsw_vision_msgs.msg import DetectionList, ObjectDetectionList
from unsw_vision_msgs.srv import ViewObjects


class ViewObjectsServer():
    def __init__(self):
        self.vision_objects = None
        rospy.Service("/unsw_vision/view_objects", ViewObjects, self.ViewObject)
        vision_sub = rospy.Subscriber('unsw_vision/detections/objects/positions', DetectionList, self.vision_callback, queue_size=1)
        self.img = None
    def vision_callback(self, msg: DetectionList):
        detected_objects = msg.objects
        ignore_objects = ["table", "cupboard", "chair", "couch"]
        objects_filtered = list(filter(lambda myobj: myobj.object_class not in ignore_objects, detected_objects))
                
        self.vision_objects = objects_filtered
        #self.img = msg.img
    def ViewObject(self,data):
        # wait for yolo to catch up before viewing the table

        rospy.sleep(1.0)

        # save objects to a list of objectDetections
        objects = ObjectDetectionList()
        objects.header = rospy.Time.now()
        objects.objects = self.vision_objects
        while self.img is None:
            rospy.sleep(0.1)
        #objects.img = self.img
        return objects

def main():
    rospy.init_node('view_objects_server')
    ViewObjectsServer()
    rospy.spin()

if __name__ == "__main__":
    main()