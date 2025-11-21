#!/usr/bin/env python
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
        detectionIn_topic = rospy.get_param("/BagHandleDetector/in_topic")
        detectionOut_topic = rospy.get_param("/BagHandleDetector/out_topic")
        rospy.loginfo(detectionIn_topic+' '+ detectionOut_topic)
        # if not detectionIn_topic:
        #     rospy.loginfo("No detection in topic for BagHadleIdentify")
        # if not detectionOut_topic:
        #     rospy.loginfo("No detection output topic for BagHadleIdentify")
        
        #TODO: paraterised following line
        self.bridge = CvBridge()
        self.type = "not_looped_handle"
        self.sub = rospy.Subscriber(detectionIn_topic, DetectionList, self.appendHandlebbox)
        self.pub = rospy.Publisher(detectionOut_topic, DetectionList, queue_size=10)
    def appendHandlebbox(self, msg:DetectionList):
        handles = []
        # rospy.loginfo("in the cb for handle detector")
        for ob in msg.objects:
            if ob.object_class in ["handbag", "suitcase", "backpack"]:
                rospy.loginfo("BAG FOUND")
                try:
                    coordinates = self.HandlePoint(msg.image, ob.bbox)
                    if coordinates == None: continue
                except:
                    continue
                handleOb = ObjectDetection()
                handleOb.object_class = "baghandle"
                handleOb.object_class_conf = ob.object_class_conf
                
                handlebox = BoundingBox()
                # detected point will be top right of the box(not ideal in most case)
                # handlebox.x = coordinates[0]
                # handlebox.y = coordinates[1]
                # handlebox.width = 2
                # handlebox.height = 10
                
                # detected point will be in the center of the bbox.
                
                handlebox.x = max(coordinates[0] - 1,0)
                handlebox.y = max(coordinates[1] - 1,0)
                handlebox.width = 2
                handlebox.height = 2
                
                handlebox.cols = ob.bbox.cols
                handlebox.rows = ob.bbox.rows
                
                handleOb.bbox = handlebox
                handles.append(handleOb)
        msg.objects.extend(handles)
        self.pub.publish(msg)
              
    
    def HandlePoint(self, o_img, bbox):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(o_img, "passthrough")
            cv_img = cv_img[bbox.y:bbox.y+bbox.height, bbox.x:bbox.x+bbox.width]
            img = cv2.cvtColor(cv_img, cv2.IMREAD_GRAYSCALE)
            # blurred = cv2.GaussianBlur(img, (5, 5), 0)
            # equalized_img = cv2.equalizeHist(img)
            # _, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_OTSU)
            edges = cv2.Canny(img, 100, 245)
            # plt.subplot(131),plt.imshow(edges, cmap = 'gray')
            # plt.show()
            
        except CvBridgeError:
            rospy.loginfo("cannot open image:"+ CvBridgeError)

        #Set up variables for starting point and step size
        start_x = edges.shape[1] // 2

        # some find tune for x(use bag centre instead of bbox centre if uncommented following section)
        # left = None
        # right = None
        # do a horizontal check at 75% y to determine the left and right edge of the bag within the frame
        # for x in range(0,edges.shape[1],1):
        #     if edges[edges.shape[0] // 2,x]:
        #         if left==None:
        #             left = x
        #         right = x

        # start_x = (left + right) //2
        # rospy.loginfo(self.type)
        handle_y = -1
        if self.type == "looped_handle":
            last_edge = -1 
            no_edge_count = 0
            # rospy.loginfo(edges)
            for y in range(0, edges.shape[0], 1):
                if edges[y, start_x] > 0:
                    # cx = start_x
                    # cy = y
                    # radius = 100
                    # img_copy = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
                    # cv2.circle(img_copy, (cx, cy), radius, (0, 0, 255), -1)
                    last_edge = y
                    print("I found edge")
                    no_edge_count = 0
                    # break
                else: 
                    # is not an edge
                    if last_edge != -1: 
                        no_edge_count += 1 
                
                # NOTE: adjust following threshhold to detect if it find a hole which we can grasp on the handle above it.
                if last_edge != -1 and no_edge_count == 30: 
                    # rospy.loginfo("Handle be found")
                    handle_y = last_edge
                    break
        elif self.type == "not_looped_handle":
            # rospy.loginfo("in_not_looped mode")
            
            for y in range(5, edges.shape[0], 1):
                if edges[y, start_x] > 0:
                    handle_y = y
                    # rospy.loginfo(y)
                    # rospy.loginfo("handle be found from not_looped handle mode")
                    break
        if handle_y == -1:
            # rospy.loginfo(edges.shape)
            rospy.loginfo("Handle cannot be found")
        else:
            rospy.loginfo("handle found" + str(bbox.x + start_x) + " "+ str(bbox.y+handle_y))
        return (bbox.x + start_x, bbox.y+handle_y)

def main(args):
    rospy.init_node('BagHandleDetect', anonymous=True)
    obc = BagHandleDetect()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)