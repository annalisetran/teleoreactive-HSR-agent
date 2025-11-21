#!/usr/bin/env python3
# General Model Object Dection Node
#
# Created on 11/06/2025
#
# Author's Beth & Oscar with reference to https://robolab.cse.unsw.edu.au/gitlab/unsw-hsr/vision/yolov8/-/blob/main/script/tracker_node.py?ref_type=heads

import numpy as np
from typing import List, Tuple

import rospy
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import TransformStamped, PointStamped, Point32

import cv2
import message_filters
from cv_bridge import CvBridge
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
from ultralytics import YOLO

from vision_msgs.msg import BoundingBox2D
from unsw_vision_msgs.msg import DetectionList, ObjectDetection, BoundingBox
from std_msgs.msg import UInt32
#from plotter import Plotter

import os
import sys
module_path = os.environ.get("UNSW_WS")
sys.path.append(module_path + "/VISION/utils/src")

class GeneralVisionModelNode():

    def __init__(self) -> None:
        # parameters
        yolo_model = rospy.get_param("~yolo_model", "yolo11n-seg.pt")        
        output_detection_topic_gm = rospy.get_param("~output_detection_topic_gm", "detection_result_gm")
        input_image_topic = rospy.get_param("~input_image_topic", "image_raw")        
        input_depth_topic = rospy.get_param("~input_depth_topic", "depth_image")
        depth_info_topic = rospy.get_param("~depth_info_topic", "depth_camera_info")        

        self.conf_thres = rospy.get_param("~conf_thres", 0.65)        
        self.max_det = rospy.get_param("~max_det", 300)
        
        self.tracker = rospy.get_param("~tracker", "botsort.yaml")
        #self.debug = rospy.get_param("~debug", False)
        self.debug = True
        #self.debug_cv_window = rospy.get_param("~debug_cv_window", False)
        self.debug_cv_window = True
        self.debug_conf = rospy.get_param("~debug_conf", True)
        self.debug_line_width = rospy.get_param("~debug_line_width", None)
        self.debug_font_size = rospy.get_param("~debug_font_size", None)
        self.debug_font = rospy.get_param("~debug_font", "Arial.ttf")
        self.debug_labels = rospy.get_param("~debug_labels", True)
        self.debug_boxes = rospy.get_param("~debug_boxes", True)
        

        # print(f"Yolo Model: {yolo_model}")        
        # print(f"Output Detection Topic: {output_detection_topic}")
        # print(f"Input Image Topic: {input_image_topic}")
        # print(f"Depth Info Topic: {depth_info_topic}")
        # print(f"Input Depth Topic: {input_depth_topic}")        

        self.model = YOLO(yolo_model)        
        #self.plotter = Plotter()
        #self.tracking_id_handler = TrackingIdHandler()
        self.target_frame = rospy.get_param("~target_frame", "base_link")

        self.maximum_detection_threshold = rospy.get_param("maximum_detection_threshold", 0.3)
        self.depth_image_units_divisor = rospy.get_param("depth_image_units_divisor", 1000)

        # aux
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        self.cv_bridge = CvBridge()

        rospy.sleep(1) # wait for the tf listener to be ready before starting subscribers that depend on it

        # publisher
        self.image_pub = rospy.Publisher("debug_image", Image, queue_size=1)
        self.temp_pub = rospy.Publisher("debug_temp_image", Image, queue_size=1)
        self.detection_pub = rospy.Publisher(output_detection_topic_gm, DetectionList, queue_size=1)

        # Subscriber       
        self.image_sub = rospy.Subscriber(input_image_topic, Image, self.on_detections)
        #self.hand_image_sub = rospy.Subscriber('/hsrb/hand_camera/image_raw', Image, self.on_detections)
        #self.image_sub = rospy.Subscriber('camera/dummy_image', Image, self.on_detections)

        rospy.loginfo("General Vision Model - Node ready")

    def on_detections(
        self,
        image_msg: Image,
        depth_msg: Image = None,
        depth_info_msg: CameraInfo = None,
    ) -> None:

        #  .header = timestamp makes the id unique
        header = image_msg.header
        numpy_image = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")  
        
        cv2.imshow('img', numpy_image)     
        
        # If bounding boxes are overlapping gives shadows or something, posibly implement
        #occlusion_map = np.zeros((image_msg.height, image_msg.width), dtype=np.uint8)
        
        # depth image init   
        if depth_msg is not None:         
            depth_image = self.cv_bridge.imgmsg_to_cv2(depth_msg)

        results = self.model.track(
            source=numpy_image,
            conf=self.conf_thres,
            max_det=self.max_det,          
            tracker=self.tracker,
            verbose=False,
            persist=True
        )
        obj_seg_results = results[0].cpu()
        
        '''
        Obj Segmentation Start 
        '''
        if self.debug and self.debug_cv_window:
            all_obj_contours = []  # List to store all individual contours
            if obj_seg_results is not None and obj_seg_results.masks is not None:
                #rospy.logwarn("Inside if")
                for mask in obj_seg_results.masks:
                    all_obj_contours.append(np.array(mask.xy, dtype=np.int32))
            else:
                rospy.logwarn("Object segmentation model failed")

            mask_obj = np.zeros((numpy_image.shape[0], numpy_image.shape[1]), dtype=np.uint8)
            cv2.fillPoly(mask_obj, all_obj_contours, color=255)
            # apply the mask to numpy image
            obj_seg_img = cv2.bitwise_and(numpy_image, numpy_image, mask=mask_obj)
            cv2.imshow("seg-mask-obj", obj_seg_img)
            cv2.waitKey(1)
        '''
        Obj segmentation End
        '''

        # init msg
        detections_msg = DetectionList()
        detections_msg.header = header
        header.frame_id = self.target_frame

        bounding_boxes = obj_seg_results.boxes.xywh
        classes = obj_seg_results.boxes.cls
        tracking_ids = obj_seg_results.boxes.id
        confidence_score = obj_seg_results.boxes.conf
        
        detections_msg.image = image_msg

        if tracking_ids == None: return

        # For detecting objects
        for bbox, cls, conf, tracking_id, masks in zip(bounding_boxes, classes, confidence_score, tracking_ids, obj_seg_results.masks):
            bbox2d = BoundingBox2D()
            bbox2d.center.x = float(bbox[0])
            bbox2d.center.y = float(bbox[1])
            bbox2d.size_x = float(bbox[2])
            bbox2d.size_y = float(bbox[3])            
            distance = 0                    

            
            class_name = self.model.names[int(cls)]
            
            # TODO: ADD CONTOUR TO ObjectDetection.msg SOONER
            contour = np.array(masks.xy, dtype=np.int32)
            
            detection = ObjectDetection()
            detection.tracking_id = int(tracking_id)
            detection.object_class = class_name
            detection.object_class_conf = float(conf.cpu().numpy())            
            detection.distance = distance
            #detection.position = bbox3d.center.position
            detection.bbox = self.convert_bbox2d_to_msg_bbox(bbox2d, image_msg.width, image_msg.height)
           #self.plotter.add_object(detection)
            detections_msg.objects.append(detection)

            #print(f"Frame Id: {header.seq}")
            self.detection_pub.publish(detections_msg)

        
        self.publish_debug_image(obj_seg_results, self.image_pub)

        
    def publish_debug_image(self, results, publisher):
        if self.debug and results is not None:
            plotted_image = results.plot(
                conf=self.debug_conf,
                line_width=self.debug_line_width,
                font_size=self.debug_font_size,
                font=self.debug_font,
                labels=self.debug_labels,
                boxes=self.debug_boxes,
            )
            debug_image_msg = self.cv_bridge.cv2_to_imgmsg(plotted_image, encoding="bgr8")
            publisher.publish(debug_image_msg)

    def convert_bbox2d_to_msg_bbox(self, bbox2d: BoundingBox2D, image_width, image_height) -> BoundingBox:
        # convert the boundingbox2d to the message boundingbox.
        # in boundingbox2d, it refers to the center of the bounding box and the size of the bounding box.
        # in boundingbox, it refers to the top left corner of the bounding box and the size of the bounding box, also including the size of original image.
        bbox = BoundingBox()
        bbox.x = self.clipBboxBoundry(int(bbox2d.center.x - bbox2d.size_x // 2), 0, image_width-1)
        bbox.y = self.clipBboxBoundry(int(bbox2d.center.y - bbox2d.size_y // 2), 0, image_height-1)
        bbox.width = int(bbox2d.size_x)
        bbox.height = int(bbox2d.size_y)
        bbox.cols = int(image_width)
        bbox.rows = int(image_height)

        return bbox
        
    def clipBboxBoundry(self, value:int,lower:int,upper:int):
            # Make the value for bounding box inside the image size. In the perspective of raw image size.
            if value < lower:
                return lower
            elif value >= upper:
                return upper
            return value


if __name__ == "__main__":
    rospy.init_node("gm_node")
    node = GeneralVisionModelNode()
    rospy.spin()