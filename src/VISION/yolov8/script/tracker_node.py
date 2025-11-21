#!/usr/bin/env python3
# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import numpy as np
from typing import List, Tuple
import torch
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

from vision_msgs.msg import BoundingBox3D, BoundingBox2D
from unsw_vision_msgs.msg import DetectionList, PersonDetection, ObjectDetection, BoundingBox, VisualVector
from std_msgs.msg import UInt32
from tracking_id_handler import TrackingIdHandler
from plotter import Plotter

from vector_generation import gaze_vector_generation, pointing_vector_generation, gaze_vector_generation_2

import os
import sys
module_path = os.environ.get("UNSW_WS")
sys.path.append(module_path + "/VISION/utils/src")


class TrackerNode():
    # Constants
    # Horizontal threshhold for occluded person. If the bbox edge to the image edge is less than this value(pixel), it is considered as occluded.
    OCCLUDED_THRESHOLD_HORIZONTAL = 15
    OCCLUDED_THRESHOLD_VERTICAL = 5

    def __init__(self) -> None:
        # parameters
        rospy.loginfo("Tracker node initialising...")
        yolo_model = rospy.get_param("~yolo_model", "yolo11n-seg.pt")
        hpe_model = rospy.get_param("~hpe_model", "yolov8m-pose.pt")
        output_detection_topic = rospy.get_param("~output_detection_topic", "detection_result")
        input_image_topic = rospy.get_param("~input_image_topic", "image_raw")
        input_depth_topic = rospy.get_param("~input_depth_topic", "depth_image")
        depth_info_topic = rospy.get_param("~depth_info_topic", "depth_camera_info")
        bp_weights = rospy.get_param("~bp_model","")

        self.conf_thres = rospy.get_param("~conf_thres", 0.65)
        self.conf_thres_bp = rospy.get_param("~conf_thres_bp", 0.7)
        self.iou_thres = rospy.get_param("~iou_thres", 0.45)
        self.max_det = rospy.get_param("~max_det", 300)
        self.classes = rospy.get_param("~inference_object_classes", None)
        # read from string into list
        if self.classes:
            self.classes = self.classes.split(",")
            # convert string to int
            self.classes = [int(i) for i in self.classes]
            if len(self.classes) == 0:
                self.classes = None
        print(f"Classes: {self.classes}")
        self.tracker = rospy.get_param("~tracker", "botsort.yaml")
        self.debug = rospy.get_param("~debug", False)
        self.debug_cv_window = rospy.get_param("~debug_cv_window", False)
        self.debug_conf = rospy.get_param("~debug_conf", True)
        self.debug_line_width = rospy.get_param("~debug_line_width", None)
        self.debug_font_size = rospy.get_param("~debug_font_size", None)
        self.debug_font = rospy.get_param("~debug_font", "Arial.ttf")
        self.debug_labels = rospy.get_param("~debug_labels", True)
        self.debug_boxes = rospy.get_param("~debug_boxes", True)
        self.generate_3d = rospy.get_param("~generate_3d", True)
        self.mask_expansion_pct = rospy.get_param("~mask_expansion_pct", 0.0)

        # print(f"Yolo Model: {yolo_model}")
        # print(f"Classes: {self.classes}")
        # print(f"HPE Model: {hpe_model}")
        # print(f"Output Detection Topic: {output_detection_topic}")
        # print(f"Input Image Topic: {input_image_topic}")
        # print(f"Depth Info Topic: {depth_info_topic}")
        # print(f"Input Depth Topic: {input_depth_topic}")
        # print(f"Body Parts Weights {bp_weights}")

        self.model = YOLO(yolo_model)
        self.hpe_model = YOLO(hpe_model)
        self.person_segmentation_model = YOLO("yolo11m-seg.pt")
        self.plotter = Plotter()
        self.tracking_id_handler = TrackingIdHandler()

        self.use_bp = False
        if bp_weights and bp_weights != "":
            rospy.loginfo(f"initilise body weights model using {bp_weights}")
            self.bp_model = YOLO(bp_weights)
            self.use_bp = True

        self.target_frame = rospy.get_param("~target_frame", "base_link")

        self.maximum_detection_threshold = rospy.get_param("maximum_detection_threshold", 0.3)
        self.depth_image_units_divisor = rospy.get_param("depth_image_units_divisor", 1000)

        # TODO: Consider put tf into if self.generate_3d
        # aux
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        self.cv_bridge = CvBridge()

        rospy.sleep(1) # wait for the tf listener to be ready before starting subscribers that depend on it

        # publisher
        self.image_pub = rospy.Publisher("debug_image", Image, queue_size=1)
        self.bp_image_pub = rospy.Publisher("debug_image_bp", Image, queue_size=1)
        self.temp_pub = rospy.Publisher("debug_temp_image", Image, queue_size=1)
        self.detection_pub = rospy.Publisher(output_detection_topic, DetectionList, queue_size=1)
        # subsscriber
        if self.generate_3d:
            self.depth_sub = message_filters.Subscriber(input_depth_topic, Image)
            self.depth_info_sub = message_filters.Subscriber(depth_info_topic, CameraInfo)
            self.image_sub = message_filters.Subscriber(input_image_topic, Image)
            self._synchronizer = message_filters.ApproximateTimeSynchronizer(
                (self.image_sub, self.depth_sub, self.depth_info_sub),
                5, 0.5, allow_headerless=False)
        
            self._synchronizer.registerCallback(self.on_detections)
        
        else:
            self.image_sub = rospy.Subscriber(input_image_topic, Image, self.on_detections)

        rospy.loginfo("Tracker node ready")

    # depth_msg: Image,
    # depth_info_msg: CameraInfo,
    
    def on_detections(
        self,
        image_msg: Image,
        depth_msg: Image = None,
        depth_info_msg: CameraInfo = None,
    ) -> None:

        header = image_msg.header
        numpy_image = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8").copy()
        self.plotter.reset()
        # create an empty 2d array to map to the cells of the image
        # this array will be used to store the mask of each person
        # the mask will be used to determin if head is in other people's bounding box
        # if the head is in other people's bounding box, it is occuluded
        # each person will add one to the pixel of the mask
        occlusion_map = np.zeros((image_msg.height, image_msg.width), dtype=np.uint8)
        
        # depth image init
        if (self.generate_3d):
            transform = self.get_transform(depth_info_msg.header)
            if transform is None:
                rospy.logwarn("transform not found")
                return
            # transform to baselink
            try:
                base_link_transform = self.tf_buffer.lookup_transform(
                    "base_link",
                    header.frame_id,
                    header.stamp,
                    rospy.Duration(0.15) # wait 50ms for the transform to catch up
                )
            except Exception as ex:
                rospy.logwarn(f"Could not transform: {ex}")
                return

            
            depth_image = self.cv_bridge.imgmsg_to_cv2(depth_msg).copy()


        '''
        Person segmentation Start 
        '''
        person_segmentation_results = self.person_segmentation_model.predict(
            source=numpy_image,
            conf=self.conf_thres,
            iou=self.iou_thres,
            max_det=self.max_det,
            classes=[0],
            verbose=False,
        )
        p_seg_results = person_segmentation_results[0].cpu()
        del person_segmentation_results
        all_person_contours = []  # List to store all individual contours
        if p_seg_results is not None and p_seg_results.masks is not None:
            for mask in p_seg_results.masks:
                all_person_contours.append(np.array(mask.xy, dtype=np.int32))
        else:
            rospy.logwarn("Person segmentation model failed and returned None, either no person in the frame or model failed")

        mask_persons = np.zeros((numpy_image.shape[0], numpy_image.shape[1]), dtype=np.uint8)
        cv2.fillPoly(mask_persons, all_person_contours, color=255)
        # apply the mask to numpy image
        person_seg_img = cv2.bitwise_and(numpy_image, numpy_image, mask=mask_persons)

        if self.debug and self.debug_cv_window:
            cv2.imshow("seg-mask-person", person_seg_img)
            cv2.waitKey(1)
            
        '''
        Person segmentation End
        '''

        results = self.model.track(
            source=numpy_image,
            conf=self.conf_thres,
            iou=self.iou_thres,
            max_det=self.max_det,
            # classes=self.classes,          
            tracker=self.tracker,
            verbose=False,
            persist=True
        )
        obj_seg_results = results[0].cpu()

        all_object_contours = []  # List to store all individual contours
        if obj_seg_results is not None and obj_seg_results.masks is not None:
            for mask in obj_seg_results.masks:
                all_object_contours.append(np.array(mask.xy, dtype=np.int32))
        else:
            rospy.logwarn("Object segmentation model failed and returned None, either no objects in the frame or model failed")

        mask_objects = np.zeros((numpy_image.shape[0], numpy_image.shape[1]), dtype=np.uint8)
        cv2.fillPoly(mask_objects, all_object_contours, color=255)
        # apply the mask to numpy image
        object_seg_img = cv2.bitwise_and(numpy_image, numpy_image, mask=mask_persons)

        if self.debug and self.debug_cv_window:
            cv2.imshow("seg-mask-object", object_seg_img)
            cv2.waitKey(1)

        '''
        Obj segmentation Start 
        '''
        if self.debug and self.debug_cv_window:
            all_obj_contours = []  # List to store all individual contours
            if obj_seg_results is not None and obj_seg_results.masks is not None:
                for mask in obj_seg_results.masks:
                    all_obj_contours.append(np.array(mask.xy, dtype=np.int32))
            else:
                rospy.logwarn("Obj segmentation model failed")

            mask_obj = np.zeros((numpy_image.shape[0], numpy_image.shape[1]), dtype=np.uint8)
            cv2.fillPoly(mask_obj, all_obj_contours, color=255)
            # apply the mask to numpy image
            obj_seg_img = cv2.bitwise_and(numpy_image, numpy_image, mask=mask_obj)
            cv2.imshow("seg-mask-obj", obj_seg_img)
            cv2.waitKey(1)
        '''
        Obj segmentation End
        '''

        hpe_model_results = self.hpe_model.track(
                    source=person_seg_img,
                    conf=self.conf_thres,   # using object threshold for hpe for testing purpose
                    max_det=5, #TODO: CHANGE MAX NUMBER LATER ON, But for now, 5 ppl should be enough
                    tracker = self.tracker,
                    verbose=False,
                    persist = True
        )

        if results is None and hpe_model_results is None:
            # publish empty ms
            msg = DetectionList()
            msg.header = header
            self.detection_pub.publish(msg)
            return

        del results
        
        hpe_results = hpe_model_results[0].cpu()
        torch.cuda.empty_cache()
        del hpe_model_results
        hpe_bounding_bboxes = hpe_results.boxes.xywh
        hpe_tracking_ids = hpe_results.boxes.id
        hpe_keypoints = hpe_results.keypoints

        # init msg
        detections_msg = DetectionList()
        detections_msg.header = header
        header.frame_id = self.target_frame


        if hpe_tracking_ids != None:
            for bbox, kps, tracking_id in zip(hpe_bounding_bboxes, hpe_keypoints, hpe_tracking_ids):
                bbox2d = BoundingBox2D()
                bbox2d.center.x = float(bbox[0])
                bbox2d.center.y = float(bbox[1])
                bbox2d.size_x = float(bbox[2])
                bbox2d.size_y = float(bbox[3])

                #getting x1,x2,y1,y2, and clip to the image size(corresponding to top left and bottom right corner of the bounding box)
                # also expand the mask by a certain percentage
                # og refers to original, without expansion
                offset_horizontal = self.mask_expansion_pct * image_msg.width
                offset_vertical = self.mask_expansion_pct * image_msg.height

                                            
                x1 = self.clipBboxBoundry(int(bbox2d.center.x - bbox2d.size_x // 2 -offset_horizontal), 0, image_msg.width-1)
                x2 = self.clipBboxBoundry(int(bbox2d.center.x + bbox2d.size_x // 2 +offset_horizontal), 0, image_msg.width-1)
                y1 = self.clipBboxBoundry(int(bbox2d.center.y - bbox2d.size_y // 2 -offset_vertical), 0, image_msg.height-1)
                y2 = self.clipBboxBoundry(int(bbox2d.center.y + bbox2d.size_y // 2 +offset_vertical), 0, image_msg.height-1)

                bbox3d = None
                distance = 0
                # getting the 3d bounding box
                if self.generate_3d:
                    bbox3d = self.convert_bb_to_3d(depth_image, depth_info_msg, bbox2d)
                    if bbox3d is not None:
                        bbox3d = TrackerNode.transform_3d_box(
                            bbox3d, transform[0], transform[1])
                        # translate the bounding box center point to the base_link.
                        detection_x_baselink = bbox3d.center.position.x + base_link_transform.transform.translation.x
                        detection_y_baselink = bbox3d.center.position.y + base_link_transform.transform.translation.y
                        # distance in 2 dimention, z is ignored
                        distance = np.sqrt(detection_x_baselink**2 + detection_y_baselink**2)
                # init bbox3d if not in 3d. Default to all 0s
                if bbox3d is None:
                    bbox3d = BoundingBox3D()

                detection = PersonDetection()
                detection.tracking_id = int(tracking_id)
                detection.bbox_person = self.convert_bbox2d_to_msg_bbox(bbox2d, image_msg.width, image_msg.height)
                # always assign bbox3d. Even 3d is false, it will be init value with all 0s
                detection.bbox3d = bbox3d
                detection.distance = distance
                detection.position = bbox3d.center.position

                # add the mask to the occlusion map
                occlusion_map[detection.bbox_person.y:detection.bbox_person.y+detection.bbox_person.height, detection.bbox_person.x:detection.bbox_person.x+detection.bbox_person.width] += 1

                
                keypoints_list = []
                for i in range(len(kps.xy[0])):
                    msg = Point32()
                    msg.x = float(kps.xy[0][i][0])
                    msg.y = float(kps.xy[0][i][1])
                    keypoints_list.append(msg)


                if self.generate_3d:
                    keypoints3d = self.convert_keypoints_to_3d(
                        depth_image, depth_info_msg, keypoints_list)
                    keypoints3d = TrackerNode.transform_3d_keypoints(
                        keypoints3d, transform[0], transform[1])
                    detection.skeleton = keypoints3d
                    # add pointing for the person
                    # TODO: This may cause the node the crash
                    # detection = pointing_vector_generation(detection)
                    # detection = gaze_vector_generation(detection)

                detection.skeleton2d = keypoints_list


                ######### additional inferencing for body parts ###########
                if self.use_bp:
                    # mask the original image around the bounding box of person
                    # make a copy of the original image
                    masked_img = np.copy(numpy_image)
                    mask = np.zeros(masked_img.shape[:2], dtype=np.uint8)
                    # Create a binary mask using the bounding box coordinates
                    mask[y1:y2, x1:x2] = 255  # Assuming white color (255) for the masked region
                    
                    # Apply the mask to image(its copy), store the result in masked_img
                    masked_img = cv2.bitwise_and(masked_img, masked_img, mask=mask)                    

                    # inferencing
                    bp_model_results= self.bp_model.predict(
                        source=masked_img,
                        conf=self.conf_thres_bp,
                        iou=self.iou_thres,
                        max_det=1,
                        # allow only inferencing on 0th class, which is head, 1 is person, not nessary to infer on person
                        classes=[0],
                        verbose=False,
                    )
                    
                    # apply the result to message
                    bp_results = bp_model_results[0].cpu()
                    del bp_model_results

                    self.publish_debug_image(bp_results,self.bp_image_pub)
                    bp_bounding_boxes = bp_results.boxes.xywh
                    bp_classes = bp_results.boxes.cls
                    bp_confidence_score = bp_results.boxes.conf

                    for bp_bbox, bp_cls, bp_conf in zip(bp_bounding_boxes, bp_classes, bp_confidence_score):
                        bp_bbox2d = BoundingBox2D()
                        bp_bbox2d.center.x = float(bp_bbox[0])
                        bp_bbox2d.center.y = float(bp_bbox[1])
                        bp_bbox2d.size_x = float(bp_bbox[2])
                        bp_bbox2d.size_y = float(bp_bbox[3])

                        # TO BE UPDATED if adding more body parts. NOW only check for head
                        if self.bp_model.names.get(int(bp_cls)) == "head":
                            # create a bounding box and attach to the person
                            bp_boundingbox = self.convert_bbox2d_to_msg_bbox(bp_bbox2d, image_msg.width, image_msg.height)
                            # check if the head is alligned with the person skeleton
                            # if the head is not alligned with the skeleton, it is not a head
                            # checcking if the eyes or ears or nose is inside the bounding box of head

                            i=0
                            inside = False
                            while i < 5 and inside == False:
                                if detection.skeleton2d[i].x != 0.0 and detection.skeleton2d[i].y != 0.0:                    
                                    if int(detection.skeleton2d[i].x) > int(bp_boundingbox.x) \
                                        and int(detection.skeleton2d[i].x) < int(bp_boundingbox.x + bp_boundingbox.width) \
                                        and int(detection.skeleton2d[i].y) > int(bp_boundingbox.y) \
                                        and int(detection.skeleton2d[i].y) < int(bp_boundingbox.y + bp_boundingbox.height): 

                                        inside = True
                                i+=1
                            if inside:
                                detection.bbox_head = bp_boundingbox

                        else:
                            rospy.logwarn(f"unsupported bp parts-- {bp_cls} -- {self.bp_model.names.get(int(bp_cls))}")



                # wrap the id

                # occlusion check
                detection.bbox_person.occluded = detection.bbox_person.x < TrackerNode.OCCLUDED_THRESHOLD_HORIZONTAL or detection.bbox_person.x + detection.bbox_person.width > image_msg.width - TrackerNode.OCCLUDED_THRESHOLD_HORIZONTAL
                detection.bbox_head.occluded = detection.bbox_head.x < TrackerNode.OCCLUDED_THRESHOLD_HORIZONTAL \
                    or detection.bbox_head.x + detection.bbox_head.width > image_msg.width - TrackerNode.OCCLUDED_THRESHOLD_HORIZONTAL \
                    or detection.bbox_head.y < TrackerNode.OCCLUDED_THRESHOLD_VERTICAL \
                    or detection.bbox_head.y + detection.bbox_head.height > image_msg.height - TrackerNode.OCCLUDED_THRESHOLD_VERTICAL
                self.plotter.add_person(detection)
                detections_msg.people.append(detection)                

        
        # For objects
        bounding_boxes = obj_seg_results.boxes.xywh
        classes = obj_seg_results.boxes.cls
        tracking_ids = obj_seg_results.boxes.id
        confidence_score = obj_seg_results.boxes.conf
        
        detections_msg.image = image_msg
        detections_msg.person_seg_image = self.cv_bridge.cv2_to_imgmsg(person_seg_img, encoding="bgr8")

        if tracking_ids == None: return

        # Only for detecting objects(skip for person detection when using bbox. BBox is gerneated using HPE directly)
        for bbox, cls, conf, tracking_id, masks in zip(bounding_boxes, classes, confidence_score, tracking_ids, obj_seg_results.masks):
            bbox2d = BoundingBox2D()
            bbox2d.center.x = float(bbox[0])
            bbox2d.center.y = float(bbox[1])
            bbox2d.size_x = float(bbox[2])
            bbox2d.size_y = float(bbox[3])
            bbox3d = None
            distance = 0
            # getting the 3d bounding box
            if self.generate_3d:
                bbox3d = self.convert_bb_to_3d(depth_image, depth_info_msg, bbox2d)
                if bbox3d is not None:
                    bbox3d = TrackerNode.transform_3d_box(
                        bbox3d, transform[0], transform[1])
                    # translate the bounding box center point to the base_link.
                    detection_x_baselink = bbox3d.center.position.x + base_link_transform.transform.translation.x
                    detection_y_baselink = bbox3d.center.position.y + base_link_transform.transform.translation.y
                     # distance in 2 dimention, z is ignored
                    distance = np.sqrt(detection_x_baselink**2 + detection_y_baselink**2)

                     

            if bbox3d is None:
                bbox3d = BoundingBox3D()
   
            class_name = self.model.names[int(cls)]
            # Next line can be deleted if we switch to a model does not detect person.
            if class_name == "person":
                continue
            else:
                # TODO: ADD CONTOUR TO MSGS LATER
                #contour = np.array(masks.xy, dtype=np.int32)
                
                detection = ObjectDetection()
                detection.tracking_id = int(tracking_id)
                detection.object_class = class_name
                detection.object_class_conf = float(conf.cpu().numpy())
                detection.bbox3d = bbox3d
                detection.distance = distance
                detection.position = bbox3d.center.position
                detection.bbox = self.convert_bbox2d_to_msg_bbox(bbox2d, image_msg.width, image_msg.height)
                self.plotter.add_object(detection)
                detections_msg.objects.append(detection)
                detection.mask = np.array(masks.xy, dtype=np.int32).flatten().tolist()
        
        # Person HPE:        
        self.tracking_id_handler.end_scene()
        # x = self.plotter.plot(numpy_image) # commented by Adam

        if (self.debug):
            x = self.plotter.plot_object_and_skeleton(numpy_image, True, True) 
            self.plotter.reset()
            self.image_pub.publish(self.cv_bridge.cv2_to_imgmsg(x, encoding="bgr8"))
        
        # go through the person list and check if the head is in other people's bounding box
        # use the occlusion map to determine if the head is in other people's bounding box
        for person in detections_msg.people:
            if person.bbox_head is not None and person.bbox_head.occluded == False:
                # check if the head is in other people's bounding box
                person.bbox_head.occluded = self.check_occlusion(person.bbox_person, person.bbox_head, occlusion_map)
                
                # check if occluded in person bounding box, if it is, tracker id not valid
                person.tracking_id = -1 if np.any(occlusion_map[person.bbox_person.y:person.bbox_person.y+person.bbox_person.height, person.bbox_person.x:person.bbox_person.x+person.bbox_person.width] > 1) else self.tracking_id_handler.get_person_tracking_id(person)     
        print(f"Frame Id: {header.seq}")
        self.detection_pub.publish(detections_msg)

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

    def convert_bb_to_3d(
        self,
        depth_image: np.ndarray,
        depth_info: CameraInfo,
        bbox: BoundingBox2D
    ) -> BoundingBox3D:

        # crop depth image by the 2d BB
        center_x = int(bbox.center.x)
        center_y = int(bbox.center.y)
        size_x = int(bbox.size_x)
        size_y = int(bbox.size_y)

        u_min = max(center_x - size_x // 2, 0)
        u_max = min(center_x + size_x // 2, depth_image.shape[1] - 1)
        v_min = max(center_y - size_y // 2, 0)
        v_max = min(center_y + size_y // 2, depth_image.shape[0] - 1)

        roi = depth_image[v_min:v_max, u_min:u_max] / \
            self.depth_image_units_divisor  # convert to meters
        if not np.any(roi):
            return None

        # find the z coordinate on the 3D BB
        bb_center_z_coord = depth_image[int(center_y)][int(
            center_x)] / self.depth_image_units_divisor
        z_diff = np.abs(roi - bb_center_z_coord)
        mask_z = z_diff <= self.maximum_detection_threshold
        if not np.any(mask_z):
            return None

        roi_threshold = roi[mask_z]
        z_min, z_max = np.min(roi_threshold), np.max(roi_threshold)
        z = (z_max + z_min) / 2
        if z == 0:
            return None

        # project from image to world space
        k = depth_info.K
        px, py, fx, fy = k[2], k[5], k[0], k[4]
        x = z * (center_x - px) / fx
        y = z * (center_y - py) / fy
        w = z * (size_x / fx)
        h = z * (size_y / fy)

        # create 3D BB
        msg = BoundingBox3D()
        msg.center.position.x = x
        msg.center.position.y = y
        msg.center.position.z = z
        msg.size.x = w
        msg.size.y = h
        msg.size.z = (z_max - z_min)

        return msg

    
    def convert_keypoints_to_3d(
        self,
        depth_image: np.ndarray,
        depth_info: CameraInfo,
        detection: List[Point32]
    ):
        # build an array of 2d keypoints
        keypoints_2d = np.array([[p.x, p.y]
                                for p in detection], dtype=np.int16)
        u = np.array(keypoints_2d[:, 1]).clip(0, depth_info.height - 1)
        v = np.array(keypoints_2d[:, 0]).clip(0, depth_info.width - 1)

        # sample depth image and project to 3D
        z = depth_image[u, v]
        k = depth_info.K
        px, py, fx, fy = k[2], k[5], k[0], k[4]
        x = z * (v - px) / fx
        y = z * (u - py) / fy
        points_3d = np.dstack([x, y, z]).reshape(-1, 3) / \
            self.depth_image_units_divisor  # convert to meters

        # generate message
        msg_array = []
        for p, d in zip(points_3d, detection):
            # if valid point append it, otherwise 000
            
            msg = Point32()
            if not np.isnan(p).any():
                if d.x != 0 and d.y != 0:
                    msg.x = p[0]
                    msg.y = p[1]
                    msg.z = p[2]

            msg_array.append(msg)
        return msg_array

    def get_transform(self, header) -> Tuple[np.ndarray]:
        # transform position from image frame to target_frame
        rotation = None
        traation = None

        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.target_frame,
                header.frame_id,
                header.stamp,
                rospy.Duration(0.15) # wait 50ms for the transform to catch up
            )

            translation = np.array([transform.transform.translation.x,
                                    transform.transform.translation.y,
                                    transform.transform.translation.z])

            rotation = np.array([transform.transform.rotation.w,
                                 transform.transform.rotation.x,
                                 transform.transform.rotation.y,
                                 transform.transform.rotation.z])

            return translation, rotation

        except Exception as ex:
            rospy.logwarn(f"Could not transform: {ex}")
            return None
        
    
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

    @staticmethod
    def transform_3d_box(
        bbox: BoundingBox3D,
        translation: np.ndarray,
        rotation: np.ndarray
    ) -> BoundingBox3D:

        # position
        position = TrackerNode.qv_mult(
            rotation,
            np.array([bbox.center.position.x,
                      bbox.center.position.y,
                      bbox.center.position.z])
        ) + translation

        bbox.center.position.x = position[0]
        bbox.center.position.y = position[1]
        bbox.center.position.z = position[2]

        # size
        size = TrackerNode.qv_mult(
            rotation,
            np.array([bbox.size.x,
                      bbox.size.y,
                      bbox.size.z])
        )

        bbox.size.x = abs(size[0])
        bbox.size.y = abs(size[1])
        bbox.size.z = abs(size[2])

        return bbox

    @staticmethod
    def transform_3d_keypoints(
        keypoints: List[Point32],
        translation: np.ndarray,
        rotation: np.ndarray,
    ):

        for point in keypoints:
            if point == Point32():
                continue
            
            position = TrackerNode.qv_mult(
                rotation,
                np.array([
                    point.x,
                    point.y,
                    point.z
                ])
            ) + translation

            point.x = position[0]
            point.y = position[1]
            point.z = position[2]

        return keypoints

    @staticmethod
    def qv_mult(q: np.ndarray, v: np.ndarray) -> np.ndarray:
        q = np.array(q, dtype=np.float64)
        v = np.array(v, dtype=np.float64)
        qvec = q[1:]
        uv = np.cross(qvec, v)
        uuv = np.cross(qvec, uv)
        return v + 2 * (uv * q[0] + uuv)

    def check_occlusion(self, bbox_person: BoundingBox, bbox_head: BoundingBox, occlusion_map: np.ndarray) -> bool:
        # check if the head is in other people's bounding box
        # if the head is in other people's bounding box, it is occuluded
        # if the head is not in other people's bounding box, it is not occuluded
        # occlusion_map is a 2d array, which is the same size as the image
        # only check the corners of the bounding box to see if it is in the occlusion map

        # take a copy of the map and unmask the person itself

        # This was working on head by checking corners
        # unmask_occlusion_map = occlusion_map.copy()
        # unmask_occlusion_map[bbox_person.y:bbox_person.y+bbox_person.height, bbox_person.x:bbox_person.x+bbox_person.width] -= 1
        # x1 = bbox_head.x
        # x2 = bbox_head.x + bbox_head.width
        # y1 = bbox_head.y
        # y2 = bbox_head.y + bbox_head.height

        # crop out the person bounding box, if any of the value is 2, it is occluded
        return np.any(occlusion_map[bbox_head.y:bbox_head.y+bbox_head.height, bbox_head.x:bbox_head.x+bbox_head.width] > 1)

        # print(f"Head occlusion check: {unmask_occlusion_map[y1][x1] >0 or unmask_occlusion_map[y1][x2] >0 or unmask_occlusion_map[y2][x1] >0 or unmask_occlusion_map[y2][x2] >0}")
        return unmask_occlusion_map[y1][x1] >0 or unmask_occlusion_map[y1][x2] >0 or unmask_occlusion_map[y2][x1] >0 or unmask_occlusion_map[y2][x2] >0


if __name__ == "__main__":
    rospy.init_node("tracker_node")
    node = TrackerNode()
    rospy.spin()