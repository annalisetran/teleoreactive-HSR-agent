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

import rospy
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import TransformStamped, Point32

import cv2
import message_filters
from cv_bridge import CvBridge
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from ultralytics import YOLO

from vision_msgs.msg import BoundingBox3D, BoundingBox2D
from unsw_vision_msgs.msg import DetectionList, PersonDetection, ObjectDetection, BoundingBox
from std_msgs.msg import UInt32
class TrackerNode():

    def __init__(self) -> None:
        # parameters
        yolo_model = rospy.get_param("~yolo_model", "yolov8m.pt")
        hpe_model = rospy.get_param("~hpe_model", "yolov8m-pose.pt")
        detection_topic = rospy.get_param("~detection_topic", "detection_result")
        image_topic = rospy.get_param("~image_topic", "image_raw")
        depth_topic = rospy.get_param("~depth_topic", "depth_image")
        depth_info_topic = rospy.get_param("~depth_info_topic", "depth_camera_info")
        bp_weights = rospy.get_param("~bp_model")

        self.conf_thres = rospy.get_param("~conf_thres", 0.65)
        self.conf_thres_bp = rospy.get_param("~conf_thres_bp", 0.7)
        self.iou_thres = rospy.get_param("~iou_thres", 0.45)
        self.max_det = rospy.get_param("~max_det", 300)
        self.classes = rospy.get_param("~classes", None)
        self.tracker = rospy.get_param("~tracker", "botsort.yaml")
        self.debug = rospy.get_param("~debug", False)
        self.debug_conf = rospy.get_param("~debug_conf", True)
        self.debug_line_width = rospy.get_param("~debug_line_width", None)
        self.debug_font_size = rospy.get_param("~debug_font_size", None)
        self.debug_font = rospy.get_param("~debug_font", "Arial.ttf")
        self.debug_labels = rospy.get_param("~debug_labels", True)
        self.debug_boxes = rospy.get_param("~debug_boxes", True)
        
        self.model = YOLO(yolo_model)
        self.hpe_model = YOLO(hpe_model)

        self.use_bp = False
        if bp_weights and bp_weights != "":
            rospy.loginfo(f"initilise body weights model using {bp_weights}")
            self.bp_model = YOLO(bp_weights)
            self.use_bp = True

        self.target_frame = rospy.get_param("~target_frame", "base_link")

        # TODO: check these params
        self.maximum_detection_threshold = rospy.get_param("maximum_detection_threshold", 0.3)
        self.depth_image_units_divisor = rospy.get_param("depth_image_units_divisor", 1000)

        # aux
        self.cv_bridge = CvBridge()

        # pubs
        self.image_pub = rospy.Publisher("debug_image", Image, queue_size=1)
        self.bp_image_pub = rospy.Publisher("debug_image_bp", Image, queue_size=1)

        # subs

        self.img_sub = rospy.Subscriber(image_topic, Image, self.on_detections, queue_size=5)

        self.detection_pub = rospy.Publisher(
            detection_topic, DetectionList, queue_size=1
        )

        rospy.loginfo("Tracker node ready")

    # depth_msg: Image,
    # depth_info_msg: CameraInfo,
    
    def on_detections(
        self,
        image_msg: Image,
    ) -> None:
        header = image_msg.header
        numpy_image = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        
        results = self.model.track(
            source=numpy_image,
            conf=self.conf_thres,
            iou=self.iou_thres,
            max_det=self.max_det,
            classes=self.classes,
            tracker=self.tracker,
            verbose=False,
            persist=True
        )
        if results is None:
            return

        results = results[0].cpu()
        detections_msg = DetectionList()
        detections_msg.header = header
        header.frame_id = self.target_frame
        bounding_boxes = results.boxes.xywh
        classes = results.boxes.cls
        tracking_ids = results.boxes.id
        confidence_score = results.boxes.conf
        detections_msg.image = image_msg

        if tracking_ids == None: return
        person_count = 0
        for bbox, cls, conf, tracking_id in zip(bounding_boxes, classes, confidence_score, tracking_ids):
            bbox2d = BoundingBox2D()
            bbox2d.center.x = float(bbox[0])
            bbox2d.center.y = float(bbox[1])
            bbox2d.size_x = float(bbox[2])
            bbox2d.size_y = float(bbox[3])

            #getting x1,x2,y1,y2, and clip to the image size(corresponding to top left and bottom right corner of the bounding box)
            x1 = self.clipBboxBoundry(int(bbox2d.center.x - bbox2d.size_x // 2), 0, image_msg.width-1)
            x2 = self.clipBboxBoundry(int(bbox2d.center.x + bbox2d.size_x // 2), 0, image_msg.width-1)
            y1 = self.clipBboxBoundry(int(bbox2d.center.y - bbox2d.size_y // 2), 0, image_msg.height-1)
            y2 = self.clipBboxBoundry(int(bbox2d.center.y + bbox2d.size_y // 2), 0, image_msg.height-1)

            class_name = self.model.names[int(cls)]
            if class_name == "person":
                detection = PersonDetection()
                detection.tracking_id = int(tracking_id)
                detection.bbox_person = self.convert_bbox2d_to_msg_bbox(bbox2d, image_msg.width, image_msg.height)

                hpe_img = numpy_image[y1:y2, x1:x2]         # crop the person from the image
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

                        print(self.bp_model.names)
                        print(bp_cls)
                        # TO BE UPDATED if adding more body parts. NOW only check for head
                        if self.bp_model.names.get(int(bp_cls)) == "head":
                            rospy.loginfo("I saw the head")
                            # create a bounding box and attach to the person
                            bp_boundingbox = self.convert_bbox2d_to_msg_bbox(bp_bbox2d, image_msg.width, image_msg.height)
                            detection.bbox_head = bp_boundingbox

                        else:
                            rospy.logwarn(f"unsupported bp parts-- {bp_cls} -- {self.bp_model.names.get(int(bp_cls))}")
                # Predict hpe
                hpe_results = self.hpe_model.predict(
                    source=hpe_img,
                    conf=self.conf_thres_bp,
                    max_det=1,
                    verbose=False,
                )
                
                if hpe_results is None:
                    continue
                hpe_results = hpe_results[0].cpu()

                if hpe_results.keypoints is None:
                    continue
                if hpe_results.keypoints[0].conf is None:
                    continue
                
                keypoints_list = []
                rospy.logwarn("ha")
                print(len(hpe_results.keypoints.xy[0]))
                for i in range(len(hpe_results.keypoints.xy[0])):
                    msg = Point32()
                    if hpe_results.keypoints.conf[0][i] >= 0.30:
                        msg.x = float(hpe_results.keypoints.xy[0][i][0])
                        msg.y = float(hpe_results.keypoints.xy[0][i][1])
                    hpe_results.keypoints.xy[0][i][0] += x1
                    hpe_results.keypoints.xy[0][i][1] += y1
                    # NOTE: DO NOT PUT NEXT LINE INTO THE IF STATEMENT. If below confidence score it should still be put into msgs to make sure index correctly for joints.
                    keypoints_list.append(msg)


                results.keypoints = hpe_results.keypoints

                detection.skeleton2d = keypoints_list
                detections_msg.people.append(detection)
            else:
                detection = ObjectDetection()
                detection.tracking_id = int(tracking_id)
                detection.object_class = class_name
                detection.object_class_conf = float(conf.cpu().numpy())

                detection.bbox = self.convert_bbox2d_to_msg_bbox(bbox2d, image_msg.width, image_msg.height)

                detections_msg.objects.append(detection)

        
        self.publish_debug_image(results, self.image_pub)
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
            if not np.isnan(p).any():
                msg = Point32()
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
                rospy.Duration(0.03) # wait 50ms for the transform to catch up
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
        
    # convert the boundingbox2d to the message boundingbox.
    # in boundingbox2d, it refers to the center of the bounding box and the size of the bounding box.
    # in boundingbox, it refers to the top left corner of the bounding box and the size of the bounding box, also including the size of original image.
    def convert_bbox2d_to_msg_bbox(self, bbox2d: BoundingBox2D, image_width, image_height) -> BoundingBox:
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


if __name__ == "__main__":
    rospy.init_node("tracker_node")
    node = TrackerNode()
    rospy.spin()