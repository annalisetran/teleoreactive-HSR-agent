#!/usr/bin/python3

import os
import rospy
import torch
import cv2
from cv_bridge import CvBridge
import numpy as np
from typing import Tuple, Union, List

from torchvision.transforms import ToTensor
from models.experimental import attempt_load
from utils.general import non_max_suppression, non_max_suppression_kpt
from utils.datasets import letterbox
from utils.plots import output_to_keypoint, plot_skeleton_kpts
from visualizer import draw_detections, draw_bp_bboxes

from sensor_msgs.msg import Image
from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import Point
from unsw_vision_msgs.msg import DetectionList, BoundingBox, ObjectDetection, PersonDetection

def parse_classes_file(path):
    classes = []
    with open(path, "r") as f:
        for line in f:
            line = line.replace("\n", "")
            classes.append(line)
    return classes


def rescale(ori_shape: Tuple[int, int], boxes: Union[torch.Tensor, np.ndarray],
            target_shape: Tuple[int, int]):
    """Rescale the output to the original image shape
    :param ori_shape: original width and height [width, height].
    :param boxes: original bounding boxes as a torch.Tensor or np.array or shape
        [num_boxes, >=4], where the first 4 entries of each element have to be
        [x1, y1, x2, y2].
    :param target_shape: target width and height [width, height].
    """
    xscale = target_shape[1] / ori_shape[1]
    yscale = target_shape[0] / ori_shape[0]

    boxes[:, [0, 2]] *= xscale
    boxes[:, [1, 3]] *= yscale

    return boxes


class YoloV7:
    def __init__(self, weights, conf_thresh: float = 0.5, iou_thresh: float = 0.45,
                 device: str = "cuda"):
        self.conf_thresh = conf_thresh
        self.iou_thresh = iou_thresh
        self.device = device
        self.model = attempt_load(weights, map_location=device)
        self.model.eval()

    @torch.no_grad()
    def inference(self, img: torch.Tensor):
        """
        :param img: tensor [c, h, w]
        :returns: tensor of shape [num_boxes, 6], where each item is represented as
            [x1, y1, x2, y2, confidence, class_id]
        """
        img = img.unsqueeze(0)
        pred_results = self.model(img)[0]
        detections = non_max_suppression(
            pred_results, conf_thres=self.conf_thresh, iou_thres=self.iou_thresh
        )
        if detections:
            detections = detections[0]
        return detections

class YoloV7_HPE:
    def __init__(self, weights: str, device: str = "cuda"):
        #PARAMS TBD
        self.device = device
        self.weigths = torch.load(weights, map_location=device)
        self.model = self.weigths['model']
        _ = self.model.float().eval()
        if torch.cuda.is_available():
            self.model.half().to(device)            

class Yolov7Publisher:
    def __init__(self, input_topic: str, 
                 output_topic: str,
                 vis_topic: str,
                 weights: str, 
                 hpe_weights : str = None,
                 bp_weights : str = None,
                 device: str = "cuda",
                 visualize: bool = False,
                 conf_thresh: float = 0.5, 
                 iou_thresh: float = 0.45,                  
                 img_size: Union[Tuple[int, int], None] = (640, 640),
                 queue_size: int = 1, 
                 class_labels: Union[List, None] = None,
                 bp_class_labels: Union[List, None] = None,
                 use_human_pose: bool = False):
        """
        :param input_topic: name of the image topic to listen to
        :param output_topic: name of the output topic
        :param vis_topic: name of the visualization topic
        :param weights: path/to/yolo_weights.pt
        :param hpe_weights: path/to/yolov7-w6-pose.pt
        :param bp_weights: path/to/yolov7-body-part.pt
        :param device: device to do inference on (e.g., 'cuda' or 'cpu')
        :param visualize: flag to enable publishing the detections visualized in the image     
        :param conf_thresh: confidence threshold
        :param iou_thresh: intersection over union threshold
        :param img_size: (height, width) to which the img is resized before being
            fed into the yolo network. Final output coordinates will be rescaled to
            the original img size.
        :param queue_size: queue size for publishers
        :param class_labels: List of length num_classes, containing the class
            labels. The i-th element in this list corresponds to the i-th
            class id. Only for viszalization. If it is None, then no class
            labels are visualized.
        :param bp_class_labels: List of length num_classes, containing the body part class
            labels. The i-th element in this list corresponds to the i-th
            class id. Only for viszalization. If it is None, then no class
            labels are visualized. This will be appended to the orginal class labels    
        :param use_human_pose: flag to enable YoloV7 Human Pose Estimation               
        """
        self.img_size = img_size
        self.device = device
        self.class_labels = class_labels
        self.bp_class_labels = bp_class_labels
        self.all_class_labels = class_labels + bp_class_labels

        self.visualization_publisher = rospy.Publisher(
            vis_topic, Image, queue_size=queue_size
        ) if visualize else None

        self.test_publisher = rospy.Publisher(
            "/unsw_vision/detections/objects/test", Image, queue_size=queue_size
        ) if visualize else None

        self.bridge = CvBridge()
        self.tensorize = ToTensor()

        self.model = YoloV7(
            weights=weights, conf_thresh=conf_thresh, iou_thresh=iou_thresh,
            device=device
        )
        if hpe_weights:
            self.hpe_model = YoloV7_HPE(
                weights=hpe_weights, device=device
            )
        if bp_weights:
            self.bp_model = YoloV7(
            weights=bp_weights, conf_thresh=0.5, iou_thresh=iou_thresh,
            device=device
        )

        self.img_subscriber = rospy.Subscriber(
            input_topic, Image, self.process_img_msg, queue_size=5
        )
        self.detection_publisher = rospy.Publisher(
            output_topic, DetectionList, queue_size=queue_size
        )
        rospy.loginfo("YOLOv7 initialization complete. Ready to start inference")
    
    def process_img_msg(self, img_msg: Image):
        """ callback function for publisher """

        t_start = rospy.Time.now()


        np_img_orig = self.bridge.imgmsg_to_cv2(
            img_msg, desired_encoding='bgr8'
        )

        # handle possible different img formats
        if len(np_img_orig.shape) == 2:
            np_img_orig = np.stack([np_img_orig] * 3, axis=2)

        h_orig, w_orig, c = np_img_orig.shape

        # automatically resize the image to the next smaller possible size
        w_scaled, h_scaled = self.img_size
        np_img_resized = cv2.resize(np_img_orig, (w_scaled, h_scaled))

        # conversion to torch tensor (copied from original yolov7 repo)
        img = np_img_resized.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        img = torch.from_numpy(np.ascontiguousarray(img))
        img = img.float()  # uint8 to fp16/32
        img /= 255  # 0 - 255 to 0.0 - 1.
        img = img.to(self.device)

        # inference & rescaling the output to original img size
        detections = self.model.inference(img)
        detections[:, :4] = rescale(
            [h_scaled, w_scaled], detections[:, :4], [h_orig, w_orig])
        detections[:, :4] = detections[:, :4].round()

        # publishing
        detection_msg = DetectionList()
        detection_msg.header = img_msg.header
        detection_msg.image = img_msg

        num_people = 0
        num_objects = 0
        rospy.logwarn(f"T1 taken: {rospy.Time.now().to_sec() - t_start.to_sec()}")
        for detection in detections:
            x1, y1, x2, y2, conf, cls = detection.tolist()
            x1 = int(self.clipBboxBoundry(int(x1),0, w_orig))
            x2 = int(self.clipBboxBoundry(int(x2),0, w_orig))
            y1 = int(self.clipBboxBoundry(int(y1),0, h_orig))
            y2 = int(self.clipBboxBoundry(int(y2),0, h_orig))

            class_name = self.class_labels[int(cls)]
            bp_class_name = None
            if use_human_pose:
                if class_name == 'person':
                    if np.any(np_img_orig):
                        try:
                            hpe_img = letterbox(np_img_orig[y1:y2, x1:x2], self.img_size, stride=64, auto=True)[0]
                        except Exception as e:
                            rospy.loginfo(f"Error Image Size: {e} {self.img_size} \nOriginal Image: {np_img_orig[y1:y2, x1:x2]}, x1: {x1}, x2: {x2}, y1: {y1}, y2: {y2}")

                        w_hpe, h_hpe, _ = hpe_img.shape
                        hpe_img = self.tensorize(hpe_img)        
                        hpe_img = torch.tensor(np.array([hpe_img.numpy()]))
                        scale = abs((h_hpe/(x2-x1) + w_hpe/(y2-y1)) / 2)
                        if torch.cuda.is_available():
                            hpe_img = hpe_img.half().to(device)
                        with torch.no_grad():
                            output, _ = self.hpe_model.model(hpe_img)

                        output = non_max_suppression_kpt(output, 0.25, 0.65, nc=self.hpe_model.model.yaml['nc'], nkpt=self.hpe_model.model.yaml['nkpt'], kpt_label=True)
                        output = output_to_keypoint(output) / scale

                        ######### additional inferencing ###########
                        
                        # mask the original image around the bounding box of person
                        # make a copy of the original image
                        masked_img = np.copy(np_img_orig)
                        # Create an empty mask with the same shape as the image
                        mask = np.zeros(masked_img.shape[:2], dtype=np.uint8)
                        # Create a binary mask using the bounding box coordinates
                        mask[y1:y2, x1:x2] = 255  # Assuming white color (255) for the masked region
                     
                        # Apply the mask to the copy of the original image
                        masked_img = cv2.bitwise_and(masked_img, masked_img, mask=mask)

                        # automatically resize the image to the next smaller possible size
                        w_scaled, h_scaled = self.img_size
                        masked_img_resized = cv2.resize(masked_img, (w_scaled, h_scaled))

                        # conversion to torch tensor (copied from original yolov7 repo)
                        bp_img = masked_img_resized.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
                        bp_img = torch.from_numpy(np.ascontiguousarray(bp_img))
                        bp_img = bp_img.float()  # uint8 to fp16/32
                        bp_img /= 255  # 0 - 255 to 0.0 - 1.
                        bp_img = bp_img.to(self.device)

                        # inference & rescaling the output to original img size
                        bp_detections = self.bp_model.inference(bp_img)
                        bp_detections[:, :4] = rescale(
                            [h_scaled, w_scaled], bp_detections[:, :4], [h_orig, w_orig])
                        bp_detections[:, :4] = bp_detections[:, :4].round()                     

                        for bp_detection in bp_detections:
                            bp_x1, bp_y1, bp_x2, bp_y2, bp_conf, bp_cls = bp_detection.tolist()
                            bp_class_name = self.bp_class_labels[int(bp_cls)]
                            if bp_class_name == "head":
                                bp_x1 = int(self.clipBboxBoundry(int(bp_x1),0, w_orig))
                                bp_x2 = int(self.clipBboxBoundry(int(bp_x2),0, w_orig))
                                bp_y1 = int(self.clipBboxBoundry(int(bp_y1),0, h_orig))
                                bp_y2 = int(self.clipBboxBoundry(int(bp_y2),0, h_orig))                                                        

                        if len(output):
                            kpts = output[0, 4:].T
                            #  create the output message
                            if bp_class_name == "head":
                                hpe_detection = self.create_hpe_detection_msg(w_orig, h_orig, x1, x2, y1, y2, kpts, bp_x1, bp_x2, bp_y1, bp_y2)
                            else:
                                hpe_detection = self.create_hpe_detection_msg(w_orig, h_orig, x1, x2, y1, y2, kpts, None, None, None, None)

                            detection_msg.people.append(hpe_detection)
                            num_people += 1
                            if visualize:
                                output[0, 7:][0::3] += x1
                                output[0, 7:][1::3] += y1
                                plot_skeleton_kpts(np_img_orig, output[0, 7:].T, 3)
                            
                                if len(bp_detections) > 0:
                                    draw_bp_bboxes(np_img_orig, self.bp_class_labels, bp_detections)
                        
                else:
                    detection = self.create_detection_msg(class_name, conf, w_orig, h_orig, x1, x2, y1, y2)                    
                    detection_msg.objects.append(detection)
                    num_objects += 1
            else:
                detection = self.create_detection_msg(class_name, conf, w_orig, h_orig, x1, x2, y1, y2)
                detection_msg.objects.append(detection)
                num_objects += 1

        if (num_objects + num_people) > 0:
            try:
                self.detection_publisher.publish(detection_msg)
            except Exception as e:
                rospy.logwarn(f"Yolo publish error {e}")
            if use_human_pose:
                # rospy.loginfo(f'Published {num_objects} objects and {num_people} person detections')
                pass
            else:
                # rospy.loginfo(f'Published {num_objects} objects')
                pass
                

        # # visualizing if required
        if self.visualization_publisher:

            bboxes = [[int(x1), int(y1), int(x2), int(y2)]
                      for x1, y1, x2, y2 in detections[:, :4].tolist()]        
            classes = [int(c) for c in detections[:, 5].tolist()]
            conf = [round(c,2) for c in detections[:, 4].tolist()]

            vis_img = draw_detections(np_img_orig, bboxes, classes,
                                      self.all_class_labels, conf)
            vis_msg = self.bridge.cv2_to_imgmsg(vis_img)

            self.visualization_publisher.publish(vis_msg)
        rospy.logwarn(f"Time taken: {rospy.Time.now().to_sec() - t_start.to_sec()}")

    def create_detection_msg(self, object_class, object_class_conf, cols, rows, x1, x2, y1, y2) -> ObjectDetection:
        single_detection_msg = ObjectDetection()

        single_detection_msg.object_class = object_class
        single_detection_msg.object_class_conf = object_class_conf
        bbox = BoundingBox()
        bbox.width = int(round(x2 - x1))
        bbox.height = int(round(y2 - y1))
        bbox.x = int(x1)
        bbox.y = int(y1)
        # original image size (cols/rows)
        bbox.cols = cols 
        bbox.rows = rows
        single_detection_msg.bbox = bbox        
        single_detection_msg.position = Point(0,0,0)

        return single_detection_msg

    def create_hpe_detection_msg(self, cols, rows, x1, x2, y1, y2, kpts, hd_x1, hd_x2, hd_y1, hd_y2) -> PersonDetection:
        single_detection_msg = PersonDetection()

        bbox = BoundingBox()
        bbox.width = int(round(x2 - x1))
        bbox.height = int(round(y2 - y1))
        bbox.x = int(x1)
        bbox.y = int(y1)
        # original image size (cols/rows)
        bbox.cols = cols 
        bbox.rows = rows        
        single_detection_msg.bbox_person = bbox
        single_detection_msg.position = Point(0,0,0)

        # add head bounding box if it exists
        if hd_x1 is not None and hd_x2 is not None and hd_y1 is not None and hd_y2 is not None:     
            bbox_head = BoundingBox()
            bbox_head.width = int(round(hd_x2 - hd_x1))
            bbox_head.height = int(round(hd_y2 - hd_y1))
            bbox_head.x = int(hd_x1)
            bbox_head.y = int(hd_y1)        
            single_detection_msg.bbox_head = bbox_head

        steps = 3
        num_kpts = len(kpts) // steps
        
        if kpts[2] > 0.5: 
            single_detection_msg.centroid = Point(x=kpts[0], y=kpts[1])

        for kid in range(1, num_kpts):
            x_coord, y_coord = kpts[steps * kid], kpts[steps * kid + 1]
            keypoint =  Point(x=0.0, y=0.0)
            if not (x_coord % 640 == 0 or y_coord % 640 == 0):
                conf = kpts[steps * kid + 2]
                if conf > 0.5:
                    keypoint.x = x_coord + x1
                    keypoint.y = y_coord + y1
            single_detection_msg.skeleton[kid - 1] = keypoint
        return single_detection_msg  

    def clipBboxBoundry(self, value:int,lower:int,upper:int):
            # Make the value for bounding box inside the image size. In the perspective of raw image size.
            if value < lower:
                return lower
            elif value > upper:
                return upper
            return value
    
if __name__ == "__main__":
    rospy.init_node("yolov7_node")

    ns = rospy.get_name() + "/"

    weights_path = rospy.get_param(ns + "weights_path")
    hpe_weights_path = rospy.get_param(ns + "hpe_weights_path")
    bp_weights_path = rospy.get_param(ns + "bp_weights_path")
    classes_path = rospy.get_param(ns + "classes_path")
    bp_classes_path = rospy.get_param(ns + "bp_classes_path")
    input_topic = rospy.get_param(ns + "input_topic")
    output_topic= rospy.get_param(ns + "output_topic")
    vis_topic = rospy.get_param(ns + "vis_topic")   
    conf_thresh = rospy.get_param(ns + "conf_thresh")
    iou_thresh = rospy.get_param(ns + "iou_thresh")
    queue_size = rospy.get_param(ns + "queue_size")
    img_size = rospy.get_param(ns + "img_size")
    visualize = rospy.get_param(ns + "visualize")
    use_human_pose = rospy.get_param(ns + "use_human_pose")
    device = rospy.get_param(ns + "device")

    # some sanity checks
    if not os.path.isfile(weights_path):
        raise FileExistsError(f"Weights not found ({weights_path}).")
    
    if not os.path.isfile(hpe_weights_path):
        raise FileExistsError(f"HPE Weights not found ({hpe_weights_path}).")
    
    if not os.path.isfile(bp_weights_path):
        raise FileExistsError(f"Body Part Weights not found ({bp_weights_path}).")
    
    if classes_path: 
        if not os.path.isfile(classes_path):
            raise FileExistsError(f"Classes file not found ({classes_path}).")
        classes = parse_classes_file(classes_path)
    else:
        rospy.loginfo("No class file provided. Class labels will not be visualized.")
        classes = None

    if bp_classes_path: 
        if not os.path.isfile(bp_classes_path):
            raise FileExistsError(f"Body Part Classes file not found ({bp_classes_path}).")
        bp_classes = parse_classes_file(bp_classes_path)
    else:
        rospy.loginfo("No class file provided. Bosy Part Class labels will not be visualized.")
        bp_classes = None


    if not ("cuda" in device or "cpu" in device):
        raise ValueError("Check your device.")
    
    rospy.loginfo("Yolo V7 Parameters Set")
    rospy.loginfo("Object Weights Path = {}".format(weights_path))
    rospy.loginfo("Human Pose Weights Path = {}".format(hpe_weights_path))
    rospy.loginfo("Body Part Weights Path = {}".format(bp_weights_path))
    rospy.loginfo("Classes Path = {}".format(classes_path))
    rospy.loginfo("Body Part Classes Path = {}".format(bp_classes_path))
    rospy.loginfo("Input Topic = {}".format(input_topic))
    rospy.loginfo("Output Topic = {}".format(output_topic))
    rospy.loginfo("Vis Topic = {}".format(str(vis_topic)))    
    rospy.loginfo("Confidence Threshold = {}".format(str(conf_thresh)))
    rospy.loginfo("IOU Threshold = {}".format(str(iou_thresh)))
    rospy.loginfo("Queueu Size = {}".format(str(queue_size)))
    rospy.loginfo("Image Size = {} x {}".format(str(img_size), str(img_size)))
    rospy.loginfo("Visualize = {}".format(str(visualize)))
    rospy.loginfo("Use Human Pose = {}".format(str(use_human_pose)))
    rospy.loginfo("Device = {}".format(device))


    publisher = Yolov7Publisher(
        input_topic=input_topic,
        output_topic=output_topic,
        vis_topic=vis_topic,
        weights=weights_path,
        hpe_weights=hpe_weights_path,
        bp_weights = bp_weights_path,
        device=device,
        visualize=visualize,
        conf_thresh=conf_thresh,
        iou_thresh=iou_thresh,
        img_size=(img_size, img_size),
        queue_size=queue_size,
        class_labels=classes,
        bp_class_labels=bp_classes,
        use_human_pose=use_human_pose
    )

    rospy.spin()
    
