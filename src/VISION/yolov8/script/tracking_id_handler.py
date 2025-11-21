#!/usr/bin/env python3

# Version: 1.0.0
# Author: Zijie Li

import rospy

from unsw_vision_msgs.msg import DetectionList, PersonDetection, ObjectDetection, BoundingBox

# the threshold for the person to be too close to the edge of the image(horizontally), as a ratio of the image width
EDGE_THRESHOLD = 0.15
FRAME_DEREGISTER_THRESHOLD = 5 # the number of frames that a person is not seen to be deregistered
class TrackingIdHandler:
    def __init__(self, start_tracking_id=1, order="incremental", width=640, height=480):
        self.tracking_id = 1
        self.order = order
        if order not in ["incremental", "decremental"]:
            raise ValueError("order should be either 'incremental' or 'decremental'")
        self.img_width = width
        self.img_height = height
        self.yolo_to_tracker_mapping = {}
        self.need_to_process = set()
        self.scene_processed_id = set()

    def next_tracking_id(self):
        if self.order == "incremental":
            self.tracking_id += 1
        else:
            self.tracking_id -= 1
        return self.tracking_id
    '''
    For a personDetection, check if it is in the tracking mapping, 
        if not, add it to the mapping if following conditions are met:
            - the person is not too close to the edge of the image
        if it is in the mapping, check following conditions:
            - if near the edge of the image, deregister it from the mapping
            - if the class changed. deregister it from the mapping
    Parameters: 
        detection: PersonDetection
    Retiurn:
        id:int - the tracking id for the person, it would be 0 if the person is too close to the edge of the image.
    '''
    def get_person_tracking_id(self, detection: PersonDetection) -> int:
        # get the bounding box centre of the detection
        x = detection.bbox_person.x + detection.bbox_person.width // 2
        y = detection.bbox_person.y + detection.bbox_person.height // 2
        self.scene_processed_id.add(detection.tracking_id)
        # check if the person is too close to the edge of the image
        # TODO: check the height if nessary
        if x < self.img_width * EDGE_THRESHOLD or x > self.img_width * (1 - EDGE_THRESHOLD):
            # remove the person from the mapping if exists
            if detection.tracking_id in self.yolo_to_tracker_mapping:
                self.yolo_to_tracker_mapping.pop(detection.tracking_id)
            
            # no tracking id for the person
            return -1
        else:
            # here the person is not too close to the edge of the image
            # check if the person is in the in a valid tracking mapping
            if detection.tracking_id in self.yolo_to_tracker_mapping:
                # check if the class changed
                if self.yolo_to_tracker_mapping[detection.tracking_id]["class"] != "Person":
                    # remove the person from the mapping if exists and add it back with a new tracking id and class
                    self.yolo_to_tracker_mapping.pop(detection.tracking_id)
                    self.yolo_to_tracker_mapping[detection.tracking_id] = {"custom_tracking_id":self.next_tracking_id(), "class":"Person", "not_seen":0}
                # make sure the not seen counter is reset
                else:
                    self.yolo_to_tracker_mapping[detection.tracking_id]["not_seen"] = 0
            else:
                # add the person to the mapping
                self.yolo_to_tracker_mapping[detection.tracking_id] = {"custom_tracking_id":self.next_tracking_id(), "class":"Person", "not_seen":0}
        return self.yolo_to_tracker_mapping[detection.tracking_id]["custom_tracking_id"]
    '''
    Same as get_person_tracking_id, but for ObjectDetection
    '''
    def get_object_tracking_id(self, detection: ObjectDetection) -> int:
        # get the bounding box centre of the detection
        x = detection.bbox.x + detection.bbox.width // 2
        y = detection.bbox.y + detection.bbox.height // 2
        self.scene_processed_id.add(detection.tracking_id)
        # check if the object is too close to the edge of the image
        # TODO: check the height if nessary
        if x < self.img_width * EDGE_THRESHOLD or x > self.img_width * (1 - EDGE_THRESHOLD):
            # remove the person from the mapping if exists
            if detection.tracking_id in self.yolo_to_tracker_mapping:
                self.yolo_to_tracker_mapping.pop(detection.tracking_id)
            
            # no tracking id for the object
            return -1
        else:
            # here the object is not too close to the edge of the image
            # check if the object is in the in a valid tracking mapping
            if detection.tracking_id in self.yolo_to_tracker_mapping:
                # check if the class changed
                if self.yolo_to_tracker_mapping[detection.tracking_id]["class"] != detection.object_class:
                    # remove the object from the mapping if exists and add it back with a new tracking id and class
                    self.yolo_to_tracker_mapping.pop(detection.tracking_id)
                    self.yolo_to_tracker_mapping[detection.tracking_id] = {"custom_tracking_id":self.next_tracking_id(), "class":detection.object_class, "not_seen":0}
                # make sure the not seen counter is reset
                else:
                    self.yolo_to_tracker_mapping[detection.tracking_id]["not_seen"] = 0
            else:
                # add the object to the mapping
                self.yolo_to_tracker_mapping[detection.tracking_id] = {"custom_tracking_id":self.next_tracking_id(), "class":detection.object_class, "not_seen":0}
                
        return self.yolo_to_tracker_mapping[detection.tracking_id]["custom_tracking_id"]

    '''
    trigger a scene end event for the tracking id handler
    any tracking in the mapping is not detected in the scene/frame will be updated/removed
    reset the status for need to process and scene processed id
    param: None
    return: None
    '''
    def end_scene(self):
        # compare the scene processed id with the need to process id
        # if there is any id that is not processed, remove it from the mapping
        for id in self.need_to_process - self.scene_processed_id:
            exist_mapping = self.yolo_to_tracker_mapping.pop(id)
            exist_mapping["not_seen"] += 1
            # if the person is not seen for 5 frames, remove it from the mapping, otherwise, keep it
            if exist_mapping["not_seen"] < FRAME_DEREGISTER_THRESHOLD:
                self.yolo_to_tracker_mapping[id] = exist_mapping
        
        # reset the status for need to process and scene processed id
        self.need_to_process = set(self.yolo_to_tracker_mapping.keys())
        self.scene_processed_id.clear()