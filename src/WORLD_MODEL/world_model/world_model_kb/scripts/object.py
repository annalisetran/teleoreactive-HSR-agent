import numpy as np

class Object(object):
    def __init__(self, point, frame_id=None, class_id=None, class_name=None, class_confidence=0, 
                 bbox_x=None, bbox_y=None, bbox_width=None, bbox_height=None, bbox_cols=None, bbox_rows=None, 
                 track=None, occluded=None):
        self.point = np.array(point)
        self.frame_id = frame_id
        self.assosciated_track = track
        self.class_confidence = class_confidence
        self.class_name = class_name
        self.class_id = class_id
        self.bbox_x = bbox_x
        self.bbox_y = bbox_y
        self.bbox_width = bbox_width
        self.bbox_height = bbox_height
        self.bbox_cols = bbox_cols
        self.bbox_rows = bbox_rows
        self.occluded = occluded

    def __str__(self):
        return f"Class Name: {self.class_name}; BBox: x_min={self.bbox_x} y_min={self.bbox_y} x_max={self.bbox_x+self.bbox_width} y_max={self.bbox_y+self.bbox_height}; Occluded={self.occluded}"

    def set_track(self, track):
        self.assosciated_track = track

    def get_track(self):
        return self.assosciated_track

    @staticmethod
    def convert_scene_object(point, frame_id, class_id, class_name, class_confidence, 
                             bbox_x, bbox_y, bbox_width, bbox_height, bbox_cols, bbox_rows, occluded):
        point = [point.x, point.y, point.z]
               
        #return DetectionObject(point=point, class_id=class_id, class_confidence=class_confidence, annotations=scn_obj)
        return Object(point=point, frame_id=frame_id, class_id=class_id, class_name=class_name, class_confidence=class_confidence,
                               bbox_x=bbox_x, bbox_y=bbox_y, bbox_width=bbox_width, bbox_height=bbox_height, bbox_cols=bbox_cols, bbox_rows=bbox_rows, occluded=occluded)

