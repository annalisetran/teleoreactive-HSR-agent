#!/usr/bin/env python3

from feature import Feature
import numpy as np

class Person:
    def __init__(self, id, tracking_id, bbox_person, bbox_head, skeleton, skeleton2d, position, features, name="unknown"):
        self.id = id
        self.tracking_id = tracking_id
        self.name = name
        self.bbox_person = bbox_person
        self.bbox_head = bbox_head
        self.skeleton = skeleton
        self.skeleton2d = skeleton2d
        self.position = position
        self.features = features or []


    # def __str__(self):
    #     features_str = "\n".join(map(str,self.features))
    #     #return f"id: {self.id}; name: {self.name}; bbox: {self.bbox_person}; skeleton: {self.skeleton}; position={self.position} \n{features_str}"
    #     # return f"{self.id}; name: {self.name}; bbox: {self.bbox_person}; skeleton: {self.skeleton}; position={self.position}"
    #     return f"id: {self.id}; name: {self.name}; "
    
    def print_features(self):
        for feature in self.features:
            # if feat["encoding"] is not None:
            print(f"{self.id}: {feature.perspective}; {feature.feature_type}")
    
    def print_feature_details(self):
        features_str = "\n".join(map(str,self.features))
        return f"{features_str}"

    def update_id(self, new_id):
        self.id = new_id

    def update_name(self, new_name):
        self.name = new_name

    def add_feature(self, new_feature):
        self.features.append(new_feature)


    def update_feature(self, upd_feature):
        i = self.find_feature(upd_feature.perspective, upd_feature.feature_type)
        
        if i is not None:
            self.features[i].encoding = upd_feature.encoding


    def find_feature(self, perspective, feature_type):
        i = 0
        for feat in self.features:
            if feat.perspective == perspective and feat.feature_type == feature_type:
                 return i
            i=i+1
            
        return None

    # get the encoding for the perspective and feature type
    def get_feature(self, perspective, feature_type):
        i = self.find_feature(perspective, feature_type)
        if i is not None:
            return self.features[i].encoding
        else: 
            return None

    def print_keypoints(self):
         for i in range(16):
             print(f"{i}: x={self.skeleton[i].x}; y={self.skeleton[i].y}; z={self.skeleton[i].z}")

    