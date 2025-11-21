import numpy as np
import cv2
import sys
import os
from tensorflow.keras.applications import VGG19
from tensorflow.keras.models import Model
from tensorflow.keras.layers import GlobalAveragePooling2D
from sklearn.neighbors import NearestNeighbors
from keras.applications.vgg19 import preprocess_input

module_path = os.environ.get("UNSW_WS")
sys.path.append(module_path + "/WORLD_MODEL/world_model/world_model_kb/scripts")
from dbase import DBase
from object_class_api import ObjectClassApi

class ObjectFeatureExtractor():
    def __init__(self, debug, db_ini, shape_w, shape_h, feat_threshold):

        self.debug = debug
        self.db_ini = db_ini
        self.shape_w = shape_w
        self.shape_h = shape_h
        self.feat_threshold = feat_threshold

        self.obj_class = ObjectClassApi()
        self.db = DBase(os.path.abspath(self.db_ini))               # database connections

        self.feature_extractor = self.init_feature_extractor()


    def init_feature_extractor(self):
        # Define the base CNN model (e.g., VGG19, ResNet50)       
        base_model = VGG19(
            input_shape=(self.shape_w, self.shape_h, 3),
            include_top=False,
            weights='imagenet'
        )                 
        global_avg_pooling_layer = GlobalAveragePooling2D()(base_model.output)
        feature_extractor_model = Model(inputs=base_model.input, outputs=global_avg_pooling_layer)
        return feature_extractor_model  
    
    '''
    This method extracts one feature from the object of interest.
    
    Parameters:
    img =    a cropped image (based on the bounding box)

    Returns:
    An array consisting of the features
    '''
    def extract_feature(self, img):
        # resize the image in case it has been adjusted from the crop
        resized_img = cv2.resize(img, (self.shape_w, self.shape_h))
        # Load and preprocess the image
        resized_img = preprocess_input(resized_img)
        features = self.feature_extractor.predict(np.expand_dims(resized_img, axis=0))
        # normalize the feature vector
        normalized_features = features / np.linalg.norm(features)
        return normalized_features
    
 
    '''
    Compares the features of the new information to that in existing/known object sub classes

    Parameters:
    new_feature  = feature that we want to compare with known features from object_sub_classes
    feature_list = the list of features for the known object_sub_classes

    Return:
    if matched found then 
        the id of the matched object_sub_class
    else  
        None
    '''
    def match_features(self, new_feature, feature_list):        
        if len(feature_list) > 0:            
            # Separate the IDs and the feature vectors
            osc_ids, features = zip(*feature_list)
            # Convert features to a 2D array
            features = np.vstack(features)
            # Fit a NearestNeighbors model on agent_list_features
            neighbors = NearestNeighbors(n_neighbors=1).fit(features)
            # Find the nearest neighbor to features
            # distances, indices = neighbors.kneighbors(features.reshape(1, -1))
            distances, indices = neighbors.kneighbors(new_feature)

            # Get the distance and index of the nearest neighbor
            min_distance = distances[0][0]
            min_index = indices[0][0]

            # print(f"Min feature distance = {min_distance}; {min_index}")
            if self.debug:
                print(f"Distances: {distances}")

            if min_distance <= self.feat_threshold:     # found a match  
                matching_object_id = osc_ids[min_index]
                return matching_object_id
            else:
                return None
        else:
            return None
        
    '''
    Insert the new feature into the object_sub_class_features
    
    **Note: this method should only be used when setting up the object_sub_classes
    '''
    def insert_feature(self, img, bbox, object_sub_class_id):
        crop = self.crop_bbox(img, bbox)

        new_feature = self.extract_feature(crop)

        self.obj_class.insert_object_sub_class_feature(self.db.con_pool, object_sub_class_id, new_feature)

    def crop_bbox(self, img, bbox):
        if bbox is not None:
            # define top and height of crop
            y = bbox.y
            h = int(bbox.width)
            # define the width of the crop
            x = bbox.x 
            w = bbox.width  

            return img[y:y+h,x:x+w] 
        else: 
            return None
        