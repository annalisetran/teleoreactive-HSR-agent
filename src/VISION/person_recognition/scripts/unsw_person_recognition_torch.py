#!/usr/bin/env python3

# Person Recoginition 
# The person recognition node is used to capture and re-identify people in an image. 
# The image can be a video stream frame.The idea is to be able to create a node where components 
# can be added and removed in a modular way
#
# Person recognition is made up of a number components:
# Face Recognition - recognising the specific features on a human face
# Feature recognition - capturing different features e.g. shirt texture
#
# This code can be run with or without the World Model
#
# Libraries:
# face recognition - https://github.com/ageitgey/face_recognition

# Dependencies:
# World Model - database with schema 2.1.14 or higher which contains the agent table structures - use with flag: use_world_model = True

# Written By: Adam Golding

from datetime import datetime
import rospy
import sys
import os
import random
from std_msgs.msg import String
from unsw_vision_msgs.msg import DetectionList, PersonDetection, BoundingBox
from pathlib import Path
import cv2
from cv_bridge import CvBridge
import numpy as np
from sklearn.neighbors import NearestNeighbors
from sklearn.decomposition import PCA
from sklearn.metrics.pairwise import cosine_similarity
import torch
import torch.nn as nn
import torchvision.models as models
import torchvision.transforms as transforms
from torch.autograd import Variable

module_path = os.environ.get("UNSW_WS")
sys.path.append(module_path + "/WORLD_MODEL/world_model_kb/scripts")
from agent_api import AgentApi
from dbase import DBase
from person import Person
from feature import Feature
import torchreid    
import face_recognition

class PersonRecognition():
    def __init__(self, db_ini, scale, feature_threshold, face_threshold, use_world_model, use_gpu, save_img, img_path) -> None:
        self.db_ini = db_ini
        self.scale = scale
        self.feature_threshold = feature_threshold
        self.face_threshold = face_threshold
        self.use_world_model = use_world_model
        self.save_img = save_img
        self.use_gpu = use_gpu
        self.img_path = img_path
        self.feature_type_list = ["face", "head"]
        self.use_statistics = True      # flag indicating to gather statistics
        self.stats_match_topic = "/unsw/person_recognition/stats_match"
        self.stats_distance_topic = "/unsw/person_recognition/stats_distance"

        seed = 42
        np.random.seed(seed)
        # tf.random.set_seed(seed)
        torch.manual_seed(seed)
        if torch.cuda.is_available():
            torch.cuda.manual_seed_all(seed)
        random.seed(seed)        

        # self.feature_type_list = ["face", "shirt", "pants", "head"]
        self.feature_attr_list = ["encoding"]
        self.perspective_list = ["front", "front-left", "back-left", "back", "back-right", "front-right"]
        # cnn_gpu_options = tf.compat.v1.GPUOptions(per_process_gpu_memory_fraction=0.3)  # Allocate 10% of GPU memory for VGG model
        # cnn_config = tf.compat.v1.ConfigProto(gpu_options=cnn_gpu_options)
        # tf.compat.v1.Session(config=cnn_config)
        self.device = torch.device("cuda:0" if torch.cuda.is_available() and self.use_gpu else "cpu")
        self.agent_api = AgentApi()
        self.db = None
        self.crop_w = 128
        self.crop_h = 128   
        self.bbox_threshold = 1200     # the area of the bounding box for the head  
        self.aspect_ratio = (11/20)    # the aspect ratio of the bounding box for the head
        # self.feature_extractor = self.init_feature_extractor()
        self.feature_extractor = torchreid.utils.FeatureExtractor(
            model_name="pcb_p6",
            model_path='',  # Use default pre-trained weights
            device='cuda' if self.use_gpu else 'cpu'
        )
        
        # Define preprocessing for PyTorch
        self.preprocess = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

        self.bridge = CvBridge()
 
        # YoloV8 generates a temporary tracking id which is included in the PersonDetection msg
        # The Person object contains a tracking id which is used within the known_agents list to 
        # manage the person id to tracking id relationship  
        self.known_agents = []      # list of known Person objects        

        # Create an empty RGB image to use in starting face locations
        # This is started in the init method as it speeds up the processing of the first frame
        empty_image = np.zeros((100, 100, 3), dtype=np.uint8)

        # Call face_locations with the empty image
        face_recognition.face_locations(empty_image, model="cnn")

        np.set_printoptions(threshold=np.inf)
        # if world model used load agent data from database
        if self.use_world_model:
            self.db = DBase(os.path.abspath(self.db_ini))               # database connections           
            self.known_agents = self.agent_api.load_agent_list(self.db.con_pool)
            self.print_known_agents()

        if self.use_statistics:
            self.stats_match_publisher = rospy.Publisher(self.stats_match_topic, String, queue_size=100)
            self.stats_distance_publisher = rospy.Publisher(self.stats_distance_topic, String, queue_size=100)

        rospy.loginfo(f"Number of existing agents loaded from the world model = {len(self.known_agents)}")

        # statistics
        # if self.use_statistics:
        #     self.person_tracking_update_list = []  # list of people that have been matched using tracking id
        #     self.person_face_update_list = []      # list of people that have been matched using face features
        #     self.person_head_update_list = []      # list of people that have been matched using head features

        self.img_with_ppl_cnt = 0
        self.total_people_cnt = 0
        self.new_agent_cnt = 0
        self.update_agent_cnt = 0
        self.occ_head_cnt = 0
        self.small_head_cnt = 0
        self.head_aspect_ratio_cnt = 0
        self.match_track_cnt = 0
        self.match_face_cnt = 0
        self.match_head_cnt = 0

    
    # def init_feature_extractor(self):
    #     # Define the base CNN model using PyTorch VGG19
    #     vgg19 = models.vgg19(pretrained=True)
    #     base_model = nn.Sequential(*list(vgg19.features.children()))
        
    #     # Add pooling and dense layer
    #     base_model.add_module('global_avg_pool', nn.AdaptiveAvgPool2d((1, 1)))
    #     base_model.add_module('flatten', nn.Flatten())
    #     base_model.add_module('dense', nn.Linear(512, 128))
    #     base_model.add_module('relu', nn.ReLU())
        
    #     # Move to device and set to evaluation mode
    #     base_model = base_model.to(self.device)
    #     base_model.eval()
        
    #     return base_model
    
    def process_scene(self, in_msg):               
        ### Image Preparation
        img = self.bridge.imgmsg_to_cv2(in_msg.person_seg_image, desired_encoding="bgr8")  
        debug_img = img.copy()  # make a copy of the image for debugging
        # img = self.sharpen_img(orig_img)
        # img = self.unsharp_mask(orig_img)

        if self.save_img:
            self.save_image(img, "orig")
            # self.save_image(img,"sharp")

        # Resize frame of video to the scale size for faster face recognition processing
        # The larger the scaling (i.e. smaller fraction) the less faces will be detected and 
        # the closer the people will need to be to the camera. 
        # the scale size is set in the launch file

        # TODO: mask out everything that is not a person??
        # can we segment all people in the image to remove background rather than bounding box?
        small_frame = cv2.resize(img, (0, 0), fx=self.scale, fy=self.scale)
            
        # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
        rgb_small_frame = np.ascontiguousarray(small_frame[:, :, ::-1])

        if self.use_gpu:
            face_locations = face_recognition.face_locations(rgb_small_frame, model="cnn")
        else:
            face_locations = face_recognition.face_locations(rgb_small_frame)

        face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations, model="large")
        print(f"People in image: {len(in_msg.people)}; Heads detected: {self.count_heads(in_msg)}; Faces detected: {len(face_encodings)}; Known People: {len(self.known_agents)}")
    
        person_list = []    # list of people found in the image

        for person in in_msg.people:
            self.img_with_ppl_cnt += 1  # update statistics
            rospy.logwarn(f"### Person Tracking Id: {person.tracking_id} person occluded: {person.bbox_person.occluded} head occluded: {person.bbox_head.occluded}")      
            print(f"Head Size: {person.bbox_head.width * person.bbox_head.height}")      
            # if the person is occluded it is difficult to extract and compare a clean set of features
            # head.occluded occurs when the bounding box for the head is againts the edge of the image
            # i.e. bbox_head.x == 0 or bbox_head.y == 0 or bbox_head.x + bbox_head.width == img_width or bbox_head.y + bbox_head.height == img_height
            # tracking_id is never re-used when generated in yolo
            # tracking_id = -1 when there is not valid track assigned to the person in the msg

            if person.bbox_head.occluded: # and len(face_encodings) == 0:
                print("head is occluded")
                self.occ_head_cnt += 1
                continue
            elif (person.bbox_head.width * person.bbox_head.height) < self.bbox_threshold:   
                print(f"head is too small {person.bbox_head.width * person.bbox_head.height}")
                self.small_head_cnt += 1
                continue
            elif person.bbox_head.height != 0 and (person.bbox_head.width / person.bbox_head.height) < self.aspect_ratio:
                print(f"head aspect ratio is too narrow - aspect ratio: {person.bbox_head.width / person.bbox_head.height}")
                self.head_aspect_ratio_cnt += 1
                continue
            else:
                # extract the features of the person of interest
                # if we find a perspective we return a person object with features
                # else we return None
                poi = self.extract_person_features(img, person, face_locations, face_encodings, debug_img = debug_img)  
                if poi is not None and poi.features is not None:     
                    agent_id = self.is_tracked(person.tracking_id)      
                    if agent_id is not None:
                        # update agent 
                        print(f"Found a match based on tracking id {person.tracking_id}")
                        poi.id = agent_id
                        self.update_agent(poi) 
                        if self.use_statistics:
                            self.stats_match_publisher.publish(f"match, tracking, {poi.id}, {poi.tracking_id}")               
                    else:
                        poi = self.match_person(poi)  # match the person to known people/agents, returns the matched person with id                           
                        # poi.tracking_id = person.tracking_id
                        if poi.id is None:      # could not find a match
                            # add agent 
                            new_id = self.add_agent(poi)                            
                            print(f"Adding Agent Id: {poi.id} new_id: {new_id}")
                            # if _person.bbox_head is not None:
                                # self.display_person(img, _person.bbox_head)
                        else:   # found a match in the known agents
                            # update agent                             
                            self.update_agent(poi)

                        # add to the list of people in the current frame                        
                        # person_list.append(poi)
                    person.id = str(poi.id)  if poi.id else ""
                    person_list.append(person)  

        in_msg.people = person_list            

        # new_msg = self.create_output_message(in_msg, person_list)
        # return new_msg
        cv2.imshow("person-recog-bbox", debug_img)
        cv2.waitKey(1)

        return in_msg

    '''
    Function to extract person and features from a detected person in an image. 
    The person object with features is returned for matching against known people.
    The intention is that this function can be expanded to include additional feature extraction processes.

    Parameters:
    img     - the original image
    img_person  - the person of interest in the original image
    face_locations  - list of face locations within the image
    face_encodings  - list of face encoding data for faces within the image

    Return:
    Person object containing details and features that have been extracted from the person of interest
    '''
    def extract_person_features(self, img, person_msg, face_locations, face_encodings, debug_img=None):
        # try:
            _person = Person(None, person_msg.tracking_id, person_msg.bbox_person, person_msg.bbox_head, person_msg.skeleton, person_msg.skeleton2d, person_msg.position, None)  
            # get the perspective of the person so we know how the robot sees them
            # (front, front-left, left, back-left, back, back-right, right, front-right)
            perspective = self.get_perspective(person_msg.skeleton2d)
            print(f"Perspective = {perspective}")
            has_feature = False
            # if we detect the perspective we can try to extract features 
            if perspective is not None:                      
                for feature_type in self.feature_type_list: 
                    crop_img = None # reset each iteration
                    encoding = None                                        
                    if feature_type == "face" and (perspective == "front" or perspective == "front-left" or perspective == "front-right"):
                        if len(face_encodings) > 0:                      
                            # try to align the face with the skeleton 
                            _face_encoding = self.align_face_to_head(person_msg, face_locations, face_encodings)   
                            if face_encodings is not None:                                
                                new_feat = Feature(perspective, feature_type, _face_encoding)
                                _person.add_feature(new_feat)
                                has_feature = True
                    else:                    
                        # crop the region of interest
                        if feature_type == "shirt":                    
                            crop_img = self.crop_roi(img, _person.bbox_head) 
                            if self.save_img:
                                self.save_image(crop_img, perspective)  
                        elif feature_type == "pants":
                            pass
                        elif feature_type == "head" and _person.bbox_head.occluded == False:
                            crop_img = self.crop_roi_head(img, _person, debug_img=debug_img) 
                            if self.save_img:
                                self.save_image(crop_img, perspective) 

                        # extract the feature
                        if crop_img is not None:
                            encoding = self.get_feature(crop_img)   
                            new_feat = Feature(perspective, feature_type, encoding)                                                
                            _person.add_feature(new_feat)
                            has_feature = True
                if has_feature == False:
                    print("No features found for person")
                    _person = None
            else:
                _person = None
            
            return _person
        # except Exception as e:
        #     rospy.logwarn(f"Extract Person Features Error: {e} ")
        #     # print("Extract Person Features Error: ",e)
        #     return None

    '''
    Align the face with the head of the person (using face bbox and skeletal keypoints)
    use keypoints 0,1,2 = 0 (nose), 1 (right eye) and 2 (left eye)     
    For the person of interest we search all existing faces
    The face bbox is generated by the DLIB library which uses a diferent format to the BoundingBox msg
    The face bbox needs to be converted to the BoundingBox msg format
    Check to see if ANY of the 3 keypoints exist inside the face bounding box
    i.e. 0 or 1 or 2
    
    Parameters:
    bbox = bounding box of the face
    person = the person from the message which includes skeletal keypoints

    Returns:
    Tuple - matched bounding box of face, face encoding
    '''
    def align_face_to_head(self, poi, face_locations, face_encodings):      
        for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):  
            # convert dlib bbox coordinates to BoundingBox msg format
            x = left
            y = top
            width = right-left
            height = bottom-top
            # rescale (to the original image size) the bounding box coordinates for the face
            bbox = self.rescale_bbox(x, y, width, height, self.scale)            

            x1 = bbox.x 
            y1 = bbox.y
            x2 = bbox.x + bbox.width        
            y2 = bbox.y + bbox.height
            inside = False   #initialise

            # check to see if keypoint are within the bounding box of the face
            # we are only interested in the face keypoints 0 - nose, 1 - left eye, 2 - right eye
            i=0
            while i < 3 and inside == False:
                if poi.skeleton2d[i].x != 0.0 and poi.skeleton2d[i].y != 0.0:                    
                    if int(poi.skeleton2d[i].x) > int(x1) \
                        and int(poi.skeleton2d[i].x) < int(x2) \
                        and int(poi.skeleton2d[i].y) > int(y1) \
                        and int(poi.skeleton2d[i].y) < int(y2): 

                        inside = True

                i += 1
            
            if inside:
                return face_encoding
        return None

    '''
    Function to compare all features of a person against known people/agents.

    If we can positively match the face we don't need to match against any other features

    Parameters:
    person - the detected person which we are trying to match to known people

    Return:
    The matched person
    '''
    def match_person(self, poi):
        # check the face features first as this gives a more confident match
        for feat in poi.features:
            if feat.feature_type == "face":
                poi.id = self.match_face(feat.encoding)
                
                if poi.id is not None:
                    print(f"Found a match based on face recognition {poi.id}")
                    if self.use_statistics:
                        self.stats_match_publisher.publish(f"match, face, {poi.id}, {poi.tracking_id}") 
                    return poi    # facial features were extracted    
                
        print(f"Face not matched")           

        # if the face was not matched then we need to check the other features
        for feat in poi.features:
            if feat.feature_type != "face":
                poi.id = self.match_features_cosine(feat.feature_type, feat.encoding)         

                if poi.id is not None:
                    print(f"Found a match based on feature recognition {poi.id}")
                    if self.use_statistics:
                        self.stats_match_publisher.publish(f"match, head, {poi.id}, {poi.tracking_id}") 
                    return poi             

        return poi

    '''
    Compares the new face to that in existing/known people

    Parameters:
    face_encoding = the new face_encoding we are comparing to known face encodings

    Return:
    if matched found then the id of the matched agent/person
    if no match then None
    '''
    def match_face(self, new_face_encoding):
        face_distances, agent_idx_list = self.face_distance(new_face_encoding)

        if self.use_statistics:
            self.stats_distance_publisher.publish(f"distance, face, {agent_idx_list}; {face_distances}")

        print(f"Agent: {agent_idx_list}")        
        print(f"Face Distances: {face_distances}")

        if face_distances is not None: 
            if len(face_distances) > 0:    
                min_idx = np.argmin(face_distances)
                if face_distances[min_idx] <= self.face_threshold:    # found a match
                    return agent_idx_list[min_idx]
                else:
                    return None
            else:
                return None
        else:
            return None       

    def match_features_cosine(self, poi_feature_type, poi_feature_encoding):
        similarity_list, agent_idx_list = self.get_feature_similarity(poi_feature_type, poi_feature_encoding)

        if self.use_statistics:
            self.stats_distance_publisher.publish(f"cosine, head, {agent_idx_list}; {similarity_list}")

        print(f"Agent: {agent_idx_list}")        
        print(f"Head Similarity: {similarity_list}")

        if similarity_list is not None: 
            if len(similarity_list) > 0:    
                max_idx = np.argmax(similarity_list)
                print(f"Min Idx: {max_idx}; Similarity: {similarity_list[max_idx]}; Threshold: {self.feature_threshold}")                
                if 1 - similarity_list[max_idx] <= self.feature_threshold:    # <= for distance, >= for similarity, here we use 1 - similarity to get distance so both match feature are consistenly using feature_threshold.                                         
                    return agent_idx_list[max_idx]
                else:
                    return None
            else:
                return None
        else:
            return None

    def match_features(self, poi_feature_type, poi_feature_encoding):
        dist_list, agent_idx_list = self.get_feature_distance(poi_feature_type, poi_feature_encoding)

        if self.use_statistics:
            self.stats_distance_publisher.publish(f"distance, head, {agent_idx_list}; {dist_list}")

        print(f"Agent: {agent_idx_list}")        
        print(f"Head Distances: {dist_list}")

        if dist_list is not None: 
            if len(dist_list) > 0:    
                min_idx = np.argmin(dist_list)
                if dist_list[min_idx] <= self.feature_threshold:    # found a match                                          
                    return agent_idx_list[min_idx]
                else:
                    return None
            else:
                return None
        else:
            return None

    def get_feature_distance(self, poi_feature_type, poi_feature_encoding):
        dist_list = []
        id_list = []
        for agent in self.known_agents:
            for feat in agent.features:
                if feat.feature_type == poi_feature_type:
                    distance = np.linalg.norm(feat.encoding - poi_feature_encoding)
                    dist_list.append(distance)
                    id_list.append(agent.id)

        return dist_list, id_list

    def get_feature_similarity(self, poi_feature_type, poi_feature_encoding):
        similarity_list = []
        id_list = []
        for agent in self.known_agents:
            for feat in agent.features:
                if feat.feature_type == poi_feature_type:
                    # Ensure the encodings are 1D arrays
                    feat_encoding_1d = np.ravel(feat.encoding)
                    poi_feature_encoding_1d = np.ravel(poi_feature_encoding)
                    
                    similarity = cosine_similarity([feat_encoding_1d], [poi_feature_encoding_1d])[0][0]
                    similarity_list.append(similarity)
                    id_list.append(agent.id)

        return similarity_list, id_list

    '''
    Determine the perspective of the person from the robot perspective 
    The perspective can only be determined if there are enough skeleton keypoints available to analyse
    
    Parameters:
    kp = skeletal keypoints. The keypoints represent joints in the human body. 
        Yolov7 Human Pose Estimation produces up to 17 skeletal keypoints

    The keypoints being used are:
    kp[0] = nose
    kp[1] = left eye
    kp[2] = right eye
    kp[3] = left ear
    kp[4] = right ear

    when the keypoint x value == 0 then it is not visible

    There are always 2 option when determining the perspective:
    1. if the shoulders and head are visible
    2. if only the head is visible

    Return:
    a string description of the perspective    
    '''
    def get_perspective(self, kp):    
        # self.print_kp(kp)       
        if (kp[4].x == 0 and kp[1].x > 0 and kp[2].x > 0 and kp[3].x > 0 and kp[3].x > kp[1].x and kp[0].x > 0):
            return "front-left"  
        elif (kp[3].x == 0 and kp[2].x > 0 and kp[1].x > 0 and kp[4].x > 0 and kp[2].x > kp[4].x and kp[0].x > 0):
            return "front-right"         
        elif (kp[4].x == 0 and kp[1].x > 0 and kp[3].x > 0 and kp[3].x > kp[1].x and kp[0].x > 0):
            return "left" 
        elif (kp[3].x == 0 and kp[2].x > 0 and kp[4].x > 0 and kp[2].x > kp[4].x and kp[0].x > 0):
            return "right"          
        elif (kp[3].x > 0 and kp[4].x > 0 and kp[3].x > kp[4].x):
            return "front"     
        elif (kp[3].x > 0 and kp[4].x > 0 and kp[4].x > kp[3].x):
            return "back"  
        elif (kp[4].x == 0 and kp[3].x > 0):
            return "back-left"                   
        elif (kp[3].x == 0 and kp[4].x > 0):
            return "back-right"           
        else:
            return None  
 
    '''
    Add the person to the list of known agents.
    Also add the agent to the world model if the flag has been set to true

    Parameters:
    person  - the new agent that we need to add. the data will represent face/features for the observed perspective
    '''
    def add_agent(self, poi):
        print("Adding new agent")
        poi.id  = len(self.known_agents)+1   
        if poi.tracking_id == -1:
            poi.tracking_id = None   
        self.known_agents.append(poi)

        if self.use_world_model:
            self.agent_api.insert_agent(self.db.con_pool, "P", poi)

        if self.use_statistics:
            self.stats_match_publisher.publish(f"match, add, {poi.id}, {poi.tracking_id}") 
        
        return poi.id
         
    '''
    Update an existing agent in the list of known agents.
    Also update the agent to the world model if the flag has been set to true

    Parameters:
    person  - the spefic agent that we need to update. the data will represent face/features for the observed perspective
    '''
    def update_agent(self, poi):
        for knwn_agent in self.known_agents:
            if knwn_agent.id == poi.id:
                if poi.tracking_id > 0:     # if it has a new tracking id
                    knwn_agent.tracking_id = poi.tracking_id

                print(f"Updating Agent Id: {knwn_agent.id} Tracking Id: {knwn_agent.tracking_id}")
                self.update_agent_cnt += 1
                if self.use_world_model:
                    self.agent_api.update_agent_last_seen(self.db.con_pool, poi.id, poi.position)
                # check if feature currently exists        
                for new_feat in poi.features:
                    found_features = False      # initialise each time
                    if new_feat.encoding is not None:
                        for known_feat in knwn_agent.features:                                          
                            if known_feat.perspective == new_feat.perspective and known_feat.feature_type == new_feat.feature_type:
                                known_feat.encoding = new_feat.encoding
                                found_features = True
                    
                        # if it is a new perspective/feature add it                   
                        if found_features == False:
                            # rospy.loginfo(f"Adding new feature for {person.id}: {new_feat.perspective} {new_feat.feature_type}")
                            feat = Feature(new_feat.perspective, new_feat.feature_type, new_feat.encoding)
                            knwn_agent.add_feature(feat)
                            print(f"Adding new feature for {poi.id}: {new_feat.perspective} {new_feat.feature_type}")
                       
                            if self.use_world_model:
                                self.agent_api.insert_agent_feature(self.db.con_pool, poi.id, new_feat)
                        else:
                            print(f"Updating existing feature for {poi.id}: {new_feat.perspective} {new_feat.feature_type}")
                            if self.use_world_model:
                                self.agent_api.update_agent_feature(self.db.con_pool, poi.id, new_feat)

    def crop_roi_head(self,img, person, debug_img=None):
        if self.check_img_size(img) > 0:
            if person.bbox_head is not None:
                # Extract person ROI from the full image
                x, y = int(person.bbox_head.x), int(person.bbox_head.y)
                w, h = int(person.bbox_head.width), int(person.bbox_head.height)
                
                # Ensure coordinates are within image boundaries
                x = max(0, x)
                y = max(0, y)
                x_end = min(img.shape[1], x + w)
                # expand it vertically to include certain joints
                y_end = min(img.shape[0], max(y + h, int(person.skeleton2d[5].y), int(person.skeleton2d[6].y)))
                # expand vertically by 1.2 to include more features
                y_end = int(y + ((y_end - y) * 1.2))
                debug_img = cv2.rectangle(debug_img, (x, y), (x_end, y_end), (0, 255, 0), 2)
                return img[y:y_end, x:x_end]
            else:
                return None
        else:
            return None
 
    '''
    This function takes a small crop of the head area around the specified perspective.
    
    Parameters:
    img = the image frame being examined
    bbox_head = the bounding box of the head
    perspective = the perspective that has been detected

    Returns:
    if the region of interest has been detected (i.e. head is visible)
        the cropped region of interest
    else
        None
    '''
    def crop_roi(self, img, bbox):   
        try:
            if self.check_img_size(img) > 0:
                # want to make the crop width a few pixels smaller on each side to minimise potential noise
                # define the height of the crop
                # if the bounding box for the head is available then use it to find the top of the head

                if bbox is not None:
                    # define top and height of crop
                    y = bbox.y
                    h = int(bbox.height)
                    # define the width of the crop
                    x = bbox.x 
                    w = bbox.width  

                    return img[y:y+h,x:x+w] 
                else: 
                    return None   
            else:
                return None             

        except Exception as e:
            print(f"Crop ROI Error: x {x}, y {y}, w {w}, h {h}, {e} ")
            return None
        
    # CNN Feature extractor with PyTorch
    def get_feature(self, img):
        if self.check_img_size(img) > 0:
            # resize the image in case it has been adjusted from the crop
            resized_img = cv2.resize(img, (self.crop_w, self.crop_h))
            # Convert from BGR to RGB (PyTorch expects RGB)
            resized_img = cv2.cvtColor(resized_img, cv2.COLOR_BGR2RGB)
            # Preprocess the image for PyTorch
            img_tensor = self.preprocess(resized_img)
            img_tensor = img_tensor.unsqueeze(0).to(self.device)
            
            # Get features with no_grad for inference
            with torch.no_grad():
                features = self.feature_extractor(img_tensor)
                
            # Convert to numpy and normalize
            features_np = features.cpu().numpy()
            normalized_features = features_np / np.linalg.norm(features_np)
            return normalized_features
        else:
            return None
        
    '''
    Function to calculate the distances between the new face encoding and all known face encodings
    This gets the distances for face encodings for each perspective so there may be multiple rows for each agent 
    depending on how many perspectives of the face we have

    Parameters: 
    new_face - the new face from the current image

    Return:
    a list of distance to each known face
    a list of index to known_agentto each known face
    '''
    def face_distance(self, new_face):
        face_distances = [] 
        agent_idx_list = []
        result = None
        
        if new_face is not None:                                  
            for agent in self.known_agents:                
                for agt_feat in agent.features:
                    if agt_feat.feature_type == "face" and agt_feat.encoding is not None:            
                        face_distances.append(agt_feat.encoding)
                        agent_idx_list.append(agent.id)
                        
            result = face_recognition.api.face_distance(face_distances, new_face)
            
        return result, agent_idx_list                       

    '''
    Method to determine if the current tracking id is associated with an agent
    Returns the Person Id of the tracked person/agent
    '''
    def is_tracked(self, tracking_id):
        for agent in self.known_agents:
            if agent.tracking_id == tracking_id:
                return agent.id
        
        return None


    #############################################################
    #
    # The following methods are used as helpers
    #
    #############################################################

    '''
    An image is generally scaled prior to processing to improve performance.
    This function rescales the bounding box coordinates to match the size of the original image

    Parameters:
    x = the x coordinate of the scaled bounding box
    y = the y coordinate of the scaled bounding box
    width = the width of the scaled bounding box
    height = the height of the scaled bounding box
    scale = the scaling factor

    Return:
    rescaled bounding box
    '''
    def rescale_bbox(self, x, y, width, height, scale):
        bbox = BoundingBox()
        bbox.width = int(width*(1/scale))
        bbox.height = int(height*(1/scale))
        bbox.x = int(x*(1/scale))
        bbox.y = int(y*(1/scale))

        return bbox
    
    def create_output_message(self, in_msg, person_list):
        # copy the Detection List message and update the people information
        out_msg = DetectionList()
        out_msg.header = in_msg.header
        out_msg.image = in_msg.image
        out_msg.objects = in_msg.objects

        # add updated people
        # TODO: need to fix the message creation with various components of the message
        for person in person_list:
            #print(f"Person: {person}")
            person_msg = PersonDetection()
            if person.id is not None:
                person_msg.id = str(person.id)
            if person.name is not None:                
                person_msg.name = str(person.name)
            if person.bbox_head is not None:
                person_msg.bbox_head = person.bbox_head
            if person.bbox_person is not None:
                person_msg.bbox_person = person.bbox_person
            # if person.get("face_encoding"):
            #     person_msg.face_encoding = person["face_encoding"]
            # if person.get("position"):
            #     person_msg.position = personp["position"]
            # if person.get("head_position"):
            #     person_msg.head_position = person["head_position"]
            # if person.get("centroid"):
            #     person_msg.centroid = person["centroid"]
            if person.skeleton is not None:
                person_msg.skeleton = person.skeleton2d
            # person_features = person.print_feature_details()
            # if person_features is not None:
            #     person_msg.attributes = person_features

            out_msg.people.append(person_msg) 

        return out_msg

    def print_known_agents(self):

        for agent in self.known_agents:
            agent.print_features()
            # agent.print_feature_details()

    def print_agent_list(self, agent_idx_list):
        agent_ids = ""
        for idx in agent_idx_list:
            agent_ids += str(self.known_agents[idx].id) + ", "
        agent_ids = agent_ids.strip(", ")

    def print_kp(self, kp):
        for i in range(5):
            print(f"KP {i}: {kp[i].x}, {kp[i].y}")

    def count_heads(self, in_msg):
        count = 0
        for person in in_msg.people:
            if person.bbox_head.x > 0 or person.bbox_head.y > 0:
                count += 1

        return count

    def save_image(self, img, perspective):
        if perspective is None:
            perspective = ""
        # save the image for debugging
        if img.size > 0:           
            # save the cropped image to a file
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            filename = f"crop_{perspective}_{timestamp}.jpg"
            # Define the directory
            directory = Path(self.img_path)

            # Ensure the directory exists
            directory.mkdir(parents=True, exist_ok=True)

            # Write the image            
            cv2.imwrite(str(directory / filename), img)   
            print(f"Saved image: - {filename}")  

    def check_img_size(self, img):
        height, width, channels = img.shape
        if height == 0 or width == 0:
            return 0
        else:
            return height*width
    
    def print_stats(self):
        msg_list = []
        msg_list.append("summary")
        print(f"Image with people count: {self.img_with_ppl_cnt}")
        print(f"Head occluded count: {self.occ_head_cnt}")
        print(f"Small head count: {self.small_head_cnt}")
        print(f"Head aspect ratio count: {self.head_aspect_ratio_cnt}")        
        print(f"New agent count: {self.new_agent_cnt}")
        print(f"Update agent count: {self.update_agent_cnt}")
        # print(f"Matched tracking id count: {self.match_track_cnt}")
        # print(f"Matched face count: {self.match_face_cnt}")
        # print(f"Matched head count: {self.match_head_cnt}")  
        # msg_list.append(f"Image with people count: {self.img_with_ppl_cnt}") 
        # msg_list.append(f"Head occluded count: {self.occ_head_cnt}")
        # msg_list.append(f"Small head count: {self.small_head_cnt}")
        # msg_list.append(f"Head aspect ratio count: {self.head_aspect_ratio_cnt}")
        # msg_list.append(f"New agent count: {self.new_agent_cnt}")
        # msg_list.append(f"Update agent count: {self.update_agent_cnt}")
        # self.stats_match_publisher.publish(msg_list)

