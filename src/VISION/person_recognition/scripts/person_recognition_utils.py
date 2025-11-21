#!/usr/bin/env python3

import os
import sys
import numpy as np
from person import Person
from feature import Feature
from face_recognition import face_recognition
module_path = os.environ.get("UNSW_WS")
sys.path.append(module_path + "/WORLD_MODEL/world_model_kb/scripts")
from agent_api import AgentApi
from dbase import DBase

class PRUtils():
    def __init__(self, face_threshold, use_world_model):
        self.face_threshold = face_threshold
        self.use_world_model = use_world_model
        self.agent = AgentApi()
        self.db = DBase(os.path.abspath(self.db_ini))

    def match_face(self, known_agents):
        face_encodings = None
        agent_list = None

        for agent in known_agents:            
            # extract face encodings for known agents
            for feat in agent:
                if feat.feature_type == "face" and feat.encoding is not None:
                    # create 2 lists that correspond to the face encodings and agent ids
                    face_encodings.append(feat.encoding)
                    agent_list.append(agent.id)                 

        # compare face encodings
        for i in range(len(face_encodings)):
            face_distances = face_recognition.face_distance(face_encodings, face_encodings[i])
            if face_distances is not None: 
                if len(face_distances) > 0:    
                    min_idx = np.argmin(face_distances)
                    if face_distances[min_idx] <= self.face_threshold:    # found a match                                          
                        # merge the agents
                        self.merge_features(known_agents[agent_list[i]], known_agents[agent_list[min_idx]])
                        self.prune_agent(known_agents[agent_list[i]])


    def merge_features(self, old_agent, new_agent):
        # check if feature currently exists        
        for new_feat in new_agent.features:
            found_features = False
            if new_feat.encoding is not None:
                for old_feat in old_agent.features:                                          
                    if old_feat.perspective == new_feat.perspective and old_feat.feature_type == new_feat.feature_type:                           
                        old_feat.encoding = new_feat.encoding
                        found_features = True
            
                # if it is a new perspective/feature add it                   
                if found_features == False:
                    feat = Feature(new_feat.perspective, new_feat.feature_type, new_feat.encoding)
                    old_agent.add_feature(feat)
                
                    if self.use_world_model:
                        self.agent.insert_agent_feature(self.db.con_pool, new_agent.id, new_feat)
                else:
                    if self.use_world_model:
                        self.agent.update_agent_feature(self.db.con_pool, new_agent.id, new_feat)

    '''
    This method deletes an agent and all asociated features from the known_agents list
    and the world model

    Parameters:     
    known_agents: list of known agents
    agent_id: id of the agent to be deleted

    '''
    def prune_agent(self, known_agents, agent_id):
        # delete agent from known_agents
        for agent in known_agents:
            if agent.id == agent_id:
                known_agents.remove(agent)
                break

        # delete agent from the world model
        if self.use_world_model:
            self.agent.delete_agent(self.db.con_pool, agent_id)

    def merge_attributes(self):
        pass

    def prune_attributes(self):
        pass


