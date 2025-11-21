#!/usr/bin/env python3
# version 1.3
# written by Adam Golding

import psycopg2
from psycopg2 import pool
import rospy
from geometry_msgs.msg import Point
from datetime import datetime
import numpy as np
from person import Person
from feature import Feature


'''
The AgentAPI contains methods used to maintain Agent related data in the database

List of Methods:
- insert_agent(self, pool, agent_name, agent_type)
- update_agent_name(self, pool, agent_id, agent_name)
'''
class AgentApi:
    '''
        This method inserts initial agent data into the database
        Parameters: 
            pool:               database connection pool object, contains database connections
            agent_name:         the name of the agent
            agent_type:         'P' = person, 'R' = robot
            encoding_array:     the numpy array of the facial encoding

        Return:
            primary_id:     the agent id
    '''
    # def insert_agent(self, pool, agent_name, agent_type, last_seen_loc, img_path, features):
    def insert_agent(self, pool, agent_type, agent):
        try:
            conn = pool.getconn()
            cur = conn.cursor()

            agent_created = datetime.now()

            query = "insert into agent (agent_name, agent_type, last_seen_loc_x, last_seen_loc_y, last_seen_loc_z, created, last_seen) \
                    values (%s, %s, %s, %s, %s, %s, %s) \
                    returning agent_id"
        
            cur.execute(query, (agent.name, agent_type, agent.position.x, agent.position.y, agent.position.z, agent_created, agent_created,))
            agent_id = cur.fetchone()[0]
            conn.commit()
            # add the features data
            self.insert_agent_features(pool, agent_id, agent.features)

            return agent_id

        except (Exception, psycopg2.DatabaseError) as error:
            rospy.loginfo("insert_agent error: {}: {}".format(type(error), error))
            return -1
        finally:
            if conn:
                pool.putconn(conn)
            if cur is not None:
                cur.close()

    '''
        This method inserts new agent features based on the entire list of features attached to the agent object.

        Parameters: 
            pool:               conection pool object
            agent_id:           the id of the agent
            features:           list of features, each feature contains a perspective, feature type and encoding
    '''
    def insert_agent_features(self, pool, agent_id, features):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()
            
            for feature in features:
                feat_type_id = self.get_agent_feature_type_id(pool, feature.feature_type)              

                if feat_type_id is not None and feature.encoding is not None:
                    # if the feature has been encoded add the agent_feature otherwise ignore it 
                    encoding_list = "[" + ",".join(map(str, feature.encoding.tolist())) + "]"   # need to convert to list to store values in database                  
                    if feature.colour is not None:
                        colour_list = feature.colour
                        # colour_list = "[" + ",".join(map(str, feature.colour.tolist())) + "]"   # need to convert to list to store values in database
                    else:
                        colour_list = []
                    qry = f"insert into agent_feature (agent_id, agent_feature_type_id, perspective, encoding, colour) \
                            values ({agent_id}, {feat_type_id}, '{feature.perspective}', ARRAY{encoding_list}::double precision[], ARRAY{colour_list}::double precision[])"   
                                                                        
                    cur.execute(qry)
                
                    conn.commit()

        except (Exception, psycopg2.DatabaseError) as error:
            rospy.loginfo("insert_agent_features error: {}: {}".format(type(error), error))
            return -1
        finally:
            if conn:
                pool.putconn(conn)
            if cur is not None:
                cur.close()

    '''
        This method inserts the specific agent feature.

        Parameters: 
            pool:               conection pool object
            agent_id:           the id of the agent
            feature:            each feature contains a perspective, feature type and encoding
    '''
    def insert_agent_feature(self, pool, agent_id, feature):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()
            
            feat_type_id = self.get_agent_feature_type_id(pool, feature.feature_type)  

            if feat_type_id is not None and feature.encoding is not None:
                # update the agent_feature 
                encoding_list = "[" + ",".join(map(str, feature.encoding.tolist())) + "]"   # need to convert to list to store values in database             
                if feature.colour is not None:
                    colour_list = feature.colour
                    # colour_list = "[" + ",".join(map(str, feature.colour.tolist())) + "]"   # need to convert to list to store values in database
                else:
                    colour_list = []
                qry = f"insert into agent_feature (agent_id, agent_feature_type_id, perspective, encoding, colour) \
                        values ({agent_id}, {feat_type_id}, '{feature.perspective}', ARRAY{encoding_list}::double precision[], ARRAY{colour_list}::double precision[])"
                
                cur.execute(qry)
            
                conn.commit()        

        except (Exception, psycopg2.DatabaseError) as error:
            rospy.loginfo("insert_agent_feature error: {}: {}".format(type(error), error))
            return -1
        finally:
            if conn:
                pool.putconn(conn)
            if cur is not None:
                cur.close()

    '''
        This method updates the agent_name and the last_seen timestamp
        Parameters: 
            pool:       conection pool object
            agent_id:   the id of the agent
            agent_name: the new name of the agent
    '''
    def update_agent_name(self, pool, agent_id, agent_name):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()
            
            # update the agent_name           
            cur.execute("update agent set agent_name = %s, last_seen = %s \
                    where agent.agent_id = %s", (agent_name, datetime.now(), agent_id))  
            conn.commit()

        except (Exception, psycopg2.DatabaseError) as error:
            rospy.loginfo("update_agent_name error: {}: {}".format(type(error), error))
            return -1
        finally:
            if conn:
                pool.putconn(conn)
            if cur is not None:
                cur.close()

    '''
        This method updates the last_seen timestamp
        Parameters: 
            pool:       conection pool object
            agent_id:   the id of the agent
            location:   the location as a point (x, y, z)
    '''
    def update_agent_last_seen(self, pool, agent_id, location):
        try: 
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()
            
            # update the agent_name           
            cur.execute("update agent set last_seen = %s \
                        ,last_seen_loc_x = %s \
                        ,last_seen_loc_y = %s \
                        ,last_seen_loc_z = %s \
                    where agent.agent_id = %s", (datetime.now(), location.x, location.y, location.z, agent_id, ))  
            conn.commit()

        except (Exception, psycopg2.DatabaseError) as error:
            rospy.loginfo("update_agent_last_seen error: {}: {}".format(type(error), error))
            return -1
        finally:
            if conn:
                pool.putconn(conn)
            if cur is not None:
                cur.close()

    '''
        This method updates the last_seen timestamp
        Parameters: 
            pool:       conection pool object
            agent_id:   the id of the agent
            img_file:   the path and filename of the image taken for the facial encoding data
    '''
    def update_agent_image(self, pool, agent_id, img_file):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()
            
            # update the agent_name           
            cur.execute("update facial_encoding set image_path = %s \
                    where facial_encoding.agent_id = %s", (img_file, agent_id))  
            conn.commit()

        except (Exception, psycopg2.DatabaseError) as error:
            rospy.loginfo("update_agent_image error: {}: {}".format(type(error), error))
            return -1
        finally:
            if conn:
                pool.putconn(conn)
            if cur is not None:
                cur.close()

    '''
        This method gets the id of the agent using the agent name and returns the
        agent_id to the calling function.
        Parameters: 
            pool:          database connection pool object, contains database connections
            agent_name:    the name of the agent you wish to find an id for
        Return:
            The id of the agent as an integer
    '''
    def get_agent_id_by_name(self, pool, agent_name):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()

            cur.execute(f"SELECT agent_id from agent where agent_name = '{agent_name}';")

            result = cur.fetchone()[0]

            return result

        except (Exception, psycopg2.DatabaseError) as error:            
            rospy.loginfo("get_agent_id_by_name error: {}: {}".format(type(error), error))
        finally:
            if conn:
                pool.putconn(conn)            
            if cur is not None:
                cur.close()

    '''
        This method gets the name of the agent using the agent id and returns the
        name to the calling function.

        Parameters: 
            pool:          database connection pool object, contains database connections
            agent_id:      the id of the agent you wish to find the name of
        Return:
            The name of the agent
    '''
    def get_agent_name(self, pool, agent_id):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()

            cur.execute(f"SELECT agent_name from agent where agent_id = '{agent_id}';")

            result = cur.fetchone()[0]

            return result

        except (Exception, psycopg2.DatabaseError) as error:            
            rospy.loginfo("get_agent_name error: {}: {}".format(type(error), error))
        finally:
            if conn:
                pool.putconn(conn)            
            if cur is not None:
                cur.close()

    '''
        This method gets the list of agents stored in the world model database.
        This method is called on initialisation of the person recognition node.
        The last seen position is not relevant as it will not be accurate 
        An agent may have more than 1 facial encoding to improve the accuracy
        Parameters: 
            pool:          database connection pool object, contains database connections
        Return:
            The list of agents with agent_id, agent_name, facial_encoding as a list of Person objects
    '''
    def load_agent_list(self, pool):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()

            agent_list = []

            cur.execute("select agent_id, agent_name from agent")
            
            rows = cur.fetchall()       # get the list of agents in the database

            for row in rows:
                position = Point()
                position.x = 0
                position.y = 0
                position.z = 0

                # get the features for the agent
                features = self.get_agent_features(pool, row[0])

                # parameters = id, bbox_person, bbox_head, bbox_face, skeleton, position, features, name
                agent = Person(row[0], None, None, None, None, position, features, row[1])
                agent_list.append(agent)

            return agent_list
            
        except (Exception, psycopg2.DatabaseError) as error:    
            rospy.loginfo("get_agent_list error: {}: {}".format(type(error), error))        
        finally:
            if conn:
                pool.putconn(conn)            
            if cur is not None:
                cur.close()

    '''
        This method gets the list of agents stored in the database. 
        An agent may have more than 1 facial encoding to improve the accuracy
        Parameters: 
            pool:          database connection pool object, contains database connections
        Return:
            The list of agents with agent_id, agent_name, facial_encoding as a list of Person objects
    '''
    def get_agent_list(self, pool):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()

            agent_list = []

            cur.execute("select agent_id, agent_name, last_seen_loc_x, last_seen_loc_y, last_seen_loc_z from agent")
            
            rows = cur.fetchall()       # get the list of agents in the database

            for row in rows:
                position = Point()
                position.x = row[2]
                position.y = row[3]
                position.z = row[4]

                # get the features for the agent
                features = self.get_agent_features(pool, row[0])

                # parameters = id, bbox_person, bbox_head, bbox_face, skeleton, position, features, name
                agent = Person(row[0], None, None, None, None, position, features, row[1])
                agent_list.append(agent)

            return agent_list
            
        except (Exception, psycopg2.DatabaseError) as error:    
            rospy.loginfo("get_agent_list error: {}: {}".format(type(error), error))        
        finally:
            if conn:
                pool.putconn(conn)            
            if cur is not None:
                cur.close()

    '''
    This method returns all features for the specified agent

    Parameters
    agent_id:       the identifier of the agent 

    Return:
    A list of agent features using the Feature class
    '''
    def get_agent_features(self, pool, agent_id):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()

            cur.execute(f"select af.perspective, aft.agent_feature_type, af.encoding, af.colour \
                        from agent_feature af inner join agent_feature_type aft on af.agent_feature_type_id = aft.agent_feature_type_id \
                        where agent_id = {agent_id}")

            rows = cur.fetchall()

            feature_list = []
            for row in rows:
                feature = Feature(row[0], row[1], np.array(row[2]), np.array(row[3]))     # need to convert the encoding back to numpy array
        
                feature_list.append(feature)

            return feature_list

        except (Exception, psycopg2.DatabaseError) as error:            
            rospy.loginfo("get_agent_features error: {}: {}".format(type(error), error))
        finally:
            if conn:
                pool.putconn(conn)            
            if cur is not None:
                cur.close()

    '''
        This method checks if an agent feature exists. 
        The comparison is based on the agent_id, agent_feature_type_id, perspective

        Parameters: 
            pool:               database connection pool object, contains database connections
            agent_id:           the id of the agent you wish to find the feature of
            feature_type_id:    the id of the feature type 
            perspective:        the perspective of the feature
        Return:
            True:   if the feature exists
            False:  if the feature does not exist
    '''
    def feature_exists(self, pool, agent_id, feature_type_id, perspective):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()

            cur.execute(f"SELECT agent_id from agent_feature where agent_id = {agent_id} and agent_feature_type_id = {feature_type_id} and perspective = '{perspective}';")

            result = cur.fetchone()

            if result is not None:
                return True
            else:
                return False

        except (Exception, psycopg2.DatabaseError) as error:            
            rospy.loginfo("feature_exists error: {}: {}".format(type(error), error))
        finally:
            if conn:
                pool.putconn(conn)            
            if cur is not None:
                cur.close()

    '''
        This method gets the id of the agent_feature_type using the agent_feature_type string and returns the id.

        Parameters: 
            pool:                       database connection pool object, contains database connections
            agent_feature_type:         string representing the agent feature type
        Return:
            The id of the agent feature type if it exists or None if it does not exist
    '''
    def get_agent_feature_type_id(self, pool, feature_type):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()

            cur.execute(f"SELECT agent_feature_type_id from agent_feature_type where agent_feature_type = '{feature_type}';")

            result = cur.fetchone()
            
            if cur.rowcount == 0:
                result = None 
            else:
                result = result[0]

            return result

        except (Exception, psycopg2.DatabaseError) as error:            
            rospy.loginfo("get_agent_attribute_id error: {}: {}".format(type(error), error))
        finally:
            if conn:
                pool.putconn(conn)            
            if cur is not None:
                cur.close()

    '''
        This method updates the agent features if they exist or inserts new features if they don't.
        f they updates the agent_features. Once the feature has been updated it t exist.

        Parameters: 
            pool:               conection pool object
            agent_id:           the id of the agent
            feature:            the list of features, each feature contains a perspective, feature type and encoding
    '''
    def update_agent_features(self, pool, agent_id, features):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()
            
            for feature in features:
                # feat_type_id = self.get_agent_feature_type_id(pool, agent_id, feature['feature_type'])  
                # feat_exists = self.feature_exists(pool, agent_id, feat_type_id, feature['perspective'])
                feat_type_id = self.get_agent_feature_type_id(pool, agent_id, feature.feature_type)  
                feat_exists = self.feature_exists(pool, agent_id, feat_type_id, feature.perspective)                

                if feat_type_id is not None:
                    encoding_list = "[" + ",".join(map(str, feature.encoding.tolist())) + "]"   # need to convert to list to store values in database
                    if feature.colour is not None:
                        colour_list = feature.colour
                        # colour_list = "[" + ",".join(map(str, feature.colour.tolist())) + "]"   # need to convert to list to store values in database
                    else:
                        colour_list = []
                    if feat_exists:
                        # update the agent_feature                         
                        qry = f"update agent_feature set encoding = ARRAY{encoding_list}::double precision[], colour = ARRAY{colour_list}::double precision[], last_updated = '{datetime.now()}' \
                            where agent_id = {agent_id} and agent_feature_type_id = {feat_type_id} and perspective = '{feature.perspective}'"      
                    else:
                        # add the agent_feature           
                        qry = f"insert in to agent_feature (agent_id, agent_feature_type_id, perspective, encoding, colour) \
                                values ({agent_id}, {feat_type_id}, '{feature.perspective}', ARRAY{encoding_list}::double precision[], ARRAY{colour_list}::double precision[])"                                                                         
                    
                    cur.execute(qry)
                
                    conn.commit()

            self.update_agent_last_seen(pool, agent_id)

        except (Exception, psycopg2.DatabaseError) as error:
            rospy.loginfo("update_agent_features error: {}: {}".format(type(error), error))
            return -1
        finally:
            if conn:
                pool.putconn(conn)
            if cur is not None:
                cur.close()

    '''
        This method updates the specific agent feature.

        Parameters: 
            pool:               conection pool object
            agent_id:           the id of the agent
            feature:            each feature contains a perspective, feature type and encoding
    '''
    def update_agent_feature(self, pool, agent_id, feature):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()
            
            feat_type_id = self.get_agent_feature_type_id(pool, feature.feature_type)  

            if feat_type_id is not None and feature.encoding is not None:               
                # update the agent_feature 
                encoding_list = "[" + ",".join(map(str, feature.encoding.tolist())) + "]"   # need to convert to list to store values in database
                if feature.colour is not None:
                    colour_list = feature.colour
                    # colour_list = "[" + ",".join(map(str, feature.colour.tolist())) + "]"   # need to convert to list to store values in database
                else:
                    colour_list = []
                qry = f"update agent_feature set encoding = ARRAY{encoding_list}::double precision[], colour = ARRAY{colour_list}::double precision[], last_updated = '{datetime.now()}' \
                    where agent_id = {agent_id} and agent_feature_type_id = {feat_type_id} and perspective = '{feature.perspective}'"           
                
                cur.execute(qry)
            
                conn.commit()        

        except (Exception, psycopg2.DatabaseError) as error:
            rospy.loginfo("update_agent_feature error: {}: {}".format(type(error), error))
            return -1
        finally:
            if conn:
                pool.putconn(conn)
            if cur is not None:
                cur.close()

    '''
        This method gets the id of the agent_attribute_type using the agent_attribute_type string and returns the id.

        Parameters: 
            pool:                       database connection pool object, contains database connections
            agent_attribute_type:       string representing the agent attribute type
        Return:
            The id of the agent feature type if it exists or None if it does not exist
    '''
    def get_agent_attribute_type_id(self, pool, attribute_type):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()

            cur.execute(f"SELECT agent_attribute_type_id from agent_attribute_type where agent_attribute_type = '{attribute_type}';")

            result = cur.fetchone()
            
            if cur.rowcount == 0:
                result = None 
            else:
                result = result[0]

            return result

        except (Exception, psycopg2.DatabaseError) as error:            
            rospy.loginfo("get_agent_attribute_type_id error: {}: {}".format(type(error), error))
        finally:
            if conn:
                pool.putconn(conn)            
            if cur is not None:
                cur.close()

    '''
        This method inserts the specific agent attributes.

        Parameters: 
            pool:                   conection pool object
            agent_id:               the id of the agent
            attribute type:         each feature contains a perspective, feature type and encoding
            attribute value:        the value associated with the attribute
    '''
    def insert_agent_attribute(self, pool, agent_id, attribute_type, attribute_value):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()
            
            attr_type_id = self.get_agent_attribute_type_id(pool, attribute_type)  

            if attr_type_id is not None and attribute_value is not None:
                # add the agent_attribute
                qry = f"insert into agent_attribute (agent_id, agent_attribute_type_id, agent_attribute_value) \
                        values ({agent_id}, {attr_type_id}, '{attribute_value}')"
                
                cur.execute(qry)
            
                conn.commit()        

        except (Exception, psycopg2.DatabaseError) as error:
            rospy.loginfo("insert_agent_attribute error: {}: {}".format(type(error), error))
            return -1
        finally:
            if conn:
                pool.putconn(conn)
            if cur is not None:
                cur.close()

    '''
        This method checks if an agent attribute exists. 
        The comparison is based on the agent_id, agent_attribute_type_id

        Parameters: 
            pool:               database connection pool object, contains database connections
            agent_id:           the id of the agent you wish to find the feature of
            attribute_type_id:  the id of the attribute type           
        Return:
            True:   if the feature exists
            False:  if the feature does not exist
    '''
    def attribute_exists(self, pool, agent_id, attribute_type_id):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()

            cur.execute(f"SELECT agent_id from agent_attribute where agent_id = {agent_id} and agent_attribute_type_id = {attribute_type_id};")

            result = cur.fetchone()

            if result is not None:
                return True
            else:
                return False

        except (Exception, psycopg2.DatabaseError) as error:            
            rospy.loginfo("attribute_exists error: {}: {}".format(type(error), error))
        finally:
            if conn:
                pool.putconn(conn)            
            if cur is not None:
                cur.close()

    '''
        This method updates the agent_attribute. Once the attribute has been updated it will update the 
        agent_last seen value.

        Parameters: 
            pool:               conection pool object
            agent_id:           the id of the agent
            attribute_type_id:  the id of the attribute type
            attribute_value:    the value associated with the attribute
    '''
    def update_agent_attribute(self, pool, agent_id, attribute_type, agent_attribute_value):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()
            
            attr_type_id = self.get_agent_attribute_type_id(pool, attribute_type) 
            agent_attribute_exists = self.attribute_exists(pool, agent_id, attr_type_id)

            if agent_attribute_exists:
                # update the agent_attribute           
                cur.execute("update agent_attribute set agent_attribute_value = %s \
                        where agent_id = %s and agent_attribute_type_id = %s", (agent_attribute_value, agent_id, attr_type_id,))  
            else:
                # add the agent_attribute           
                cur.execute("insert into agent_attribute (agent_id, agent_attribute_type_id, agent_attribute_value) \
                            values (%s, %s, %s)",(agent_id, attr_type_id, agent_attribute_value,))
                
            conn.commit()

            self.update_agent_last_seen(pool, agent_id)

        except (Exception, psycopg2.DatabaseError) as error:
            rospy.loginfo("update_agent_attribute error: {}: {}".format(type(error), error))
            return -1
        finally:
            if conn:
                pool.putconn(conn)
            if cur is not None:
                cur.close()

    '''
        This method gets the spedified attribute of the agent.
        
        Parameters: 
            pool:                   database connection pool object, contains database connections
            agent_id:               the id of the agent you wish to find the attribute of
            agent_attribute_type_id:    
        Return:
            The value associated with the agent attribute type
    '''
    def get_agent_attribute(self, pool, agent_id, attribute_type):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()

            attr_type_id = self.get_agent_attribute_type_id(pool, attribute_type) 

            cur.execute(f"SELECT agent_attribute_value from agent_attribute where agent_id = '{agent_id}' and \
                        agent_attribute_type_id={attr_type_id};")

            result = cur.fetchone()[0]

            return result

        except (Exception, psycopg2.DatabaseError) as error:            
            rospy.loginfo("get_agent_attribute error: {}: {}".format(type(error), error))
        finally:
            if conn:
                pool.putconn(conn)            
            if cur is not None:
                cur.close()

    '''
        This method deletes an agent and related data.
        
        Parameters: 
            pool:                   database connection pool object, contains database connections
            agent_id:               the id of the agent you wish to delete   
        Return:
            No return value   
    '''
    def delete_agent(self, agent_id):
        try:
            conn = pool.getconn()
            cur = conn.cursor()

            cur.execute("delete from agent_object where agent_id = %s", (agent_id,))  
            cur.execute("delete from agent_feature where agent_id = %s", (agent_id,))  
            cur.execute("delete from agent_attribute where agent_id = %s", (agent_id,))  
            cur.execute("delete from agent where agent_id = %s", (agent_id,))  
        
            conn.commit()

        except (Exception, psycopg2.DatabaseError) as error:
            rospy.loginfo("delete_agent error: {}: {}".format(type(error), error))
            return -1
        finally:
            if conn:
                pool.putconn(conn)
            if cur is not None:
                cur.close()


    def get_agent_attribute_types(self, pool):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()
            
            cur.execute("select json_agg(json_build_object( \
                        'agent_attribute_type_id', aat.agent_attribute_type_id, 'agent_attribute_type', aat.agent_attribute_type, 'question', aat.question \
                        from agent_attribute_type aat))")

            result = cur.fetchall()

            return result

        except (Exception, psycopg2.DatabaseError) as error:            
            print(error)
        finally:
            if conn:
                pool.putconn(conn)            
            if cur is not None:
                cur.close()

    def get_agent_attribute_type(self, pool, attribute_type):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()
            
            cur.execute("select json_agg(json_build_object( \
                        'agent_attribute_type_id', aat.agent_attribute_type_id, 'agent_attribute_type', aat.agent_attribute_type, 'question', aat.question \
                        from agent_attribute_type aat)) where aat.agent_attribute_type = %s", (attribute_type,))

            result = cur.fetchall()

            return result

        except (Exception, psycopg2.DatabaseError) as error:            
            print(error)
        finally:
            if conn:
                pool.putconn(conn)            
            if cur is not None:
                cur.close()