#!/usr/bin/env python3
# World Model Version 2.8
# written by Adam Golding

import psycopg2
from psycopg2 import pool
from datetime import datetime

class SceneApi:

    def __init__(self):
        pass

    def __del__(self):
        pass

    '''
        This method inserts a new scene record
        1. sets the is_current flag = 0 for the current scene record
        2. inserts the new scene
        Parameters: 
            pool:       conection pool object
        return: 
            if the insert was successful 
                return the scene_id
            else if the insert failed
                return -1        
    '''
    def insert_scene(self, pool, scene_id):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            conn.autocommit = False
            cur = conn.cursor()
          
            #add the new record
            cur.execute(f"insert into scene (scene_id, scene_ts) values ({scene_id}, '{datetime.now()}') returning scene_id")            

            seq_id = cur.fetchone()[0]
            
            conn.commit()

            return scene_id

        except (Exception, psycopg2.DatabaseError) as error:
            print(error)
            return -1
        finally:
            if conn:
                pool.putconn(conn)
            if cur is not None:
                cur.close()


    '''
        This method inserts a new scene object record into a scene
        Parameters: 
            pool:       conection pool object
            json_:      json string containing the data to be inserted
            scene_id:   the scene the object belongs to
        return: 
            if the insert was successful 
                return the scene_object_id
            else if the insert failed
                return -1
    '''
    def insert_scene_object(self, pool, scene_id, object_id):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            conn.autocommit = False
            cur = conn.cursor()

            cur.execute(f"insert into scene_object(scene_id, object_id) values ({scene_id}, {object_id})")
            
            conn.commit()

        except (Exception, psycopg2.DatabaseError) as error:
            print("insert_scene_object error: " + str(error))
            return -1
        finally:
            if conn:
                pool.putconn(conn)
            if cur is not None:
                cur.close()

