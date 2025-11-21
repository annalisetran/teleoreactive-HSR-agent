# Version 1.1
# written by Adam Golding

import psycopg2
from psycopg2 import pool
import json
from datetime import datetime

class RobotStateApi:

    def __init__(self):
        pass

    def __del__(self):
        pass

    '''
        This method gets the current robot state and returns to the calling function
        Parameters: 
            pool:       database connection pool object, contains database connections
        Return:
            The current robot state as a json format string
    '''
    def get_robot_state_current(self, pool):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()
            
            cur.execute('SELECT json_agg(robot_state) from robot_state where is_current = 1')

            result = cur.fetchall()

            return result

        except (Exception, psycopg2.DatabaseError) as error:
            print(error)
        finally:
            if conn:
                pool.putconn(conn)
            if cur is not None:
                cur.close()

    '''
        This method gets the id of the current robot state and returns it to the calling function
        Parameters: 
            pool:       database connection pool object, contains database connections
        Return:
            The current robot state id as a integer
    '''   
    def get_robot_state_current_id(self, pool):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()
            
            cur.execute('select robot_state_id from robot_state where is_current = 1')

            result = cur.fetchall()

            return result

        except (Exception, psycopg2.DatabaseError) as error:
            print(error)
        finally:
            if conn:
                pool.putconn(conn)
            if cur is not None:
                cur.close()

    '''
        This method gets the list of all robot_states in the database
        Parameters: 
            pool:       database connection pool object, contains database connections
        Return:
            List of all robot_states in the database (i.e. robot_state history) as a json format string
    '''
    def get_robot_state_history(self, pool):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()
            
            cur.execute('SELECT json_agg(robot_state) from robot_state')
    
            result = cur.fetchall()

            return result

        except (Exception, psycopg2.DatabaseError) as error:
            print(error)
        finally:
            if conn:
                pool.putconn(conn)
            if cur is not None:
                cur.close()
                #print('Database cursor closed.')

    
    '''
        This method inserts a new robot_state record
        1. sets the is_current flag = 0 for the current robot_state record
        2. inserts the new robot_state
        Parameters: 
            pool:       conection pool object
            json_:      json string containing the data to be inserted
        return: 
            if the insert was successful 
                return the robot_state_id
            else if the insert failed
                return -1        
    '''
    def insert_robot_state(self, pool, json_):
        conn = None
        cur = None
        try:       
            # get connection and create 
        
            conn = pool.getconn()
            conn.autocommit = False
            cur = conn.cursor()

            # age the current robot_state prior to adding the new robot_state
            cur.execute('update robot_state set is_current = 0 where is_current = 1')  

            # validate params
            data = json.loads(json_)

            data.update({'created': datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")})
            data.update({'is_current': 1})
            
            qry = f"insert into robot_state (base_pose_x, base_pose_y, base_pose_z, frame_id, base_r, base_p, base_yw,  \
                head_pose_z, head_r, head_p, head_yw, \
                end_effector_x, end_effector_y, end_effector_z, end_effector_r, end_effector_p, end_effector_yw, \
                driveable_state, is_wrist_pressed, gripper_closed, listening_state, speaking_state, battery_status, holding_object_id, \
                created, is_current) \
                        values ({data['base_pose_x']}, {data['base_pose_y']}, {data['base_pose_z']}, {data['frame_id']}, {data['base_r']}, {data['base_p']}, {data['base_yw']},  \
                {data['head_pose_z']}, {data['head_r']}, {data['head_p']}, {data['head_yw']}, \
                {data['end_effector_x']}, {data['end_effector_y']}, {data['end_effector_z']}, {data['end_effector_r']}, {data['end_effector_p']}, {data['end_effector_yw']}, \
                {data['driveable_state']}, {data['is_wrist_pressed']}, {data['gripper_closed']}, {data['listening_state']}, {data['speaking_state']}, {data['battery_status']}, {data['holding_object_id']}, \
                '{data['created']}', {data['is_current']})"

            #add the new robot_state
            cur.execute(qry)
            conn.commit()

            return 0

        except (Exception, psycopg2.DatabaseError) as error:
            if conn:
                conn.rollback()
            print("insert_robot_state error: " + str(error))
            return -1
        finally:
            if conn:
                pool.putconn(conn)
            if cur is not None:
                cur.close()
                #print('Database cursor closed.')
