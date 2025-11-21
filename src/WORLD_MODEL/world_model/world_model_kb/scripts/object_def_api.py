import psycopg2
from psycopg2 import pool
import json
from datetime import datetime

class ObjectDefApi:

    def __init__(self):
        pass

    def __del__(self):
        pass

    '''
        returns the list of all object_def as a json format string
    '''
    def get_object_def_list(self, pool):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()
            
            cur.execute("select json_agg(json_build_object('object_def_id', od.object_def_id, 'description', od.description, 'category_id', od.category_id, \
                        'category', oc.category, 'sub_category_id', osc.sub_category_id, 'sub_category', osc.sub_category, 'pc_path', od.pc_path, \
                        'image_path', od.image_path, 'decay_weight', od.decay_weight)) \
                        from object_def od \
                        inner join object_category oc on od.category_id = oc.category_id \
                        left outer join object_sub_category osc on od.sub_category_id = osc.sub_category_id")

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
        returns the object definition as a json formatted string
    '''
    def get_object_def(self, pool, object_def_id):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()

            data={}
            data.update({'object_def_id': object_def_id})
            
            cur.execute("select json_agg(json_build_object('object_def_id', od.object_def_id, 'description', od.description, 'category_id', od.category_id, \
                        'category', oc.category, 'sub_category_id', osc.sub_category_id, 'sub_category', osc.sub_category, 'pc_path', od.pc_path, \
                        'image_path', od.image_path, 'decay_weight', od.decay_weight)) \
                        from object_def od \
                        inner join object_category oc on od.category_id = oc.category_id \
                        left outer join object_sub_category osc on od.sub_category_id = osc.sub_category_id \
                        where od.object_def_id = %(object_def_id)s", data)
    
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
        This method inserts a new object_def record
    '''
    def insert_object_def(self, pool, json_):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()

            # validate params
            data = json.loads(json_)[0]
    
            q = "insert into object_def (category_id, sub_category_id, description, pc_path, image_path, decay_weight) " \
                "values (%(category_id)s, %(sub_category_id)s, %(description)s, %(pc_path)s, %(image_path)s, %(decay_weight)s)"      
            
            #add the new record
            cur.execute(q, data)
            
            conn.commit()

        except (Exception, psycopg2.DatabaseError) as error:
            print(error)
        finally:
            if conn:
                pool.putconn(conn)
            if cur is not None:
                cur.close()
                #print('Database cursor closed.')
