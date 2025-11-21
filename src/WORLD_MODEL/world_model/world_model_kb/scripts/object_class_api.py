import numpy as np
import psycopg2
from psycopg2 import pool
import json
from datetime import datetime

class ObjectClassApi:

    def __init__(self):
        pass

    def __del__(self):
        pass

    '''
        This method gets the list of all objects classes (including object attributes) in the database
        Parameters: 
            pool:       database connection pool object, contains database connections
        Return:
            List of all object classes (including object attributes) in the database as a json format string
    '''
    def get_object_class_list(self, pool):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()
            
            cur.execute("select json_agg(json_build_object('object_class_id', oc.object_class_id, 'description', oc.description, 'pc_path', oc.pc_path, \
                        'image_path', oc.image_path, 'decay_weight', oc.decay_weight, \
						'object_attribute', (select json_agg(json_build_object('attribute_type_id', oa.attribute_type_id, 'object_attribute_type', at.attribute_type, \
                        'attribute_value', oa.attribute_value)) \
                        from object_attribute oa inner join attribute_type at on oa.attribute_type_id = at.attribute_type_id \
                        where oa.object_class_id = oc.object_class_id),	\
						'object_class_label', (select json_agg(json_build_object('object_label_id', ocl.object_label_id, 'object_label', ol.object_label)) \
                        from object_class_label ocl inner join object_label ol on ocl.object_label_id = ol.object_label_id \
                        where ocl.object_class_id = oc.object_class_id))) \
                        from object_class oc")

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
        This method gets the object class (including object attributes) with the specific object_class_id and returns to the calling function
        Parameters: 
            pool:       database connection pool object, contains database connections
            object_class_id:  the object class identifier that is the subject of the query
        Return:
            The details of the object class (including object attributes) as a json format string
    '''
    def get_object_class_by_id(self, pool, object_class_id):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()

            data={}
            data.update({'object_class_id': object_class_id})
            
            cur.execute("select json_agg(json_build_object('object_class_id', oc.object_class_id, 'description', oc.description, 'pc_path', oc.pc_path, \
                        'image_path', oc.image_path, 'decay_weight', oc.decay_weight, \
						'object_attribute', (select json_agg(json_build_object('attribute_type_id', oa.attribute_type_id, 'object_attribute_type', at.attribute_type, \
                        'attribute_value', oa.attribute_value)) \
                        from object_attribute oa inner join attribute_type at on oa.attribute_type_id = at.attribute_type_id \
                        where oa.object_class_id = oc.object_class_id),	\
						'object_class_label', (select json_agg(json_build_object('object_label_id', ocl.object_label_id, 'object_label', ol.object_label)) \
                        from object_class_label ocl inner join object_label ol on ocl.object_label_id = ol.object_label_id \
                        where ocl.object_class_id = oc.object_class_id))) \
                        from object_class oc \
                        where oc.object_class_id = %(object_class_id)s", data)
    
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
        This method gets the object class (including object attributes) using the description and returns to the calling function
        Parameters: 
            pool:           database connection pool object, contains database connections
            object_desc:    the object class identifier that is the subject of the query
        Return:
            The details of the object class (including object attributes) as a json format string
    '''
    def get_object_class_by_desc(self, pool, object_desc):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()

            data={}
            data.update({'description': object_desc})
            
            cur.execute("select json_agg(json_build_object('object_class_id', oc.object_class_id, 'description', oc.description, 'pc_path', oc.pc_path, \
                        'image_path', oc.image_path, 'decay_weight', oc.decay_weight, \
						'object_attribute', (select json_agg(json_build_object('attribute_type_id', oa.attribute_type_id, 'object_attribute_type', at.attribute_type, \
                        'attribute_value', oa.attribute_value)) \
                        from object_attribute oa inner join attribute_type at on oa.attribute_type_id = at.attribute_type_id \
                        where oa.object_class_id = oc.object_class_id),	\
						'object_class_label', (select json_agg(json_build_object('object_label_id', ocl.object_label_id, 'object_label', ol.object_label)) \
                        from object_class_label ocl inner join object_label ol on ocl.object_label_id = ol.object_label_id \
                        where ocl.object_class_id = oc.object_class_id))) \
                        from object_class oc \
                        where oc.description = %(description)s", data)
    
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
        This method gets the object class (including object attributes) using the object label and returns to the calling function
        Parameters: 
            pool:           database connection pool object, contains database connections
            object_label:   tuple of object labels, pass in as tuple e.g. ('can','coke')
        Return:
            The details of the object class (including object attributes) as a json format string
    '''
    def get_object_class_by_label(self, pool, object_labels):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()

            cur.execute("select json_agg(to_json(obj)) from \
                (select json_build_object('object_class_id', oc.object_class_id, 'description', oc.description, \
                'pc_path', oc.pc_path, 'image_path', oc.image_path, 'decay_weight', oc.decay_weight, 'object_attribute', \
                (select json_agg(json_build_object('attribute_type_id', oa.attribute_type_id, 'object_attribute_type', \
                at.attribute_type, 'attribute_value', oa.attribute_value)) \
                from object_attribute oa inner join attribute_type at on oa.attribute_type_id = at.attribute_type_id \
                where oa.object_class_id = oc.object_class_id), 'object_class_label', \
                (select json_agg(json_build_object('object_label_id', ocl.object_label_id, 'object_label', ol.object_label)) \
                from object_class_label ocl inner join object_label ol on ocl.object_label_id = ol.object_label_id \
                where ocl.object_class_id = oc.object_class_id)) \
                from object_class_label ocl inner join object_class oc on ocl.object_class_id = oc.object_class_id \
                inner join object_label ol on ocl.object_label_id = ol.object_label_id \
                where ol.object_label in %s \
                group by oc.object_class_id \
                having count(distinct ocl.object_label_id) = %s) as obj", (object_labels, len(object_labels),))
    
            result = cur.fetchone()

            if result[0] is not None:
                str_result = json.dumps(result)
                str_result = str_result.replace('[{"json_build_object":', '').replace('}]}}','}]}')
                json_result = json.loads(str_result[:-1])
                return json_result
            else: 
                return None

        except (Exception, psycopg2.DatabaseError) as error:
            print(error)
        finally:
            if conn:
                pool.putconn(conn)            
            if cur is not None:
                cur.close()
                #print('Database cursor closed.')
    


    '''
        This method gets the base object class (including object attributes) using the object label and returns to the calling function
        The base object class is a class that only has exactly one label associated with it.
        Parameters: 
            pool:           database connection pool object, contains database connections
            object_label:  single object labels
        Return:
            The details of the object class (including object attributes) as a json format string
    '''
    def get_base_object_class_by_label(self, pool, object_label):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()

            cur.execute("select json_build_object('object_class_id', oc.object_class_id, 'description', oc.description, \
                'pc_path', oc.pc_path, 'image_path', oc.image_path, 'decay_weight', oc.decay_weight, 'object_attribute', \
                (select json_agg(json_build_object('attribute_type_id', oa.attribute_type_id, 'object_attribute_type', \
                at.attribute_type, 'attribute_value', oa.attribute_value)) \
                from object_attribute oa inner join attribute_type at on oa.attribute_type_id = at.attribute_type_id \
                where oa.object_class_id = oc.object_class_id), 'object_class_label', \
                (select json_build_object('object_label_id', ocl.object_label_id, 'object_label', ol.object_label) \
                from object_class_label ocl inner join object_label ol on ocl.object_label_id = ol.object_label_id \
                where ocl.object_class_id = oc.object_class_id)) \
                from object_class_label ocl inner join object_class oc on ocl.object_class_id = oc.object_class_id \
                inner join object_label ol on ocl.object_label_id = ol.object_label_id \
                where ol.object_label = %s \
                and (select count(object_class_id) from object_class_label ocl1 where ocl1.object_class_id = oc.object_class_id) = 1", (object_label,))
    
            result = cur.fetchone()

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
        This method gets the id of the base object class using the object label and returns to the calling function
        The base object class is a class that only has exactly one label associated with it.
        Parameters: 
            pool:          database connection pool object, contains database connections
            object_label:  single object labels
        Return:
            The id of the object class as an integer
    '''
    def get_base_object_class_id_by_label(self, pool, object_label):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()

            cur.execute("select oc.object_class_id \
                from object_class_label ocl inner join object_class oc on ocl.object_class_id = oc.object_class_id \
                inner join object_label ol on ocl.object_label_id = ol.object_label_id \
                where ol.object_label = %s \
                and (select count(object_class_id) from object_class_label ocl1 where ocl1.object_class_id = oc.object_class_id) = 1", (object_label,))
    
            result = cur.fetchone()

            if result is not None:
                return result[0]
            else:
                return None

        except (Exception, psycopg2.DatabaseError) as error:
            print(error)
        finally:
            if conn:
                pool.putconn(conn)            
            if cur is not None:
                cur.close()
                #print('Database cursor closed.')   


    '''
        This method gets the id of the base object class using the object label and returns to the calling function
        The base object class is a class that only has exactly one label associated with it.
        Parameters: 
            pool:          database connection pool object, contains database connections
            object_label:  single object labels
        Return:
            The id of the object class as an integer
    '''
    def get_base_object_class_id_by_name(self, pool, object_label):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()

            cur.execute("select oc.object_class_id \
                from object_class_label ocl inner join object_class oc on ocl.object_class_id = oc.object_class_id \
                inner join object_label ol on ocl.object_label_id = ol.object_label_id \
                where ol.object_label = %s \
                and (select count(object_class_id) from object_class_label ocl1 where ocl1.object_class_id = oc.object_class_id) = 1", (object_label,))
    
            result = cur.fetchone()

            if result is not None:
                return result[0]
            else:
                return None

        except (Exception, psycopg2.DatabaseError) as error:
            print(error)
        finally:
            if conn:
                pool.putconn(conn)            
            if cur is not None:
                cur.close()
                #print('Database cursor closed.')   

    '''
        This method gets the id of the object class using the object name and returns to the calling function
        Parameters: 
            pool:          database connection pool object, contains database connections
            object_name:   the name of the object
        Return:
            The id of the object class as an integer
    '''
    def get_object_class_id_by_name(self, pool, object_name):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()

            cur.execute(f"select object_class_id from object_class where name = '{object_name}'")
    
            result = cur.fetchone()

            if result is not None:
                return result[0]
            else:
                return None

        except (Exception, psycopg2.DatabaseError) as error:
            print(error)
        finally:
            if conn:
                pool.putconn(conn)            
            if cur is not None:
                cur.close()
                #print('Database cursor closed.')   


    '''
        This method gets a list of object class ids that contain the object label and returns to the calling function
        Parameters: 
            pool:           database connection pool object, contains database connections
            object_label:   tuple of object labels, pass in as tuple e.g. ('can','coke')
        Return:
            The ids of the object classes as a json format string
    '''
    def get_object_class_id_by_label(self, pool, object_labels):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()

            cur.execute("select json_agg(obj) from \
                (select json_build_object('object_class_id', oc.object_class_id) \
                from object_class_label ocl inner join object_class oc on ocl.object_class_id = oc.object_class_id \
                inner join object_label ol on ocl.object_label_id = ol.object_label_id \
                where ol.object_label in %s \
                group by oc.object_class_id \
                having count(distinct ocl.object_label_id) = %s) as obj", (object_labels, len(object_labels),))
    
            result = cur.fetchone()

            if result[0] is not None:
                str_result = json.dumps(result)
                str_result = str_result.replace('{"json_build_object":', '').replace('}}','}')
                json_result = json.loads(str_result[1:-1])
                return json_result
            else: 
                return None

        except (Exception, psycopg2.DatabaseError) as error:
            print(error)
        finally:
            if conn:
                pool.putconn(conn)            
            if cur is not None:
                cur.close()
                #print('Database cursor closed.')

    def insert_object_class(self, pool, json_):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()

            # validate params
            data = json.loads(json_)[0]

            # remove the object class label data (if exists)
            obj_labels = data.pop("object_class_label", None)
            
            q = "insert into object_class (description, pc_path, image_path, decay_weight)  \
                values ( %(description)s, %(pc_path)s, %(image_path)s, %(decay_weight)s) \
                returning object_class_id"   

            #add the new record
            cur.execute(q, data)
            object_class_id = cur.fetchone()[0]

            # add the object_class_labels records
            if obj_labels:
                for lbl in obj_labels:
                    lbl.update({'object_class_id': object_class_id})
                    lbl.update({'object_label_id': self.get_object_label_id(pool, lbl['object_label'])})
                    del lbl['object_label']

                    q = "insert into object_class_label (object_class_id, object_label_id) " \
                        "values (%(object_class_id)s, %(object_label_id)s)"
                    cur.execute(q, lbl)
            
            conn.commit()

            return object_class_id

        except (Exception, psycopg2.DatabaseError) as error:
            print("insert_object_class error: " + str(error))
            return -1 
        finally:
            if conn:
                pool.putconn(conn)
            if cur is not None:
                cur.close()
                #print('Database cursor closed.')


    '''
    This method gets the object_label_id for the object label returns to the calling function
    Parameters: 
        pool:       database connection pool object, contains database connections
        object_lbl: the description/label to get the id
    Return:
        The object_labl_id
    '''
    def get_object_label_id(self, pool, object_lbl):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()
            
            #error here
            cur.execute("SELECT object_label_id \
                        from object_label where object_label = %s", (object_lbl,))

            result = cur.fetchall()

            return result[0]

        except (Exception, psycopg2.DatabaseError) as error:
            print(error)
        finally:
            if conn:
                pool.putconn(conn)
            if cur is not None:
                cur.close()


    '''
        This method inserts a new object label,
        if the label already exists it will not insert again

        Parameters: 
            pool:       conection pool object
            new_label:  the string containing the new label
        return: 
            if the insert was successful 
                return the object_label_id
            else if the object_label already exists
                return None
            else if the insert failed
                return -1        
    '''
    def insert_object_label(self, pool, new_label):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            conn.autocommit = False
            cur = conn.cursor()        

            q = "insert into object_label (object_label) \
                values (%s) on conflict (object_label) do nothing \
                returning object_label_id"
            
            #add the new record
            cur.execute(q, (new_label,))
            object_lbl_id = cur.fetchone()
            
            conn.commit()

            return object_lbl_id

        except (Exception, psycopg2.DatabaseError) as error:
            print("insert_object_label error: " + str(error))
            return -1
        finally:
            if conn:
                pool.putconn(conn)
            if cur is not None:
                cur.close()
                #print('Database cursor closed.')


    '''
        This method inserts the specific object sub class feature. 
        An object sub class may have multiple features, e.g. 8 encodings that represent the different views 
        of the object

        Parameters: 
            pool:                   conection pool object
            object_sub_class_id:    the id of the object sub class
            feature:                each feature contains an encoding
    '''
    def insert_object_sub_class_feature(self, pool, object_sub_class_id, feature):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()
            
            if feature.encoding is not None:
                # convert the encoding to a list
                encoding_list = "[" + ",".join(map(str, feature.encoding.tolist())) + "]"   # need to convert to list to store values in database             

                qry = f"insert into object_sub_class_feature (object_sub_class_id, encoding) \
                        values ({object_sub_class_id}, ARRAY{encoding_list}::double precision[])"
                
                cur.execute(qry)
            
                conn.commit()        

        except (Exception, psycopg2.DatabaseError) as error:
            print("insert_object_class_features error: {}: {}".format(type(error), error))
            return -1
        finally:
            if conn:
                pool.putconn(conn)
            if cur is not None:
                cur.close()      


    '''
    This method returns all features for the specified object_sub_class

    Parameters
    object_class_id:       the identifier of the object_class 

    Return:
    A list of object_sub_class features for the relevant object_class
    '''
    def get_object_sub_class_features(self, pool, object_class_id):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()

            cur.execute(f"select osc.object_sub_class_id, oscf.encoding \
                        from object_sub_class osc inner join object_sub_class_features oscf on osc.object_sub_class_id = oscf.object_sub_class_id \
                        where osc.object_class_id = {object_class_id}")

            rows = cur.fetchall()

            feature_list = []
            for row in rows:     
                feature_list.append((row[0], np.array(row[1])))   # need to convert the encoding back to numpy array and add to the list

            return feature_list

        except (Exception, psycopg2.DatabaseError) as error:            
            print("get_object_sub_class_features error: {}: {}".format(type(error), error))
        finally:
            if conn:
                pool.putconn(conn)            
            if cur is not None:
                cur.close()                          
          
