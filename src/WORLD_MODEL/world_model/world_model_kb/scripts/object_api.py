# World Model Version 2.8
# written by Adam Golding

import psycopg2
from psycopg2 import pool
import rospy
import json
from datetime import datetime

class ObjectApi:

    def __init__(self):
        pass

    def __del__(self):
        pass



    '''
        This method inserts a new object record and object relationships if they exist
        Parameters: 
            pool:      conection pool object
            data:      json string containing the data to be inserted
            _obj:      data in the object structure that will be inserted
        return: 
            if the insert was successful 
                return the object_id
            else if the insert failed
                return -1
    '''
    # TODO: check on how to remove the 'data' parameter
    def insert_object(self, pool, obj):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()

            # validate params
            #data = json.loads(json_)

            # TODO: Check how to handle the Object Relationships
            # extract the object relationship data (if exists) and remove it from the list
            #obj_rels = data.pop("object_relationship", None)    

            dt_now = datetime.now()  
            loc_confidence = 1.0
            # TODO: Get the region
            region_id = 1          

            # data for the following columns can be added later
            associated_text = None
            mean_colour = None
            texture = None
            shape_id = None
            size_id = None
            img_path = None
            pcd_path = None

            #add the new record
            cur.execute("insert into object(object_class_id, class_confidence, associated_text, frame_id, \
                    bbox_x, bbox_y, bbox_width, bbox_height, bbox_cols, bbox_rows, \
                    loc_x, loc_y, loc_z, loc_confidence, mean_colour, texture, region_id, size_id, shape_id, img_path, pcd_path, \
				    created, last_seen, last_updated, is_current) \
	                values (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s) returning object_id", \
                    (obj.class_id, obj.class_confidence, associated_text, obj.frame_id, \
                    obj.bbox_x, obj.bbox_y, obj.bbox_width, obj.bbox_height, obj.bbox_cols, obj.bbox_rows, \
                    obj.point[0], obj.point[1], obj.point[2], loc_confidence, mean_colour, texture, region_id, \
                    size_id, shape_id, img_path, pcd_path, dt_now, dt_now, dt_now, 1))
            
            primary_id = cur.fetchone()[0]            

            # Uncomment the Object Relationship code when work out how it will work
            # add the object_relationship records            
            # if obj_rels:
            #     for rel in obj_rels:
            #         rel.update({'primary_id': primary_id})

            #         q = "insert into object_relationship (related_id, object_relationship_type_id, primary_id) \
            #             values (%(related_id)s, %(object_relationship_type_id)s, %(primary_id)s)"
            #         cur.execute(q, rel)
            
            conn.commit()

            return primary_id

        except (Exception, psycopg2.DatabaseError) as error:
            print("insert_object error: {} {}".format(type(error), error))
            return -1
        finally:
            if conn:
                pool.putconn(conn)
            if cur is not None:
                cur.close()


    '''
        This method gets the list of all objects and their relationships between objects in the database
        Parameters: 
            pool:       database connection pool object, contains database connections
        Return:
            List of all objects (including object relationships) in the database as a json format string
    '''
    def get_object_list(self, pool):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()
            
            cur.execute("select json_agg(json_build_object( \
                        'object_id', obj.object_id, 'object_class_id', obj.object_class_id, 'class_confidence', obj.class_confidence, \
                        'name', oc.name, 'description', oc.description,  \
                        'associated_text', obj.associated_text, 'frame_id', obj.frame_id, \
                        'bbox_x', obj.bbox_x, 'bbox_y', obj.bbox_y, 'bbox_width', obj.bbox_width, 'bbox_height', obj.bbox_height, 'bbox_cols', obj.bbox_cols, 'bbox_rows', obj.bbox_rows, \
                        'loc_x', obj.loc_x, 'loc_y', obj.loc_y, 'loc_z', obj.loc_z, 'loc_confidence', obj.loc_confidence, \
                        'mean_colour', obj.mean_colour, 'texture', obj.texture, 'region_id', obj.region_id, 'region_label', r.label, \
                        'shape_id', obj.shape_id, 'shape_desc', osh.shape_desc, 'size_id', obj.size_id, 'size_desc', osz.size_desc, \
                        'img_path', obj.img_path, 'pcd_path', obj.pcd_path, \
                        'created', obj.created, 'last_updated', obj.last_updated, 'last_seen', obj.last_seen, 'is_current', obj.is_current)) \
                        from object obj \
                        left outer join object_shape osh on obj.shape_id = osh.shape_id \
                        left outer join object_size osz on obj.size_id = osz.size_id \
                        left outer join object_class oc on obj.object_class_id = oc.object_class_id \
						left outer join region r on obj.region_id = r.region_id")

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
        This method gets the object with the specific object_id and all of their relationships with other objects 
        and returns to the calling function
        Parameters: 
            pool:       database connection pool object, contains database connections
            object_id:  the object identifier that is the subject of the query
        Return:
            The details of the object and their relationships with other objects as a json format string
    '''
    def get_object_by_id(self, pool, object_id):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()
            
            cur.execute("select json_agg(json_build_object( \
                        'object_id', obj.object_id, 'object_class_id', obj.object_class_id, 'class_confidence', obj.class_confidence, \
                        'name', oc.name, 'description', oc.description,  \
                        'associated_text', obj.associated_text, 'frame_id', obj.frame_id, \
                        'bbox_x', obj.bbox_x, 'bbox_y', obj.bbox_y, 'bbox_width', obj.bbox_width, 'bbox_height', obj.bbox_height, 'bbox_cols', obj.bbox_cols, 'bbox_rows', obj.bbox_rows, \
                        'loc_x', obj.loc_x, 'loc_y', obj.loc_y, 'loc_z', obj.loc_z, 'loc_confidence', obj.loc_confidence, \
                        'mean_colour', obj.mean_colour, 'texture', obj.texture, 'region_id', obj.region_id, 'region_label', r.label, \
                        'shape_id', obj.shape_id, 'shape_desc', osh.shape_desc, 'size_id', obj.size_id, 'size_desc', osz.size_desc, \
                        'img_path', obj.img_path, 'pcd_path', obj.pcd_path, \
                        'created', obj.created, 'last_updated', obj.last_updated, 'last_seen', obj.last_seen, 'is_current', obj.is_current)) \
                        from object obj \
                        left outer join object_shape osh on obj.shape_id = osh.shape_id \
                        left outer join object_size osz on obj.size_id = osz.size_id \
                        left outer join object_class oc on obj.object_class_id = oc.object_class_id \
						left outer join region r on obj.region_id = r.region_id \
                        where obj.object_id = %s", object_id)
    
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
        This method gets the object with the specific object_id and all of their relationships with other objects 
        and returns to the calling function
        Parameters: 
            pool:       database connection pool object, contains database connections
            object_id:  the object identifier that is the subject of the query
        Return:
            The details of the object and their relationships with other objects as a json format string
    '''
    def get_object_by_class(self, pool, class_name):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()
            
            cur.execute("select json_agg(json_build_object( \
                        'object_id', obj.object_id, 'object_class_id', obj.object_class_id, 'class_confidence', obj.class_confidence, \
                        'name', oc.name, 'description', oc.description,  \
                        'associated_text', obj.associated_text, 'frame_id', obj.frame_id, \
                        'bbox_x', obj.bbox_x, 'bbox_y', obj.bbox_y, 'bbox_width', obj.bbox_width, 'bbox_height', obj.bbox_height, 'bbox_cols', obj.bbox_cols, 'bbox_rows', obj.bbox_rows, \
                        'loc_x', obj.loc_x, 'loc_y', obj.loc_y, 'loc_z', obj.loc_z, 'loc_confidence', obj.loc_confidence, \
                        'mean_colour', obj.mean_colour, 'texture', obj.texture, 'region_id', obj.region_id, 'region_label', r.label, \
                        'shape_id', obj.shape_id, 'shape_desc', osh.shape_desc, 'size_id', obj.size_id, 'size_desc', osz.size_desc, \
                        'img_path', obj.img_path, 'pcd_path', obj.pcd_path, \
                        'created', obj.created, 'last_updated', obj.last_updated, 'last_seen', obj.last_seen, 'is_current', obj.is_current)) \
                        from object obj \
                        left outer join object_shape osh on obj.shape_id = osh.shape_id \
                        left outer join object_size osz on obj.size_id = osz.size_id \
                        left outer join object_class oc on obj.object_class_id = oc.object_class_id \
						left outer join region r on obj.region_id = r.region_id \
                        where oc.name like %s", (class_name,))
    
            result = cur.fetchone()[0]
            
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
        This method gets the object with the specific object_id and all of their relationships with other objects 
        and returns to the calling function
        Parameters: 
            pool:           database connection pool object, contains database connections
            object_lregion: the description of the region
        Return:
            The details of the object and their relationships with other objects as a json format string
    '''
    def get_object_by_region(self, pool, object_region):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()
            
            cur.execute("select json_agg(json_build_object( \
                        'object_id', obj.object_id, 'object_class_id', obj.object_class_id, 'class_confidence', obj.class_confidence, \
                        'name', oc.name, 'description', oc.description,  \
                        'associated_text', obj.associated_text, 'frame_id', obj.frame_id, \
                        'bbox_x', obj.bbox_x, 'bbox_y', obj.bbox_y, 'bbox_width', obj.bbox_width, 'bbox_height', obj.bbox_height, 'bbox_cols', obj.bbox_cols, 'bbox_rows', obj.bbox_rows, \
                        'loc_x', obj.loc_x, 'loc_y', obj.loc_y, 'loc_z', obj.loc_z, 'loc_confidence', obj.loc_confidence, \
                        'mean_colour', obj.mean_colour, 'texture', obj.texture, 'region_id', obj.region_id, 'region_label', r.label, \
                        'shape_id', obj.shape_id, 'shape_desc', osh.shape_desc, 'size_id', obj.size_id, 'size_desc', osz.size_desc, \
                        'img_path', obj.img_path, 'pcd_path', obj.pcd_path, \
                        'created', obj.created, 'last_updated', obj.last_updated, 'last_seen', obj.last_seen, 'is_current', obj.is_current)) \
                        from object obj \
                        left outer join object_shape osh on obj.shape_id = osh.shape_id \
                        left outer join object_size osz on obj.size_id = osz.size_id \
                        left outer join object_class oc on obj.object_class_id = oc.object_class_id \
						left outer join region r on obj.region_id = r.region_id \
                        where r.label like %s", object_region)
    
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
        Objects are not removed from the database but flagged as not current.
        This method sets the is current flag = 0 
        Parameters: 
            pool:       conection pool object
            object_id:  the id of the object 
        return: 
            if the update was successful 
                return the object_id
            else if the insert failed
                return -1
    '''
    def remove_object(self, pool, object_id):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()

            # set the current object prior to adding the new version of object           
            cur.execute("update object set is_current = 0, last_updated = %(last_updated)s where object_id = %(object_id)s and is_current = 1", (datetime.now(), object_id))  

            result = cur.fetchall()

            return result

        except (Exception, psycopg2.DatabaseError) as error:
            print(error)
            return -1
        finally:
            if conn:
                pool.putconn(conn)
            if cur is not None:
                cur.close()
                #print('Database cursor closed.')

    # TODO: Check the up Update Object function
    def update_object(self, pool, object_id, **kwargs):
        old_object = self.get_object_by_id(pool, object_id)[0][0][0]

        old_object.update(**kwargs)

        new_object = old_object
        try: 
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()

            # remove the object relationship data (if exists)
            obj_rels = new_object.pop("object_relationship", None)
            
            q = "update object set \
                    object_class_id = %(object_class_id)s, \
                    name            = %(name)s, \
                    description     = %(description)s, \
                    associated_text = %(associated_text)s, \
                    frame_id        = %(frame_id)s, \
                    pose_x      = %(pose_x)s, \
                    pose_y      = %(pose_y)s, \
                    pose_z      = %(pose_z)s, \
                    colour      = %(colour)s, \
                    hue_degrees     = %(hue_degrees)s, \
                    shape_id        = %(shape_id)s, \
                    bounding_box        = %(bounding_box)s, \
                    size_id     = %(size_id)s, \
                    mean_colour        = %(mean_colour)s, \
                    texture        = %(texture)s, \
                    image_path      = %(image_path)s, \
                    pc_path     = %(pc_path)s, \
                    class_confidence        = %(class_confidence)s, \
                    loc_confidence      = %(loc_confidence)s, \
                    created     = %(created)s, \
                    last_updated        = %(last_updated)s, \
                    last_seen        = %(last_seen)s, \
                    is_current      = %(is_current)s \
                where object_id = %(object_id)s;"

            #add the new record
            cur.execute(q, new_object)

            # add the object_relationship records
            if obj_rels:
                for rel in obj_rels:
                    rel.update({'primary_id': object_id})

                    q = "insert into object_relationship (related_id, object_relationship_type_id, primary_id) \
                        values (%(related_id)s, %(object_relationship_type_id)s, %(primary_id)s)"
                    cur.execute(q, rel)
            
            conn.commit()

            return True, "success"


        except (Exception, psycopg2.DatabaseError) as error:
            print("update_object error: {} {}".format(type(error), error))
            return False, "update_object error: {} {}".format(type(error), error)
        finally:
            if conn:
                pool.putconn(conn)
            if cur is not None:
                cur.close()
                #print('Database cursor closed.')


    '''
        This method updates the location of the object
         - loc_x, loc_y, loc_z, loc_confidence values and the last updated timestamp
        This occurs when the object has been seen again
        Parameters: 
            pool:              conection pool object
            object_id:         the id of the object
            loc_x:             the pose_z for the latest detection
            loc_y:             the pose_y for the latest detection
            loc_z:             the pose_z for the latest detection
            loc_confidence:    set to 1.0 as it has just been updated 
    '''
    def update_object_location(self, pool, object_id, loc_x, loc_y, loc_z):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()
                    
            cur.execute("update object set loc_x = %s,  loc_y = %s, loc_z = %s, loc_confidence = 1.0, last_seen = %s, last_updated = %s \
                    from object_class \
                    where object.object_id = %s and object.is_current = 1", (loc_x, loc_y, loc_z, datetime.now(), datetime.now(), object_id))  
            
            conn.commit()

        except (Exception, psycopg2.DatabaseError) as error:
            print(error)
            return -1
        finally:
            if conn:
                pool.putconn(conn)
            if cur is not None:
                cur.close()

    '''
        This method updates the location confidence and the last updated timestamp
        of the object based on the decay weight in the object class
        Parameters: 
            pool:       conection pool object
            object_id:  the id of the object
            std_decay_rate:     the standard rate of decay, the weight will be applied to this rate
            time_interval:      the interval in seconds i.e. since last update
    '''
    def update_location_confidence_by_object_id(self, pool, object_id, std_decay_rate, time_interval):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()
            
            # update the confidence of the object location           
            cur.execute("update o set o.loc_confidence = %s, o.last_updated = %s \
                    from object o inner join object_class oc on o.object_class_id = oc.object_class_id \
                    where o.object_id = %s", (o.loc_confidence-(time_interval*std_decay_rate*oc.decay_weight), datetime.now(), object_id))  
            conn.commit()   

        except (Exception, psycopg2.DatabaseError) as error:            
            rospy.loginfo("update_location_confidence error: {}: {}".format(type(error), error))
        finally:
            if conn:
                pool.putconn(conn)            
            if cur is not None:
                cur.close()


    '''
        This method updates the location confidence and the last updated timestamp
        for all current objects based on the decay rate, weight in the object class
        and time in seconds since last processed
        Parameters: 
            pool:       conection pool object
            std_decay_rate:     the standard rate of decay, the weight will be applied to this rate
            time_interval:      the interval in seconds i.e. since last update
    '''
    def update_location_confidence_all_objects(self, pool, std_decay_rate, time_interval):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()
            
            # update the confidence of the object location           
            cur.execute("update object set loc_confidence = loc_confidence - (%s*%s*decay_weight), last_updated = %s \
                    from object_class  \
                    where object.object_class_id = object_class.object_class_id and object.is_current = 1", (time_interval,std_decay_rate, datetime.now()))  
            conn.commit()   

        except (Exception, psycopg2.DatabaseError) as error:            
            rospy.loginfo("update_location_confidence error: {}: {}".format(type(error), error))
        finally:
            if conn:
                pool.putconn(conn)            
            if cur is not None:
                cur.close()


    '''
        This method updates the location, bounding box, location confidence, class confidence and last seen timestamp
        This method is called when the object has been seen again
        Parameters: 
            pool:              conection pool object
            object_id:         the id of the object
            obj:               the object in the detection with updated values

    '''
    def update_object_last_seen(self, pool, object_id, obj):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()            
                                          
            # TODO: modify the update statement to include condition for class confidence, i.e. only update if confidence is higher
            cur.execute("update object set loc_x = %s, loc_y = %s, loc_z = %s, \
                    bbox_x = %s, bbox_y = %s, bbox_width = %s, bbox_height = %s, bbox_cols = %s, bbox_rows = %s, \
                    loc_confidence = 1.0, class_confidence = %s, last_seen = %s, last_updated = %s \
                    where object.object_id = %s and object.is_current = 1", 
                    (obj.point[0], obj.point[1], obj.point[2], obj.bbox_x, obj.bbox_y, obj.bbox_width, obj.bbox_height, obj.bbox_cols, obj.bbox_rows, 
                     obj.class_confidence, datetime.now(), datetime.now(), object_id))  
            
            conn.commit()

        except (Exception, psycopg2.DatabaseError) as error:
            print(error)
            return -1
        finally:
            if conn:
                pool.putconn(conn)
            if cur is not None:
                cur.close()                
