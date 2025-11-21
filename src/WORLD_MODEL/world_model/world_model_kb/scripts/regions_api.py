# version 2.0
# written by Adam Golding

import psycopg2
from psycopg2 import pool
import json
from datetime import datetime

class RegionsApi:

    '''
        This method inserts region data into the database
        Parameters: 
            pool:               database connection pool object, contains database connections
            label:              
            polygon_points_x:   
            polygon_points_y:   
            neigbors:           
            region_id:          

        Return:
            primary_id:     the region id
    '''
    def insert_region(self, pool, label, polygon_points_x, polygon_points_y, neighbors=[], region_id=None):
        polygon_points =["(" + str(x) + "," +str(y) + ")" for (x, y) in zip(polygon_points_x, polygon_points_y)]
        polygon_string = '(' + ",".join(polygon_points) + ')'
        print(polygon_string)
        #ewks_polygon = ppygis.Polygon([ppygis.LineString(polygon_points)])
        #print(ewks_polygon)
        try:
            conn = pool.getconn()
            cur = conn.cursor()
            
            if region_id is None:
                query = "insert into region(label, polygon, neighbors) \
                        values (%(label)s, %(polygon)s, %(neighbors)s) \
                        returning region_id"
            else: 
                query = "insert into region(region_id, label, polygon, neighbors) \
                        values (%(region_id)s, %(label)s, %(polygon)s, %(neighbors)s) \
                        returning region_id"
            data = {"label": label, "polygon": polygon_string, "region_id": region_id, "neighbors": neighbors}
            print(cur.mogrify(query, data))
            cur.execute(query, data)
            primary_id = cur.fetchone()[0]

            conn.commit()
            return primary_id

        except (Exception, psycopg2.DatabaseError) as error:
            print("insert_region error: " + str(type(error)) + ": " + str(error))
            return -1
        finally:
            if conn:
                pool.putconn(conn)
            if cur is not None:
                cur.close()
                #print('Database cursor closed.')


    '''
    
    '''
    def find_point_label(self, pool, x, y):
        '''
        Use sql's geometry features to lookup which polygon contains the point <x,y>
        Returns: json object with label and region_id
        '''
        try:
            conn = pool.getconn()
            cur = conn.cursor()

            query = "select json_build_object('region_id', r.region_id, 'label', r.label) \
                    from region r where point '(%(x)s, %(y)s)' <@ r.polygon"

            data = {"x": x, "y": y}
            cur.execute(query, data)
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
    
    '''
    def get_regions(self, pool):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()

            cur.execute("SELECT json_agg(json_build_object('region_id', r.region_id, 'label', r.label, \
                    'polygon', r.polygon, 'neighbors', r.neighbors)) \
                    from region r")

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
    Get region by label
    '''
    def get_region_by_label(self, pool, label):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()

            cur.execute(f"SELECT json_build_object('region_id', r.region_id, 'label', r.label, \
                    'polygon', r.polygon, 'neighbors', r.neighbors) \
                    from region r where r.label = '{label}';")

            result = cur.fetchone()

            return result[0]

        except (Exception, psycopg2.DatabaseError) as error:            
            print("Error in API method 'get_region_by_label': ", error)
        finally:
            if conn:
                pool.putconn(conn)            
            if cur is not None:
                cur.close()

    '''
        This method gets the list of point of interest
        Parameters:     none

        Return:         List of all points of interest (point_of_interest_id, name, loc_x, loc_y, loc_z, region) in the database as a json format string
    '''
    def get_poi_list(self, pool):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()

            cur.execute("SELECT json_agg(json_build_object('point_of_interest_id', p.point_of_interest_id, 'name', p.name, \
                    'loc_x', p.loc_x, 'loc_y', p.loc_y, 'loc_z', p.loc_z, 'yaw', p.yaw, 'region', r.label)) \
                    from point_of_interest p left outer join region r on p.region_id = r.region_id;")

            result = cur.fetchone()[0]

            return result

        except (Exception, psycopg2.DatabaseError) as error:            
            print("Error in API method 'get_poi_list': ", error)
        finally:
            if conn:
                pool.putconn(conn)            
            if cur is not None:
                cur.close()

    '''
    
    '''
    def get_poi_list_inc_desc(self, pool):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()

            cur.execute("SELECT json_agg(json_build_object('point_of_interest_id', p.point_of_interest_id, 'name', p.name, \
                    'description', p.description, 'loc_x', p.loc_x, 'loc_y', p.loc_y, 'loc_z', p.loc_z, 'yaw', p.yaw, 'region', r.label)) \
                    from point_of_interest p left outer join region r on p.region_id = r.region_id;")

            result = cur.fetchone()[0]

            return result

        except (Exception, psycopg2.DatabaseError) as error:            
            print("Error in API method 'get_poi_list_inc_desc': ", error)
        finally:
            if conn:
                pool.putconn(conn)            
            if cur is not None:
                cur.close()

    '''
    
    '''
    def get_poi_by_name(self, pool, poi_name):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()

            cur.execute(f"SELECT json_agg(json_build_object('point_of_interest_id', p.point_of_interest_id, 'name', p.name, \
                    'loc_x', p.loc_x, 'loc_y', p.loc_y, 'loc_z', p.loc_z, 'yaw', p.yaw, 'region', r.label)) \
                    from point_of_interest p left outer join region r on p.region_id = r.region_id \
                    where name = '{poi_name}';")

            result = cur.fetchone()

            return result[0]

        except (Exception, psycopg2.DatabaseError) as error:            
            print("Error in API method 'get_poi_by_name': ", error)
        finally:
            if conn:
                pool.putconn(conn)            
            if cur is not None:
                cur.close()

    '''
    
    '''
    def get_poi_by_name_inc_desc(self, pool, name):
        try:       
            # get connection and create a cursor
            conn = pool.getconn()
            cur = conn.cursor()

            cur.execute(f"SELECT json_agg(json_build_object('point_of_interest_id', p.point_of_interest_id, 'name', p.name, \
                    'description', p.description, 'loc_x', p.loc_x, 'loc_y', p.loc_y, 'loc_z', p.loc_z, 'yaw', p.yaw, 'region', r.label)) \
                    from point_of_interest p left outer join region r on p.region_id = r.region_id \
                    where name = '{name}';")

            result = cur.fetchone()[0]

            return result

        except (Exception, psycopg2.DatabaseError) as error:            
            print("Error in API method 'get_poi_by_name_inc_desc': ", error)
        finally:
            if conn:
                pool.putconn(conn)            
            if cur is not None:
                cur.close()                