#!/usr/bin/env python3
import psycopg2
from configparser import ConfigParser
from psycopg2.pool import SimpleConnectionPool

'''
The Db class contains methods to parse the database settings from the initialisation file (.ini)

List of Methods:
- read_config(self, filename, section)
'''
class DBase:
    def __init__(self, config_path):
        db_params = self.read_config(config_path, "postgresql")
        pool_params = self.read_config(config_path, "pool")
        self.con_pool = psycopg2.pool.SimpleConnectionPool(pool_params['min'], pool_params['max'], **db_params)

    def __del__(self):
        if (self.con_pool):
            self.con_pool.closeall
            print("Database Connections Closed")

    def read_config(self, filename, section):
        # create a parser
        parser = ConfigParser()
        # read config file
        parser.read(filename)
    
        # get section, default to postgresql
        prm = {}
        if parser.has_section(section):
            params = parser.items(section)
            for param in params:
                prm[param[0]] = param[1]
        else:
            raise Exception('Section {0} not found in the {1} file'.format(section, filename))            
    
        return prm
