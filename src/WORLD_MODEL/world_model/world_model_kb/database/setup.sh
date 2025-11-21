#!/bin/bash

DB="world_model"
echo Database: $DB

psql $DB -f ./setup_drop_tables_2.1.4.sql
psql $DB -f ./setup_schema_2.1.4.sql
psql $DB -f ./setup_insert_1.3.1.sql

# For simplicity setup the regions as well
#psql $DB -f ./sample_data/regions.sql


