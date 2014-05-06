#!/bin/bash

rosrun annotations_store save_markers.py _map_uuid:=$1 _filename:=./ar_list.yaml
rosrun annotations_store save_tables.py  _map_uuid:=$1 _filename:=./table_list.yaml
rosrun annotations_store save_columns.py _map_uuid:=$1 _filename:=./column_list.yaml
rosrun annotations_store save_walls.py   _map_uuid:=$1 _filename:=./wall_list.yaml
