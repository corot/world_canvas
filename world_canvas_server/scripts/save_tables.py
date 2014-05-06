#!/usr/bin/env python

import rospy
import yaml
import annotations_store.srv

from geometry_msgs.msg import *
from rospy_message_converter import message_converter
from yocs_msgs.msg import Table, TableList

def read(filename):
    yaml_data = None 
    with open(filename) as f:
       yaml_data = yaml.load(f)

    table_list = TableList()

    for t in yaml_data:
        object = Table()
        object.name = t['name']
        object.radius = float(t['radius'])
        object.height = float(t['height'])
        object.pose.header.frame_id = t['frame_id']
        object.pose.header.stamp = rospy.Time.now()
        object.pose.pose.pose = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',t['pose'])
        table_list.tables.append(object)
    
    return table_list

if __name__ == '__main__':
    rospy.init_node('tables_saver')
    map_uuid = rospy.get_param('~map_uuid')
    filename = rospy.get_param('~filename')
    tables = read(filename)

    print "Waiting for save_tables service..."
    rospy.wait_for_service('save_tables')
    save_srv = rospy.ServiceProxy('save_tables', annotations_store.srv.SaveTables)

#    rospy.loginfo('Saving virtual tables from ', filename,' with map uuid ',map_uuid)
    save_srv(map_uuid, tables)
    print "Done"
