#!/usr/bin/env python

import rospy
import yaml
import annotations_store.srv

from geometry_msgs.msg import *
from rospy_message_converter import message_converter
from yocs_msgs.msg import Column, ColumnList

def read(filename):
    yaml_data = None 
    with open(filename) as f:
       yaml_data = yaml.load(f)

    column_list = ColumnList()

    for t in yaml_data:
        object = Column()
        object.name = t['name']
        object.radius = float(t['radius'])
        object.height = float(t['height'])
        object.pose.header.frame_id = t['frame_id']
        object.pose.header.stamp = rospy.Time.now()
        object.pose.pose.pose = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',t['pose'])
        column_list.obstacles.append(object)
    
    return column_list

if __name__ == '__main__':
    rospy.init_node('columns_saver')
    map_uuid = rospy.get_param('~map_uuid')
    filename = rospy.get_param('~filename')
    columns = read(filename)

    print "Waiting for save_columns service..."
    rospy.wait_for_service('save_columns')
    save_srv = rospy.ServiceProxy('save_columns', annotations_store.srv.SaveColumns)

#    rospy.loginfo('Saving virtual columns from ', filename,' with map uuid ',map_uuid)
    save_srv(map_uuid, columns)
    print "Done"
