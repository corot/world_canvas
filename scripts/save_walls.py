#!/usr/bin/env python

import rospy
import yaml
import annotations_store.srv

from geometry_msgs.msg import *
from rospy_message_converter import message_converter
from yocs_msgs.msg import Wall, WallList

def read(filename):
    yaml_data = None 
    with open(filename) as f:
       yaml_data = yaml.load(f)

    wall_list = WallList()

    for t in yaml_data:
        object = Wall()
        object.name = t['name']
        object.length = float(t['length'])
        object.width  = float(t['width'])
        object.height = float(t['height'])
        object.pose.header.frame_id = t['frame_id']
        object.pose.header.stamp = rospy.Time.now()
        object.pose.pose.pose = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',t['pose'])
        wall_list.obstacles.append(object)
    
    return wall_list

if __name__ == '__main__':
    rospy.init_node('walls_saver')
    map_uuid = rospy.get_param('~map_uuid')
    filename = rospy.get_param('~filename')
    walls = read(filename)

    print "Waiting for save_walls service..."
    rospy.wait_for_service('save_walls')
    save_srv = rospy.ServiceProxy('save_walls', annotations_store.srv.SaveWalls)

#    rospy.loginfo('Saving virtual walls from ', filename,' with map uuid ',map_uuid)
    save_srv(map_uuid, walls)
    print "Done"
