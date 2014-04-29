#!/usr/bin/env python

import rospy
import yaml
import annotations_store.srv

from geometry_msgs.msg import *
from rospy_message_converter import message_converter
from ar_track_alvar.msg import AlvarMarkers, AlvarMarker

def read(filename):
    yaml_data = None 
    with open(filename) as f:
       yaml_data = yaml.load(f)

    ar_list = AlvarMarkers()

    for m in yaml_data:
        ar = AlvarMarker()
        ar.id = m['id']
        ar.confidence = m['confidence']
        ar.pose.header.frame_id = m['frame_id']
        ar.pose.header.stamp = rospy.Time.now()
        ar.pose.pose = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',m['pose'])
        ar_list.markers.append(ar)
    
    return ar_list


if __name__ == '__main__':
    rospy.init_node('markers_saver')
    map_uuid = rospy.get_param('~map_uuid')
    filename = rospy.get_param('~filename')
    markers = read(filename)

    print "Waiting for save_markers service..."
    rospy.wait_for_service('save_markers')
    save_srv = rospy.ServiceProxy('save_markers', annotations_store.srv.SaveMarkers)

#    rospy.loginfo('Saving AR markers from ', filename,' with map uuid ',map_uuid)
    save_srv(map_uuid, markers)
    print "Done"
