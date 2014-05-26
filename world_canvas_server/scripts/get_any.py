#!/usr/bin/env python

import rospy
import roslib
import rostopic
import importlib
import yaml
import uuid
import copy
import pickle
import unique_id
import world_canvas_msgs.msg
import world_canvas_msgs.srv

from geometry_msgs.msg import *
from rospy_message_converter import message_converter
from visualization_msgs.msg import Marker, MarkerArray


def publish(anns, data):
    object_list = list()
    marker_list = MarkerArray()    

    marker_id = 1
    for a, d in zip(anns, data):
        
        # Objects
        object = pickle.loads(d.data)
        object_list.append(object)
        
        # Markers
        marker = Marker()
        marker.id = marker_id
        marker.header = a.pose.header
        marker.type = a.shape
        marker.ns = a.type
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration.from_sec(0)
        marker.pose = copy.deepcopy(a.pose.pose.pose)
#        marker.pose.position.z += marker.pose.position.z/2.0
        marker.scale = a.size
        marker.color = a.color

        marker_list.markers.append(marker)

        marker_id = marker_id + 1

    topic_class = roslib.message.get_message_class(topic_type)
     
    marker_pub = rospy.Publisher(topic_name + '_markers', MarkerArray, latch = True)
    object_pub = rospy.Publisher(topic_name,              topic_class, latch = True)
 
    marker_pub.publish(marker_list)
    object_pub.publish(object_list)
    
    
# Other ways to do the same: not using ros magic
#     module_name, class_name = topic_type.rsplit(".", 1)
#     module = importlib.import_module(module_name)
#     topic_class = getattr(module, class_name)
#
# or inspecting an already advertised topic
#     input_class, input_topic, input_fn = rostopic.get_topic_class('/annotations')
#     topic_class = roslib.message.get_message_class('yocs_msgs/ColumnList')

    return

def callback(self, m):
    return

if __name__ == '__main__':
    rospy.init_node('objects_loader')
    topic_name = rospy.get_param('~topic_name', 'annotations')
    topic_type = rospy.get_param('~topic_type')
    world_id = rospy.get_param('~world_id')
    ids      = rospy.get_param('~ids', [])
    types    = rospy.get_param('~types', [])
    keywords = rospy.get_param('~keywords', [])

    rospy.loginfo("Waiting for get_annotations service...")
    rospy.wait_for_service('get_annotations')

    rospy.loginfo('Loading annotations with map uuid %s', world_id)
    get_anns_srv = rospy.ServiceProxy('get_annotations', world_canvas_msgs.srv.GetAnnotations)
    respAnns = get_anns_srv(unique_id.toMsg(uuid.UUID('urn:uuid:' + world_id)),
                           [unique_id.toMsg(uuid.UUID('urn:uuid:' + id)) for id in ids],
                            types, keywords, [], [], [])

    if len(respAnns.annotations) > 0:
        rospy.loginfo('Publishing visualization markers for %d retrieved annotations...',
                       len(respAnns.annotations))
    else:
        rospy.loginfo('No annotations found for map %s with the given search criteria', world_id)
        sys.exit()

    rospy.loginfo('Loading data for the %d retrieved annotations', len(respAnns.annotations))
    get_data_srv = rospy.ServiceProxy('get_annotations_data', world_canvas_msgs.srv.GetAnnotationsData)
    respData = get_data_srv([a.id for a in respAnns.annotations])

    if len(respData.data) > 0:
        rospy.loginfo('Publishing data for %d retrieved annotations...', len(respData.data))
        publish(respAnns.annotations, respData.data)
    else:
        rospy.logwarn('No data found for the %d retrieved annotations', len(respAnns.annotations))
        
    rospy.loginfo("Done")
    rospy.spin()
