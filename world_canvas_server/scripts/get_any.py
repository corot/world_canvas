#!/usr/bin/env python

import rospy
import roslib
import rostopic
import importlib
import yaml
import uuid
import copy
import unique_id
import cPickle as pickle
import world_canvas_msgs.msg
import world_canvas_msgs.srv

from geometry_msgs.msg import *
from rospy_message_converter import message_converter
from visualization_msgs.msg import Marker, MarkerArray


def publish(anns, data, topic_name, topic_type, pub_as_list):
    # Process retrieved data to build annotations and markers lists
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
        marker.scale = a.size
        marker.color = a.color

        marker_list.markers.append(marker)

        marker_id = marker_id + 1

    # Advertise topics for retrieved annotations and their visualization markers
    # Message type is topic_type if we publish results as a list; a.type otherwise
    if not pub_as_list:
        topic_type = a.type
    
    topic_class = roslib.message.get_message_class(topic_type)
    if topic_class is None:
        # This happens if the topic type is wrong or not known (i.e. absent from ROS_PACKAGE_PATH)
        rospy.logerr("Topic type %s definition not found; unable to publish annotations" % topic_type)
        return

    marker_pub = rospy.Publisher(topic_name + '_markers', MarkerArray, latch=True, queue_size=1)
    object_pub = rospy.Publisher(topic_name + '_client',  topic_class, latch=True, queue_size=1)

    marker_pub.publish(marker_list)

    # Publish resulting lists
    if pub_as_list:
        object_pub.publish(object_list)
    else:
        # if pub_as_list is false, publish objects one by one
        for object in object_list:
            if isinstance(object, topic_class):
                object_pub.publish(object)
            else:
                rospy.logwarn("Object is not of type %s but %s" % (topic_type, type(object).__name__))
    
# Other ways to do the same: not using ros magic
#     module_name, class_name = topic_type.rsplit(".", 1)
#     module = importlib.import_module(module_name)
#     topic_class = getattr(module, class_name)
#
# or inspecting an already advertised topic
#     input_class, input_topic, input_fn = rostopic.get_topic_class('/annotations')
#     topic_class = roslib.message.get_message_class('yocs_msgs/ColumnList')

    return

if __name__ == '__main__':
    rospy.init_node('objects_loader')
    topic_name  = rospy.get_param('~topic_name', 'annotations')
    topic_type  = rospy.get_param('~topic_type', None)
    pub_as_list = rospy.get_param('~pub_as_list', False)
    world_id = rospy.get_param('~world_id')
    ids      = rospy.get_param('~ids', [])
    names    = rospy.get_param('~names', [])
    types    = rospy.get_param('~types', [])
    keywords = rospy.get_param('~keywords', [])
    related  = rospy.get_param('~relationships', [])

    if pub_as_list and topic_type is None:
        rospy.logerr("You must specify the topic type if pub_as_list is true")
        sys.exit()
    
    rospy.loginfo("Waiting for get_annotations service...")
    rospy.wait_for_service('get_annotations')

    rospy.loginfo('Loading annotations with map uuid %s', world_id)
    get_anns_srv = rospy.ServiceProxy('get_annotations', world_canvas_msgs.srv.GetAnnotations)
    respAnns = get_anns_srv(unique_id.toMsg(uuid.UUID('urn:uuid:' + world_id)),
                           [unique_id.toMsg(uuid.UUID('urn:uuid:' + id)) for id in ids],
                            names, types, keywords,
                           [unique_id.toMsg(uuid.UUID('urn:uuid:' + r)) for r in related])

    if len(respAnns.annotations) > 0:
        rospy.loginfo('Publishing visualization markers for %d retrieved annotations...',
                       len(respAnns.annotations))
    else:
        rospy.loginfo('No annotations found for map %s with the given search criteria', world_id)
        sys.exit()

    rospy.loginfo('Loading data for the %d retrieved annotations', len(respAnns.annotations))
    get_data_srv = rospy.ServiceProxy('get_annotations_data', world_canvas_msgs.srv.GetAnnotationsData)
    respData = get_data_srv([a.data_id for a in respAnns.annotations])

    if len(respData.data) > 0:
        rospy.loginfo('Publishing data for %d retrieved annotations...', len(respData.data))
        publish(respAnns.annotations, respData.data, topic_name, topic_type, pub_as_list)
    else:
        rospy.logwarn('No data found for the %d retrieved annotations', len(respAnns.annotations))

    rospy.loginfo('Requesting server to also publish the same data')
    pub_data_srv = rospy.ServiceProxy('pub_annotations_data', world_canvas_msgs.srv.PubAnnotationsData)
    respData = pub_data_srv([a.data_id for a in respAnns.annotations], topic_name, topic_type, pub_as_list)

    rospy.loginfo("Done")
    rospy.spin()
