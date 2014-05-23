#!/usr/bin/env python

import rospy
import yaml
import uuid
import copy
import pickle
import unique_id
import world_canvas_msgs.msg
import world_canvas_msgs.srv

from geometry_msgs.msg import *
from rospy_message_converter import message_converter
from yocs_msgs.msg import Table, TableList
from visualization_msgs.msg import Marker, MarkerArray


def publish(anns, data):
    table_list = TableList()
    marker_list = MarkerArray()    

    marker_id = 1
    for a, d in zip(anns, data):
        
        # Tables
        object = pickle.loads(d.data)
        table_list.tables.append(object)
        
        # Markers
        marker = Marker()
        marker.id = marker_id
        marker.header = a.pose.header
        marker.type = a.shape
        marker.ns = "table_obstacles"
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration.from_sec(0)
        marker.pose = copy.deepcopy(a.pose.pose.pose)
#        marker.pose.position.z += marker.pose.position.z/2.0
        marker.scale = a.size
        marker.color = a.color

        marker_list.markers.append(marker)

        marker_id = marker_id + 1

    marker_pub = rospy.Publisher('table_marker',    MarkerArray, latch = True)
    table_pub  = rospy.Publisher('table_pose_list', TableList,   latch = True)

    table_pub.publish(table_list)
    marker_pub.publish(marker_list)
    
    return


if __name__ == '__main__':
    rospy.init_node('tables_loader')
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
