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
from yocs_msgs.msg import Wall, WallList
from visualization_msgs.msg import Marker, MarkerArray


def publish(anns, data):

    wall_list = WallList()
    marker_list = MarkerArray()    

    marker_id = 1
    for a, d in zip(anns, data):
        
        # Walls
        object = pickle.loads(d.data)
        wall_list.obstacles.append(object)

        # Markers
        marker = Marker()
        marker.id = marker_id
        marker.header = object.pose.header
        marker.type = a.shape
        marker.ns = "wall_obstacles"
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration.from_sec(0)
        marker.pose = copy.deepcopy(object.pose.pose.pose)
        marker.pose.position.z += object.height/2.0
        marker.scale = a.size
        marker.color = a.color

        marker_list.markers.append(marker)

        marker_id = marker_id + 1


    marker_pub = rospy.Publisher('wall_marker',    MarkerArray, latch = True)
    wall_pub   = rospy.Publisher('wall_pose_list', WallList,    latch = True)

    wall_pub.publish(wall_list)
    marker_pub.publish(marker_list)
    
    return

if __name__ == '__main__':
    rospy.init_node('walls_loader')
    world_id = rospy.get_param('~world_id')

    print "Waiting for load_annotations_data service..."
    rospy.wait_for_service('load_annotations_data')

    rospy.loginfo('Loading annotations with world uuid ' + world_id)
    load_srv = rospy.ServiceProxy('load_annotations_data', world_canvas_msgs.srv.LoadAnnotationsData)
    response = load_srv(unique_id.toMsg(uuid.UUID('urn:uuid:' + world_id)))

    rospy.loginfo('Publishing obstacles and visualization markers...')
    publish(response.annotations, response.data)

    print "Done"
    rospy.spin()
