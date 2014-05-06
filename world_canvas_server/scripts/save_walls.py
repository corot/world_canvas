#!/usr/bin/env python

import rospy
import yaml
import uuid
import pickle
import unique_id
import world_canvas_msgs.msg
import world_canvas_msgs.srv

from geometry_msgs.msg import *
from rospy_message_converter import message_converter
from yocs_msgs.msg import Wall, WallList
from world_canvas_msgs.msg import Annotation, AnnotationData

def read(filename):
    yaml_data = None 
    with open(filename) as f:
       yaml_data = yaml.load(f)

    anns_list = []
    data_list = []


    for t in yaml_data:
        ann = Annotation()
        ann.timestamp = rospy.Time.now()
        ann.map_uuid = map_uuid
        ann.world_id = unique_id.toMsg(uuid.UUID('urn:uuid:' + map_uuid))  #unique_id.toMsg(unique_id.fromRandom())
        ann.id = unique_id.toMsg(unique_id.fromRandom())
        ann.name = t['name']
        ann.type = 'wall'
        ann.shape = 1 # CUBE
        ann.color.r = 0.8
        ann.color.g = 0.2
        ann.color.b = 0.2
        ann.color.a = 0.4
        ann.size.x = float(t['length'])
        ann.size.y = float(t['width'])
        ann.size.z = float(t['height'])
        ann.pose.header.frame_id = t['frame_id']
        ann.pose.header.stamp = rospy.Time.now()
        ann.pose.pose.pose = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',t['pose'])
# Querying properties
# string[] keywords
# uuid_msgs/UniqueID[] relationships
        anns_list.append(ann)

        object = Wall()
        object.name = t['name']
        object.length = float(t['length'])
        object.width  = float(t['width'])
        object.height = float(t['height'])
        object.pose.header.frame_id = t['frame_id']
        object.pose.header.stamp = rospy.Time.now()
        object.pose.pose.pose = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',t['pose'])
        data = AnnotationData()
        data.id = ann.id
        data.data = pickle.dumps(object)
        
        data_list.append(data)
        
        print ann, object, data
    
    return anns_list, data_list

if __name__ == '__main__':
    rospy.init_node('walls_saver')
    map_uuid = rospy.get_param('~map_uuid')
    filename = rospy.get_param('~filename')
    anns, data = read(filename)

    print "Waiting for save_annotations_data service..."
    rospy.wait_for_service('save_annotations_data')
    save_srv = rospy.ServiceProxy('save_annotations_data', world_canvas_msgs.srv.SaveAnnotationsData)

#    rospy.loginfo('Saving virtual walls from ', filename,' with map uuid ',map_uuid)
    save_srv(map_uuid, unique_id.toMsg(unique_id.fromRandom()), anns, data)
    print "Done"
