#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2014, Yujin Robot
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Jorge Santos

import roslib; roslib.load_manifest('warehouse_ros')
import roslib.message
import rospy
import unique_id
import cPickle as pickle
import warehouse_ros as wr

from world_canvas_msgs.msg import *
from world_canvas_msgs.srv import *


class AnnotationsServer:

    ##########################################################################
    # Constants
    ##########################################################################
    MAX_KEYWORDS = 10
    
    ##########################################################################
    # Initialization
    ##########################################################################
    def __init__(self):
        # Set up collections
        self.anns_collection = \
            wr.MessageCollection("world_canvas", "annotations", Annotation)
        self.anns_collection.ensure_index("id")

        self.data_collection = \
            wr.MessageCollection("world_canvas", "annotations_data", AnnotationData)
        self.data_collection.ensure_index("id")
        
        self.get_anns_srv = \
            rospy.Service('get_annotations',      GetAnnotations,     self.getAnnotations)
        self.get_data_srv = \
            rospy.Service('get_annotations_data', GetAnnotationsData, self.getAnnotationsData)
        self.pub_data_srv = \
            rospy.Service('pub_annotations_data', PubAnnotationsData, self.pubAnnotationsData)
        
        self.load_data_srv = \
            rospy.Service('load_annotations_data', LoadAnnotationsData, self.loadAnnotationsData)
        self.save_data_srv = \
            rospy.Service('save_annotations_data', SaveAnnotationsData, self.saveAnnotationsData)

        rospy.loginfo("Annotations server : initialized.")


    def getAnnotations(self, request):

        response = GetAnnotationsResponse()
        
        query = {'world_id': {'$in': [unique_id.toHexString(request.world_id)]}}
        if len(request.ids) > 0:
            query['id'] = {'$in': [unique_id.toHexString(id) for id in request.ids]}
        if len(request.types) > 0:
            query['type'] = {'$in': request.types}
        if len(request.keywords) > 0:
            query['$or'] = []
            for i in range(1, self.MAX_KEYWORDS):
                query['$or'].append({'keyword' + str(i): {'$in': request.keywords}})
                
        matching_anns = self.anns_collection.query(query)            

        i = 0
        while True:
            try:
                response.annotations.append(matching_anns.next()[0])
                i += 1
            except StopIteration:
                if (i == 0):
                    rospy.loginfo("No annotations found for query %s", query)
                    response.result = True  # we don't consider this an error
                    return response
                break
    
    
#         if (len(matching_anns) != len(matching_data)):
#             # we consider this an error by now, as we assume a 1 to 1 relationship;
#             # but in future implementations this will change, probably, to a N to 1 relationship
#             rospy.logerror("Pulled annotations and associated data don't match (%lu != %lu)",
#                      len(matching_anns), len(matching_data))
#             response.message = "Pulled annotations and associated data don't match"
#             response.result = False
#             return response
#     
#         response.annotations = matching_anns
#         response.data        = matching_data
    
        rospy.loginfo("%lu annotations loaded", i)
        response.result = True
        return response
        
    def getAnnotationsData(self, request):
        response = GetAnnotationsDataResponse()
        
        query = {'id': {'$in': [unique_id.toHexString(id) for id in request.annotation_ids]}}                
        matching_data = self.data_collection.query(query)

        i = 0
        while True:
            try:
                response.data.append(matching_data.next()[0])
                i += 1
            except StopIteration:
                if (i == 0):
                    rospy.loginfo("No annotations data found for query %s", query)  # we don't consider this an error
                else:
                    rospy.loginfo("%d objects found for %d annotations", i, len(request.annotation_ids))
                break

        response.result = True
        return response
        
    def pubAnnotationsData(self, request):
        response = PubAnnotationsDataResponse()
        
        query = {'id': {'$in': [unique_id.toHexString(id) for id in request.annotation_ids]}}                
        matching_data = self.data_collection.query(query)

        topic_class = roslib.message.get_message_class(request.topic_type)
        pub = rospy.Publisher(request.topic_name, topic_class, latch = True)
    
        i = 0
        object_list = list()
        while True:
            try:
                ann_data = matching_data.next()[0]
                object = pickle.loads(ann_data.data)
                if request.pub_as_list:
                    object_list.append(object)
                else:
                    pub.publish(object)
                    
                i += 1
            except StopIteration:
                if (i == 0):
                    rospy.loginfo("No annotations data found for query %s", query)  # we don't consider this an error
                else:
                    rospy.loginfo("%d objects found for %d annotations", i, len(request.annotation_ids))
                    if request.pub_as_list:
                        pub.publish(object_list)
                break

        response.result = True
        return response
        
        
    def loadAnnotationsData(self, request):

        response = LoadAnnotationsDataResponse()
        
        query = {'world_id': {'$in': [unique_id.toHexString(request.world_id)]}}
        matching_anns = self.anns_collection.query(query)
        matching_data = self.data_collection.query(query)

        i = 0
        while True:
            try:
                response.annotations.append(matching_anns.next()[0])
                response.data.append(matching_data.next()[0])
                i += 1
            except StopIteration:
                if (i == 0):
                    rospy.loginfo("No annotations found for map '%s'; we don't consider this an error",
                              request.world_id)
                    response.result = True  # we don't consider this an error
                    return response
                break
    
    
#         if (len(matching_anns) != len(matching_data)):
#             # we consider this an error by now, as we assume a 1 to 1 relationship;
#             # but in future implementations this will change, probably, to a N to 1 relationship
#             rospy.logerror("Pulled annotations and associated data don't match (%lu != %lu)",
#                      len(matching_anns), len(matching_data))
#             response.message = "Pulled annotations and associated data don't match"
#             response.result = False
#             return response
#     
#         response.annotations = matching_anns
#         response.data        = matching_data
    
        rospy.loginfo("%lu annotations loaded", i)
        response.result = True
        return response

    def saveAnnotationsData(self, request):
        response = SaveAnnotationsDataResponse()

        print request.annotations
        for i, annotation in enumerate(request.annotations):
            data = request.data[i]
            
            metadata = { 'world_id': unique_id.toHexString(annotation.world_id),
                         'id'      : unique_id.toHexString(annotation.id),
                         'name'    : annotation.name,
                         'type'    : annotation.type 
                       }
            for i, kw in enumerate(annotation.keywords):
                if i >= self.MAX_KEYWORDS:
                    rospy.logwarn('Only %d keywords can be stored; the other %d will be discarded',
                                   self.MAX_KEYWORDS, len(annotation.keywords) - self.MAX_KEYWORDS)
                    break
                metadata['keyword' + str(i + 1)] = kw
            
            rospy.logdebug("Saving annotation %s for map %s", annotation.id, annotation.world_id)

            ###mr::Query q = mr::Query().append("world_id", annotation.world_id).append("id", annotation.id)
            self.anns_collection.remove({'id': {'$in': [unique_id.toHexString(annotation.id)]}})
            self.anns_collection.insert(annotation, metadata)
            self.data_collection.remove({'id': {'$in': [unique_id.toHexString(annotation.id)]}})
            self.data_collection.insert(data, metadata)

        rospy.loginfo("%lu annotations saved", len(request.annotations))
        response.result = True
        return response


if __name__ == "__main__":
    rospy.init_node('annotations_server')

    AnnotationsServer()
  
    rospy.spin()
