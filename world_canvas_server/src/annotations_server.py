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
import rospy
import unique_id
import warehouse_ros as wr

from world_canvas_msgs.msg import *
from world_canvas_msgs.srv import *


class AnnotationsServer:

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
        
        self.load_data_srv = \
            rospy.Service('load_annotations_data', LoadAnnotationsData, self.loadAnnotationsData)
        self.save_data_srv = \
            rospy.Service('save_annotations_data', SaveAnnotationsData, self.saveAnnotationsData)

        rospy.loginfo("Annotations server : initialized.")
        
    def loadAnnotationsData(self, request):

        response = LoadAnnotationsDataResponse()
        matching_anns = self.anns_collection.query({'map_uuid': {'$in': [request.map_uuid]}})
        matching_data = self.data_collection.query({'map_uuid': {'$in': [request.map_uuid]}})

        i = 0
        while True:
            try:
                response.annotations.append(matching_anns.next()[0])
                response.data.append(matching_data.next()[0])
                i += 1
            except StopIteration:
                if (i == 0):
                    rospy.loginfo("No annotations found for map '%s'; we don't consider this an error",
                              request.map_uuid)
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
            
            #          std::string annotation_id = uuid2str(annotation.id.uuid.c_array())
            metadata = { 'map_uuid': annotation.map_uuid,
                         'world_id': unique_id.toHexString(annotation.world_id),
                         'id'      : unique_id.toHexString(annotation.id) }
            #     //    mr::Metadata metadata = mr::Metadata("timestamp",   request.map_name,
            #     //                                         "map_uuid",   request.map_uuid,
            #     //                                         "world_id", request.session_id,
            #     //                                         "id",   request.map_uuid,
            #     //                                         "name", request.session_id,
            #     //                                         "type", request.session_id)
            
            rospy.logdebug("Saving annotation %s for map %s", annotation.id, annotation.map_uuid)

            ###mr::Query q = mr::Query().append("map_uuid", annotation.map_uuid).append("id", annotation.id)
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
