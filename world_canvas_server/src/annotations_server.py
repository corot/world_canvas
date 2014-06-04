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

from yaml_database import *

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

        self.set_keyword_srv = \
            rospy.Service('set_keyword',      SetKeyword,      self.setKeyword)
        self.set_related_srv = \
            rospy.Service('set_relationship', SetRelationship, self.setRelationship)

        # Configure import from/export to YAML file
        self.yaml_db = YAMLDatabase(self.anns_collection, self.data_collection)
        self.import_srv = \
            rospy.Service('yaml_import', YAMLImport, self.yaml_db.importFromYAML)
        self.export_srv = \
            rospy.Service('yaml_export', YAMLExport, self.yaml_db.exportToYAML)

        rospy.loginfo("Annotations server : initialized.")


    def getAnnotations(self, request):

        response = GetAnnotationsResponse()
        
        query = {'$and':[]}
        query['$and'].append({'world_id': {'$in': [unique_id.toHexString(request.world_id)]}})
        if len(request.ids) > 0:
            query['$and'].append({'id': {'$in': [unique_id.toHexString(id) for id in request.ids]}})
        if len(request.types) > 0:
            query['$and'].append({'type': {'$in': request.types}})
        if len(request.keywords) > 0:
            query['$and'].append({'keywords': {'$in': request.keywords}})
        if len(request.relationships) > 0:
            query['$and'].append({'relationships': {'$in': [unique_id.toHexString(r) for r in request.relationships]}})

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
#             rospy.logerr("Pulled annotations and associated data don't match (%lu != %lu)",
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
#             rospy.logerr("Pulled annotations and associated data don't match (%lu != %lu)",
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
            
            # Compose metadata: mandatory fields
            metadata = { 'world_id': unique_id.toHexString(annotation.world_id),
                         'id'      : unique_id.toHexString(annotation.id),
                         'name'    : annotation.name,
                         'type'    : annotation.type,
                       }

            # Optional fields; note that both are stored as lists of strings
            if len(annotation.keywords) > 0:
                metadata['keywords'] = annotation.keywords
            if len(annotation.relationships) > 0:
                metadata['relationships'] = [unique_id.toHexString(r) for r in annotation.relationships]

            rospy.logdebug("Saving annotation %s for map %s", annotation.id, annotation.world_id)

            # Insert both annotation and associated data to the appropriate collection
            # TODO: using by now the same metadata for both, while data only need annotation id
            self.anns_collection.remove({'id': {'$in': [unique_id.toHexString(annotation.id)]}})
            self.anns_collection.insert(annotation, metadata)
            self.data_collection.remove({'id': {'$in': [unique_id.toHexString(annotation.id)]}})
            self.data_collection.insert(data, metadata)

        rospy.loginfo("%lu annotations saved", len(request.annotations))
        response.result = True
        return response

    def setKeyword(self, request):
        response = SetKeywordResponse()
        annot_id, metadata = self.getMetadata(request.id)
        
        if metadata is None:
            response.message = 'Annotation not found' 
            response.result = False
            return response

        if request.action == SetKeywordRequest.ADD:
            return self.addElement(annot_id, request.keyword, metadata, 'keywords', response)
        elif request.action == SetKeywordRequest.DEL:
            return self.delElement(annot_id, request.keyword, metadata, 'keywords', response)
        else:
            # Sanity check against crazy service clients
            rospy.logerr('Invalid action %d', request.action)
            response.message = 'Invalid action' 
            response.result = False
            return response


    def setRelationship(self, request):
        response = SetRelationshipResponse()
        annot_id, metadata = self.getMetadata(request.id)
        relat_id = unique_id.toHexString(request.relationship)
        
        if metadata is None:
            response.message = 'Annotation not found' 
            response.result = False
            return response

        if request.action == SetRelationshipRequest.ADD:
            return self.addElement(annot_id, relat_id, metadata, 'relationships', response)
        elif request.action == SetRelationshipRequest.DEL:
            return self.delElement(annot_id, relat_id, metadata, 'relationships', response)
        else:
            # Sanity check against crazy service clients
            rospy.logerr('Invalid action %d', request.action)
            response.message = 'Invalid action' 
            response.result = False
            return response


    def addElement(self, annot_id, element, metadata, md_field, response):

        # Look on metadata for md_field field, for the target element
        field = metadata.get(md_field)

        # Add the new element to metadata field 
        if field is None:
            field = []
        if element not in field:
            field.append(element)
            metadata[md_field] = field
            self.anns_collection.update(metadata)
            rospy.loginfo("%s added to %s for annotation %s", element, md_field, annot_id)
        else:
            # Already present; nothing to do (not an error)
            rospy.loginfo("%s already set on %s for annotation %s", element, md_field, annot_id)

        response.result = True
        return response

    def delElement(self, annot_id, element, metadata, md_field, response):
        # Look on metadata for md_field field, for the target element
        field = metadata.get(md_field)

        # Remove the element from metadata field, if already present 
        if field is None or element not in field:
            # Requested element not found; nothing to do, but we answer as error
            rospy.logerr("%s not found on %s for annotation %s", element, md_field, annot_id)
            response.message = element + ' not found'
            response.result = False
            return response
        else:
            # Found the requested element; remove from metadata
            rospy.loginfo("%s deleted on %s for annotation %s", element, md_field, annot_id)
            field.remove(element)
            if len(field) == 0:
                del metadata[md_field]
            else:
                metadata[md_field] = field
            self.anns_collection.update(metadata)

        # No error so return success
        response.result = True
        return response


    def getMetadata(self, uuid):
        # Get metadata for the given annotation id
        annot_id = unique_id.toHexString(uuid)
        matching_anns = self.anns_collection.query({'id': {'$in': [annot_id]}})

        try:
            return annot_id, matching_anns.next()[1]
        except StopIteration:
            rospy.logwarn("Annotation %s not found", annot_id)
            return annot_id, None


if __name__ == "__main__":
    rospy.init_node('annotations_server')

    AnnotationsServer()
  
    rospy.spin()
