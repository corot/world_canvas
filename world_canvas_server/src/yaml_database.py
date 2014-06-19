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
import os
import re
import yaml
import uuid
import unique_id
import cPickle as pickle
import warehouse_ros as wr

from world_canvas_msgs.msg import *
from world_canvas_msgs.srv import *


class YAMLDatabase:
    
    ##########################################################################
    # Initialization
    ##########################################################################

    def __init__(self, anns_collection, data_collection):
        # Set up collections
        self.anns_collection = anns_collection
        self.data_collection = data_collection

    
    ##########################################################################
    # Services callbacks
    ##########################################################################

    def importFromYAML(self, request):

        response = YAMLImportResponse()
        
        if not os.path.isfile(request.filename):
            return self.serviceError(response, "File does not exist: %s" % (request.filename))

        try:
            with open(request.filename, 'r') as f:
                # load all documents
                yaml_data = yaml.load(f)
        except yaml.YAMLError as e:
            return self.serviceError(response, "Invalid YAML file: %s" % (str(e)))
    
        # Clear existing database content  TODO: flag to choose whether keep content
        self.anns_collection.remove({})
        self.data_collection.remove({})

        # Parse file: a database is composed of a list of elements, each one containing a list of annotations plus
        # their referenced data. TODO: Can change according to issue https://github.com/corot/world_canvas/issues/9
        for t in yaml_data:
            # Annotations: a list of one or more annotations
            try:
                anns_list = t['annotations']
            except KeyError as e:
                return self.serviceError(response, "Invalid database file format: %s field not found" % str(e))

            if len(anns_list) == 0:
                # Coherence check: there must be at least one annotation for database entry
                return self.serviceError(response, "Invalid database file format: " \
                                         "there must be at least one annotation for database entry")

            for a in anns_list:
                annotation = Annotation()
                try:
                    genpy.message.fill_message_args(annotation, a)

                    if 'prev_data_id' in locals() and prev_data_id != annotation.data_id.uuid:
                        # Coherence check: all data_id fields must be equal between them and to data id
                        return self.serviceError(response, "Invalid database file format: " \
                                                 "data ids must be equal for annotations referencing the same data")

                    prev_data_id = annotation.data_id.uuid
                    
                    # Forced conversion because UUID expects a string of 16 bytes, not a list
                    annotation.world_id.uuid = ''.join(chr(x) for x in annotation.world_id.uuid)
                    annotation.data_id.uuid = ''.join(chr(x) for x in annotation.data_id.uuid)
                    annotation.id.uuid = ''.join(chr(x) for x in annotation.id.uuid)
                    for r in annotation.relationships:
                        r.uuid = ''.join(chr(x) for x in r.uuid)
        
                    # Compose metadata: mandatory fields
                    metadata = { 'world_id': unique_id.toHexString(annotation.world_id),
                                 'data_id' : unique_id.toHexString(annotation.data_id),
                                 'id'      : unique_id.toHexString(annotation.id),
                                 'name'    : annotation.name,
                                 'type'    : annotation.type,
                               }
        
                    # Optional fields; note that both are stored as lists of strings
                    if len(annotation.keywords) > 0:
                        metadata['keywords'] = annotation.keywords
                    if len(annotation.relationships) > 0:
                        metadata['relationships'] = [unique_id.toHexString(r) for r in annotation.relationships]
                        
                    rospy.logdebug("Saving annotation %s for map %s" % (annotation.id, annotation.world_id))
            
        #                     self.anns_collection.remove({'id': {'$in': [unique_id.toHexString(annotation.id)]}})
                    
                    self.anns_collection.insert(annotation, metadata)
                except (genpy.MessageException, genpy.message.SerializationError) as e:
                    return self.serviceError(response, "Invalid annotation msg format: %s" % str(e))

            # Annotation data, of message type annotation.type
            msg_class = roslib.message.get_message_class(annotation.type)
            if msg_class is None:
                # annotation.type doesn't contain a known message type; we cannot insert on database
                return self.serviceError(response, "Unknown message type: %s" % annotation.type)
            
            data = msg_class()

            # Data metadata: just the object id, as all querying is done over the annotations
            data_metadata = { 'id' : unique_id.toHexString(annotation.data_id) }

            try:
                genpy.message.fill_message_args(data, t['data'])
                data_msg = AnnotationData()
                data_msg.id = annotation.data_id
                data_msg.data = pickle.dumps(data)
                self.data_collection.insert(data_msg, data_metadata)
            except (genpy.MessageException, genpy.message.SerializationError) as e:
                # TODO: here I would have an incoherence in db: annotations without data;
                # do mongo has rollback? do it manually? or just clear database content?
                return self.serviceError(response, "Invalid %s msg format: %s" % (annotation.type, str(e)))
            except KeyError as e:
                return self.serviceError(response, "Invalid database file format: %s field not found" % str(e))

#               self.data_collection.remove({'id': {'$in': [unique_id.toHexString(annotation.id)]}})

            del prev_data_id # clear this so it doen't interfere with next element validation

        return self.serviceSuccess(response, "%lu annotations imported on database" % len(yaml_data))


    def exportToYAML(self, request):
        response = YAMLExportResponse()

        # Query for all annotations in database, shorted by data id, so we can export packed with its referenced data
        matching_anns = self.anns_collection.query({}, sort_by='data_id')

        try:
            with open(request.filename, 'w') as f:
                entries = []
                annotations = []
                last = False

                # Loop over annotations retrieved from database
                while True:
                    try:
                        annotation, metadata = matching_anns.next()
                    except StopIteration:
                        last = True

                    if not last and (len(annotations) == 0 or annotation.data_id == annotations[-1].data_id):
                        # We pack annotations with the same data_id
                        annotations.append(annotation)
                    elif len(annotations) > 0:
                        # When the next annotation have a different data_id, or is the
                        # last one, it's time to append a new entry to the output file:
                        #  a) retrieve data for current data_id
                        data_id_str = unique_id.toHexString(annotations[-1].data_id)
                        matching_data = self.data_collection.query({'id': {'$in': [data_id_str]}})
                        try:
                            d = matching_data.next()[0]
                        except StopIteration:
                            return self.serviceError(response, "Export to file failed: " \
                                                     "No data found with id %s" % data_id_str)

                        #  b) pack together with their referencing annotations in a dictionary
                        entry = dict(
                            annotations = [yaml.load(genpy.message.strify_message(a)) for a in annotations],
                            data = yaml.load(genpy.message.strify_message(pickle.loads(d.data)))
                        )
                        entries.append(entry)

                        if last:
                            break;
                        else:
                            # No the last annotation; note that it's still not stored,
                            # in the entries list so add to list for the next iteration
                            annotations = [annotation]


                if len(entries) == 0:
                    # we don't consider this an error
                    return self.serviceSuccess(response, "Database is empty!; nothing to export")
                else:
                    # default_flow_style = False writes lists with an element per-line, (with -),
                    # what makes the output very long and not very readable. Method flowStyleLists
                    # makes uuids and covariances list flow-styled, i.e. [x, y, z, ...]
                    dump = yaml.dump(entries, default_flow_style = False)
                    dump = self.flowStyleLists(dump)
                    
                    # Add a decimal point to exponential formated floats; if not, get loaded as strings,
                    # due to a bug in pyyaml. See this ticket for details: http://pyyaml.org/ticket/359
                    dump = self.removeExp(dump)
                    f.write(dump)
                    return self.serviceSuccess(response, "%lu annotations exported from database" % len(entries))
        except Exception as e:
            return self.serviceError(response, "Export to file failed: %s" % (str(e)))

    
    ##########################################################################
    # Auxiliary methods
    ##########################################################################

    def serviceSuccess(self, response, message = None):
        if message is not None:
            rospy.loginfo(message)
        response.result = True
        return response

    def serviceError(self, response, message):
        rospy.logerr(message)
        response.message = message
        response.result = False
        return response

    def removeExp(self, target):
        # Compose and compile regex to match exponential notation floats without floating point
        regex = '([-+]?\d+[eE][-+]?\d+)'
        offset = 0
        comp_re = re.compile(regex)
        for match in comp_re.finditer(target):
            # Replace matches with a the same float plus a .0 to make pyyaml parser happy
            fp_nb = match.group(1)
            fp_nb = re.sub('[eE]', '.0e', fp_nb)
            # print target[match.start(1) + offset:match.end(1) + offset]
            target = target[:match.start(1) + offset] + fp_nb + target[match.end(1) + offset:]
            # print target[match.start(1) + offset:match.end(1) + offset+2]
            offset += 2

        return target
    
    def flowStyleLists(self, target):
        # Compose and compile regex to match uuids: lists of 16 integers
        regex = 'uuid: *\n'
        for i in range(0,16):
            regex += ' *- +(\d+)\n'
        comp_re = re.compile(regex)
        while True:
            match = comp_re.search(target)
            if match is None:
                break;
            
            # Replace matches with a flow-stile version; group(n) returns the nth integer
            flow_list = 'uuid: ['
            for i in range(1, 16):
                flow_list += match.group(i) + ', '
            flow_list += match.group(16) + ']\n'
        
            target = comp_re.sub(flow_list, target, 1)

        # Compose and compile regex to match covariances: lists of 36 floats, possibly on exponential notation
        regex = 'covariance: *\n'
        for i in range(0,36):
            regex += ' *- +([-+]?\d*\.?\d+|\d+[eE][-+]?\d+)\n'
        comp_re = re.compile(regex)
        while True:
            match = comp_re.search(target)
            if match is None:
                break;
            
            # Replace matches with a flow-stile version; group(n) returns the nth float
            flow_list = 'covariance: ['
            for i in range(1, 36):
                flow_list += match.group(i) + ', '
            flow_list += match.group(36) + ']\n'
        
            target = comp_re.sub(flow_list, target, 1)

        return target