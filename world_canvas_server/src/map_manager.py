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
# 
# Pythonized version of map_store/map_manager to provide the same services
# within the WCF to keep backward compatibility while client software adapt
# to our new interface.
# Includes also map_store/map_saver functionality but improved according to
# this issue: https://github.com/ros-planning/map_store/issues/4
# 
# behavior:
#  - sets up connection to warehouse
#  - tells warehouse to publish latest map of any session (or default map?  or nothing?)
#  - spins, handling service calls
#  - listens on "map" topic.  On each map:
#     - id_of_most_recent_map = Collection.publish(map, {session ID, map name})
# service calls:
#  - list_maps() returns list of map metadata: {id, name, timestamp, maybe thumbnail}
#    - query for all maps.
#  - delete_map(map id) returns void
#    - Deletes the given map
#  - rename_map(map id, name) returns void
#    - renames a given map
#  - save_map(map name) returns void
#    - save the map returned by dynamic_map as map name
#  - publish_map(map id) returns void
#    - queries warehouse for map of given id
#    - publishes the map on /map topic
#    - sets dynamic map up to load it
#  - dynamic_map() returns nav_msgs/OccupancyGrid
#    - returns the dynamic map

import rospy
import unique_id

import warehouse_ros as wr

from nav_msgs.msg import *
from nav_msgs.srv import *

from world_canvas_msgs.msg import *
from world_canvas_msgs.srv import *
from world_canvas_utils.serialization import *


class MapManager:
    
    ##########################################################################
    # Initialization
    ##########################################################################

    def __init__(self):
        # Set up map collection
        self.map_collection = wr.MessageCollection('world_canvas', 'maps', OccupancyGrid)
        self.map_collection.ensure_index('uuid', unique=True)
        
        # Set up map management services
        self.list_maps_srv   = rospy.Service('list_maps',   ListMaps,   self.listMaps)
        self.publish_map_srv = rospy.Service('publish_map', PublishMap, self.publishMap)
        self.delete_map_srv  = rospy.Service('delete_map',  DeleteMap,  self.deleteMap)
        self.rename_map_srv  = rospy.Service('rename_map',  RenameMap,  self.renameMap)
        self.save_map_srv    = rospy.Service('save_map',    SaveMap,    self.saveMap)
        self.dynamic_map_srv = rospy.Service('dynamic_map', GetMap,     self.dynamicMap)

        # Set up map subscriber for saving under-construction maps
        self.map_subscriber = rospy.Subscriber('map', OccupancyGrid, self.onMapReceived, queue_size=1)

        # Use the current ROS time in seconds as the session id for saved maps
        self.session_id = str(rospy.get_time())

        # Set up map publisher and publish the last used map, if any
        self.map_publisher = rospy.Publisher('map', OccupancyGrid, latch=True, queue_size=1)

        try:
            self.last_map = rospy.get_param('last_map_id')
            map = self.lookupMap(self.last_map)
            if map is None:
                rospy.logerr("Invalid last_map_id: %s" % str(self.last_map))
            else:
                self.map_publisher.publish(map)
        except KeyError:
            self.last_map = None

        rospy.loginfo("Map manager : initialized.")


    ##########################################################################
    # Services callbacks
    ##########################################################################

    def listMaps(self, request):
        rospy.logdebug("list_maps service call");
        
        response = ListMapsResponse()
        
        all_maps = self.map_collection.query({}, metadata_only=True, sort_by='creation_time', ascending=False)
        
        # Loop over all maps metadata to get the first of each session.
        while True:
            try:
                map_md = all_maps.next()
                rospy.logdebug("Add map to result list: %s" % map_md)

                # Add the map info to our result list.
                new_entry = MapListEntry()
                new_entry.name = map_md['name']
                new_entry.date = map_md['creation_time']
                new_entry.session_id = map_md['session_id']
                new_entry.map_id = map_md['uuid']
                
                response.map_list.append(new_entry)
            except StopIteration:
                break
        
        rospy.logdebug("list_maps : service call done")
        return response


    def lookupMap(self, uuid):
        rospy.logdebug("Load map %s" % uuid)
        matching_maps = self.map_collection.query({'uuid': {'$in': [uuid]}})
        try:
            return matching_maps.next()[0]
        except StopIteration:
            rospy.logerr("publish_map : no map found for uuid %s" % uuid)
            return None


    def publishMap(self, request):
        rospy.logdebug("publish_map : service call")
        response = PublishMapResponse()

        map = self.lookupMap(request.map_id)
        if map is None:
            rospy.logerr("Invalid map id: %s" % str(request.map_id))
            return None
        else:
            self.last_map = request.map_id
            rospy.set_param('last_map_id', self.last_map)
            self.map_publisher.publish(map)

        return response
        
    def deleteMap(self, request):
        rospy.logdebug("Service call : delete map %s" % request.map_id)
        response = DeleteMapResponse()
        
        if rospy.has_param('last_map_id') and rospy.get_param('last_map_id') == request.map_id:
            rospy.delete_param('last_map_id')
        if self.last_map == request.map_id: 
            self.last_map = None

        if self.map_collection.remove({'uuid': {'$in': [request.map_id]}}) == 0:
            return None

        return response

    def renameMap(self, request):
        rospy.logdebug("Service call : rename map %s as %s" % (request.map_id, request.new_name))
        response = RenameMapResponse()

        matching_maps = self.map_collection.query({'uuid': {'$in': [request.map_id]}}, True)

        try:
            map_metadata = matching_maps.next()
        except StopIteration:
            rospy.logerr("Map %s not found" % request.map_id)
            return None

        map_metadata['name'] = request.new_name
        self.map_collection.update(map_metadata)
        return response  

    def dynamicMap(self, request):
        rospy.logdebug("Service call : get last map (%s)" % self.last_map)
        response = GetMapResponse()

        if self.last_map is None:
            return None

        map = self.lookupMap(self.last_map)
        if map is None:
            return None

        response.map = map;
        return response

    def onMapReceived(self, map_msg):
        rospy.logdebug("received map")
        self.rec_map = map_msg
#         std::string uuid_string = uuidGenerate();
#         mr::Metadata metadata
#           = mr::Metadata("uuid", uuid_string,
#                  "session_id", session_id);
#         
#         map_collection->insert(*map_msg, metadata);
#         
#         rospy.logdebug("saved map");
        
    def saveMap(self, request):
        rospy.logdebug("Service call : save current map as %s" % request.map_name)
        response = SaveMapResponse()

#         nav_msgs::GetMap srv;
#         if (!dynamic_map_service_client.call(srv)) {
#           rospy.logerr("Dynamic map getter service call failed");
#           return false;
#         }
        if hasattr(self, 'rec_map'):
            metadata = {'uuid': str(unique_id.fromRandom()),
                        'session_id': self.session_id,
                        'name': request.map_name
                       }

            rospy.logdebug("Saved map %d by %d @ %f as %s" % (self.rec_map.info.width, self.rec_map.info.height,
                           self.rec_map.info.resolution, request.map_name))
            try:
                self.map_collection.insert(self.rec_map, metadata, safe=True)
            except Exception as e:
                # Assume collection.insert raised this, as we set safe=True (typically a DuplicateKeyError)
                # This should not happen, as we have generated an uuid; but just in case I copy-paste the code...
                rospy.logerr("Insert annotation failed: %s" % str(e))
                return None
                
            return response
        else:
            rospy.logerr("No map received so far! Nothing to save")
            return None
