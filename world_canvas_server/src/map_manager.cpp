/*
 * Copyright (c) 2013, Yujin Robot.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Based on map_store, but adapted to publish/store semantic information (annotations)
behavior:
 - sets up connection to warehouse
 - tells warehouse to publish latest annotations of any session (this is commented by now; can be misleading)
 - spins, handling service calls

service calls:
 - publish_map(map uuid) returns true if annotations were found for the given map
   - queries warehouse for annotations associated to the given map
   - publishes the annotations on markers, tables, columns, walls topics
   - publishes visualization markers for all the annotations
   - sets map uuid as the current map, so we can update published annotations if needed
 - rename_map(map uuid, new name) returns void
   - renames the associated map identified by map_uuid on annotations database
 - delete_map(map uuid) returns true if annotations were found for the given map
   - deletes the annotations associated to the given map
   - if current map is set, calls publish_annotations to reflect changes
 - save_map(map uuid, map name, session id) returns error message if any
   - saves currently published annotations as associated to the given map
   - if current map is set, calls publish_annotations to reflect changes

 NOT IMPLEMENTED, and not useful by now
 - list_maps() returns list of map metadata: {id, name, timestamp, maybe thumbnail}
   - query for all annotations.
 - dynamic_map() returns nav_msgs/OccupancyGrid
   - returns the dynamic map
 */

#include <mongo_ros/message_collection.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <world_canvas_msgs/Annotation.h>
#include <world_canvas_msgs/SemanticMap.h>
#include <world_canvas_msgs/ListMaps.h>
#include <world_canvas_msgs/PublishMap.h>
#include <world_canvas_msgs/DeleteMap.h>
#include <world_canvas_msgs/RenameMap.h>
#include <world_canvas_msgs/SaveMap.h>

#include <string>
#include <sstream>
#include <exception>
namespace mr=mongo_ros;

mr::MessageCollection<world_canvas_msgs::SemanticMap> *maps_collection;

ros::Publisher map_pub;
ros::Publisher markers_pub;

std::string last_map;
std::string pub_map_id;

visualization_msgs::MarkerArray markers_array;

world_canvas_msgs::SemanticMap map_msg;

typedef std::vector<mr::MessageWithMetadata<world_canvas_msgs::SemanticMap>::ConstPtr> MapsVector;


void onMapReceived(const world_canvas_msgs::SemanticMap::ConstPtr& msg)
{
  map_msg = *msg;
}

void clearMarkers()
{
  for (int i = 0; i < markers_array.markers.size(); ++i)
  {
    markers_array.markers[i].action = visualization_msgs::Marker::DELETE;
  }

  if (markers_array.markers.size() > 0)
  {
    markers_pub.publish(markers_array);
    markers_array.markers.clear();
  }
}

visualization_msgs::Marker makeMarker(int id, const world_canvas_msgs::Annotation& ann)
{
  std::stringstream name; name << ann.type << '/' << ann.name;

  visualization_msgs::Marker marker;
  marker.header.frame_id = ann.pose.header.frame_id;
  marker.header.stamp = ros::Time::now();
  marker.scale = ann.size;
  marker.color = ann.color;
  marker.ns = name.str();
  marker.id = id;
  marker.pose = ann.pose.pose.pose;
  marker.type = ann.shape;
  marker.action = visualization_msgs::Marker::ADD;

  // Make z-coordinate the lowest point of markers, so they appear to lay in the floor if they
  // have zero z. This is wrong for AR markers, but doesn't matter as they are rather small.
  marker.pose.position.z += marker.scale.z/2.0;

  return marker;
}

visualization_msgs::Marker makeLabel(const visualization_msgs::Marker& marker)
{
  visualization_msgs::Marker label = marker;
  label.id = marker.id + 1000000;  // marker id must be unique
  label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  label.pose.position.z = marker.pose.position.z + marker.scale.z/2.0 + 0.1; // just above the visual
  label.text = marker.ns;
  label.scale.x = label.scale.y = label.scale.z = 0.12;
  // label.color.r = label.color.g = label.color.b = 0.0; label.color.a = 1.0; // make solid black

  return label;
}

bool publishMap(world_canvas_msgs::PublishMap::Request &request,
                world_canvas_msgs::PublishMap::Response &response)
{
  ROS_INFO("Publish semantic map '%s'", request.map_uuid.c_str());

//  last_map = request.map_id;
//  ros::NodeHandle nh;
//  nh.setParam("last_map_id", last_map);

  try
  {
    // remove from visualization tools and delete visualization markers
    clearMarkers();

    MapsVector matching_maps =
        maps_collection->pullAllResults(mr::Query("map_uuid", request.map_uuid));

    if (matching_maps.size() == 0)
    {
      ROS_WARN("No semantic map found for id '%s'; we don't consider this an error",
               request.map_uuid.c_str());
      response.found = false;
      return true;  // we don't consider this an error
    }
    else if (matching_maps.size() > 1)
    {
      // Extra sanity checking
      ROS_WARN("More than one (%lu) semantic maps found for id '%s'; we consider only the first one",
               matching_maps.size(), request.map_uuid.c_str());
    }

    // At least one map found; publish it
    response.found = true;

    ROS_INFO("Semantic map fetched containing %lu annotations",
             matching_maps[0]->annotations.size());
    map_pub.publish(world_canvas_msgs::SemanticMapConstPtr(matching_maps[0]));

    // compose and publish visualization markers
    for (int i = 0; i < matching_maps[0]->annotations.size(); ++i)
    {
      world_canvas_msgs::Annotation ann = matching_maps[0]->annotations[i];
      markers_array.markers.push_back(makeMarker(i, ann));
      markers_array.markers.push_back(makeLabel(markers_array.markers.back()));
    }

    if (markers_array.markers.size() > 0)
      markers_pub.publish(markers_array);

    // Keep track of currently published annotations to republish if we receive updated data
    pub_map_id = request.map_uuid;
    return true;
  }
  catch(const std::exception &e) {
    ROS_ERROR("Error during query: %s", e.what());
    return false;
  }

  return true;
}

bool deleteMap(world_canvas_msgs::DeleteMap::Request &request,
	       world_canvas_msgs::DeleteMap::Response &response)
{
//  ros::NodeHandle nh;
//  std::string param;
//  if (nh.getParam("last_map_id", param))
//  {
//    if (param == request.map_id)
//    {
//      nh.deleteParam("last_map_id");
//    }
//  }
//  if (last_map == request.map_id)
//  {
//    last_map = "";
//  }
  ROS_INFO("Delete semantic map '%s'", request.map_uuid.c_str());
  int removed = maps_collection->removeMessages(mr::Query("map_uuid", request.map_uuid));

  if (removed == 0)
  {
    ROS_WARN("No semantic map found with id '%s'; we don't consider this an error", request.map_uuid.c_str());
    response.found = false;
  }
  else
  {
    if (removed > 1)  // extra sanity check
      ROS_WARN("More than one (%d) semantic maps removed for id '%s'", removed, request.map_uuid.c_str());

    // Check if we are removing currently published semantic map to "unpublish" them if so
    if (pub_map_id == request.map_uuid)
    {
      world_canvas_msgs::PublishMap::Request pubReq;
      world_canvas_msgs::PublishMap::Response pubRes;
      pubReq.map_uuid = request.map_uuid;
      if (publishMap(pubReq, pubRes) == false)
        ROS_WARN("Unpublish removed semantic map failed for map '%s'", request.map_uuid.c_str());
    }
  }

  // TODO catch db exception
  return true;
}

bool renameMap(world_canvas_msgs::RenameMap::Request &request,
               world_canvas_msgs::RenameMap::Response &response)
{
  maps_collection->modifyMetadata(mr::Query("map_uuid", request.map_uuid),
                                  mr::Metadata("map_name", request.new_name));
  return true;
}


bool saveMap(world_canvas_msgs::SaveMap::Request &request,
             world_canvas_msgs::SaveMap::Response &response)
{
// TODO verify that the map exists; I need a version of the removed "lookMap"
//  nav_msgs::GetMap srv;
//  if (!dynamic_map_service_client.call(srv)) {
//    ROS_ERROR("Dynamic map getter service call failed");
//    return false;
//  }

  mr::Metadata metadata = mr::Metadata("map_name",   request.map_name,
                                       "map_uuid",   request.map_uuid,
                                       "session_id", request.session_id);

  ROS_INFO("Saving semantic map %s with uuid %s", request.map_name.c_str(), request.map_uuid.c_str());
  try
  {
    maps_collection->removeMessages(mr::Query("map_uuid", request.map_uuid));
    maps_collection->insert(map_msg, metadata);

    ROS_INFO("Semantic map containing %lu annotations saved", map_msg.annotations.size());

    // Check if we are modifying currently published semantic map to republish them if so
    if (pub_map_id == request.map_uuid)
    {
      world_canvas_msgs::PublishMap::Request pubReq;
      world_canvas_msgs::PublishMap::Response pubRes;
      pubReq.map_uuid = request.map_uuid;
      if (publishMap(pubReq, pubRes) == false)
        ROS_WARN("Republish modified semantic map failed for map '%s'", request.map_uuid.c_str());
    }

    return true;
  }
  catch (mongo::DBException& e)
  {
    ROS_ERROR("Error during saving: %s", e.what());
    response.error_msg = e.what();
    return false;
  }
}


int main (int argc, char** argv)
{
  ros::init(argc, argv, "world_canvas_manager");
  ros::NodeHandle nh;

  maps_collection = new mr::MessageCollection<world_canvas_msgs::SemanticMap> ("world_canvas_server", "maps");
  maps_collection -> ensureIndex("map_uuid");

//  if (!nh.getParam("last_map_id", last_map))   TODO I don't think it make sense to pub last semantic map
//  {
//    last_map = "";
//  }

  map_pub = nh.advertise<world_canvas_msgs::SemanticMap> ("semantics_out", 1, true);

  markers_pub = nh.advertise<visualization_msgs::MarkerArray>  ("visual_markers", 1, true);
//  if (last_map != "")
//  {
//    nav_msgs::OccupancyGridConstPtr map;
//    if (lookupMap(last_map, map))
//    {
//      try {
//	map_publisher.publish(map);
//      } catch(...) {
//	ROS_ERROR("Error publishing map");
//      }
//    }
//    else
//    {
//      ROS_ERROR("Invalid last_map_id");
//    }
//  }

  ros::Subscriber map_sub = nh.subscribe("semantics_in", 1, onMapReceived);

  ros::ServiceServer publish_map_srv = nh.advertiseService("publish_map", publishMap);
  ros::ServiceServer delete_map_srv  = nh.advertiseService("delete_map",  deleteMap);
  ros::ServiceServer rename_map_srv  = nh.advertiseService("rename_map",  renameMap);
  ros::ServiceServer save_map_srv    = nh.advertiseService("save_map",    saveMap);

//  NOT IMPLEMENTED, and not useful by now
//  ros::ServiceServer list_map_srv    = nh.advertiseService("list_maps",    listMap);
//  ros::ServiceServer dynamic_map_srv = nh.advertiseService("dynamic_map", dynamicMap);

  ROS_DEBUG("Semantic maps manager running");

  ros::spin();

  delete maps_collection;

  return 0;
}
