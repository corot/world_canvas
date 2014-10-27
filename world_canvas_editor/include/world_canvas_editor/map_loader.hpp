/*
 * map_loader.hpp.
 * This file is a reduced version of the main.cpp from map_server, authored by Brian Gerkey.
 *
 *  Created on: Oct 27, 2014
 *      Author: Jorge Santos
 */

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>

#include "ros/ros.h"
#include "ros/console.h"
#include "map_server/image_loader.h"
#include "yaml-cpp/yaml.h"

// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}

class MapLoader
{
public:
  static void load(const std::string& fname, nav_msgs::OccupancyGrid& map)
  {
    std::string mapfname = "";
    double origin[3];
    int negate;
    double res, occ_th, free_th;
    bool trinary = true;
    std::string frame_id;
    ros::NodeHandle private_nh("~");
    private_nh.param("frame_id", frame_id, std::string("map"));

    // The document loading process changed in yaml-cpp 0.5.
    std::ifstream fin(fname.c_str());
    if (fin.fail()) {
      throw ros::Exception("Map loader could not open file " + fname);
    }
    YAML::Node doc = YAML::Load(fin);

    try {
      doc["resolution"] >> res;
    } catch (YAML::InvalidScalar&) {
      throw ros::Exception("The map does not contain a resolution tag or it is invalid.");
    }
    try {
      doc["negate"] >> negate;
    } catch (YAML::InvalidScalar&) {
      throw ros::Exception("The map does not contain a negate tag or it is invalid.");
    }
    try {
      doc["occupied_thresh"] >> occ_th;
    } catch (YAML::InvalidScalar&) {
      throw ros::Exception("The map does not contain an occupied_thresh tag or it is invalid.");
    }
    try {
      doc["free_thresh"] >> free_th;
    } catch (YAML::InvalidScalar&) {
      throw ros::Exception("The map does not contain a free_thresh tag or it is invalid.");
    }
    try {
      doc["trinary"] >> trinary;
    } catch (YAML::Exception&) {
      ROS_DEBUG("The map does not contain a trinary tag or it is invalid... assuming true");
      trinary = true;
    }
    try {
      doc["origin"][0] >> origin[0];
      doc["origin"][1] >> origin[1];
      doc["origin"][2] >> origin[2];
    } catch (YAML::InvalidScalar&) {
      throw ros::Exception("The map does not contain an origin tag or it is invalid.");
    }
    try {
      doc["image"] >> mapfname;
      // TODO: make this path-handling more robust
      if(mapfname.size() == 0)
      {
        throw ros::Exception("The image tag cannot be an empty string.");
      }
      if(mapfname[0] != '/')
      {
        // dirname can modify what you pass it
        char* fname_copy = strdup(fname.c_str());
        mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
        free(fname_copy);
      }
    } catch (YAML::InvalidScalar&) {
      throw ros::Exception("The map does not contain an image tag or it is invalid.");
    }

    ROS_INFO("Loading map from image \"%s\"", mapfname.c_str());
    nav_msgs::GetMap::Response map_resp_;
    map_server::loadMapFromFile(&map_resp_, mapfname.c_str(), res, negate, occ_th, free_th, origin, trinary);
    map_resp_.map.info.map_load_time = ros::Time::now();
    map_resp_.map.header.frame_id = frame_id;
    map_resp_.map.header.stamp = ros::Time::now();
    ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
             map_resp_.map.info.width,
             map_resp_.map.info.height,
             map_resp_.map.info.resolution);
    map = map_resp_.map;
  }
};
