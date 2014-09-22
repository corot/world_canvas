/*
 * annotations.cpp
 *
 *  Created on: Sep 21, 2014
 *      Author: Jorge Santos
 */

#include <ros/ros.h>

#include <QTreeWidget>
#include <QTreeWidgetItem>

#include <yocs_math_toolkit/common.hpp>
#include <world_canvas_client_cpp/unique_id.h>

#include "annotations.hpp"


AnnotationsList::AnnotationsList(const std::string& world, QTreeWidget* treeWidget)
               : AnnotationCollection(world), treeWidget_(treeWidget)
{
//  // Parameters
//  ros::NodeHandle nh("~");

  std::string world_name;
  std::string topic_name;
  std::string topic_type;
  std::string default_tn("annotations");
  std::string default_tt;
  std::string default_wn("INVALID_WORLD");
  bool        pub_as_list;
  std::vector<std::string> uuids;
  std::vector<std::string> names;
  std::vector<std::string> types;
  std::vector<std::string> keywords;
  std::vector<std::string> relationships;

//  nh.param("world",         world_name, default_wn);
//  nh.param("topic_name",    topic_name, default_tn);
//  nh.param("topic_type",    topic_type, default_tt);
//  nh.param("pub_as_list",   pub_as_list, false);
//  nh.param("uuids",         uuids, uuids);
//  nh.param("names",         names, names);
//  nh.param("types",         types, types);
//  nh.param("keywords",      keywords, keywords);
//  nh.param("relationships", relationships, relationships);

  // Prepare the annotation collection
//  FilterCriteria filter(world_name, uuids, names, types, keywords, relationships);
//  this->filterBy(filter);
  this->load();
  this->loadData();
  this->updateWidget();
  ROS_INFO("Annotation collection ready!");

  // Publish annotations' visual markers on client side
  this->publishMarkers("annotation_markers");

  // Request server to publish the annotations
//  ac_.publish(topic_name, true, pub_as_list, topic_type);
//
//  std::vector<yocs_msgs::Wall> walls;
//  std::vector<yocs_msgs::Column> columns;
//  std::vector<nav_msgs::OccupancyGrid> maps;
//
//  ROS_INFO("Done!  got %u walls, %u columns and %u maps",
//           ac.getData(walls), ac.getData(columns), ac.getData(maps));
//  ROS_INFO("(for confirmation    %lu   %lu   %lu)", walls.size(), columns.size(), maps.size());
//  ros::Publisher wp = nh.advertise<yocs_msgs::Wall> ("walls_on_client", 1, true);
//  ros::Publisher cp = nh.advertise<yocs_msgs::Column> ("columns_on_client", 1, true);
//  ros::Publisher mp = nh.advertise<nav_msgs::OccupancyGrid> ("maps_on_client", 1, true);
//
//  for (unsigned int i = 0; i < walls.size(); i++)
//    wp.publish(walls[i]);
//  for (unsigned int i = 0; i < columns.size(); i++)
//    cp.publish(columns[i]);
//  for (unsigned int i = 0; i < maps.size(); i++)
//    mp.publish(maps[i]);
}

void AnnotationsList::updateWidget()
{
  treeWidget_->clear();

  for (unsigned int i = 0; i < this->annotations.size(); i++)
  {
    QTreeWidgetItem *item = new QTreeWidgetItem();
    item->setText(0, annotations[i].name.c_str());
    item->setText(1, annotations[i].type.c_str());
    item->setText(2, mtk::pose2str(annotations[i].pose.pose.pose));
//    item->setText(1, annotations[i].name.c_str());
//
//    marker.id     = i;
//    marker.header = annotations[i].pose.header;
//    marker.type   = annotations[i].shape;
//    marker.ns     = annotations[i].type;
//    marker.action = visualization_msgs::Marker::ADD;
//    marker.pose   = annotations[i].pose.pose.pose;
//    marker.scale  = annotations[i].size;
//    marker.color  = annotations[i].color;

    treeWidget_->addTopLevelItem(item);
  }
}

bool AnnotationsList::add(const world_canvas_msgs::Annotation& annotation,
                          const world_canvas_msgs::AnnotationData& annot_data)
{
  if (annotation.data_id.uuid != annot_data.id.uuid)
  {
    ROS_ERROR("Incoherent annotation and data uuids '%s' != '%s'",
              unique_id::toHexString(annotation.id).c_str(), unique_id::toHexString(annot_data.id).c_str());
    return false;
  }

  for (unsigned int i = 0; i < this->annotations.size(); i++)
  {
    if (this->annotations[i].id.uuid == annotation.id.uuid)
    {
      ROS_ERROR("Duplicated annotation with uuid '%s'", unique_id::toHexString(annotation.id).c_str());
      return false;
    }
  }

  for (unsigned int i = 0; i < this->annots_data.size(); i++)
  {
    if (this->annots_data[i].id.uuid == annot_data.id.uuid)
    {
      ROS_ERROR("Duplicated annotation data with uuid '%s'", unique_id::toHexString(annot_data.id).c_str());
      return false;
    }
  }

  this->annotations.push_back(annotation);
  this->annots_data.push_back(annot_data);

  return true;
}
