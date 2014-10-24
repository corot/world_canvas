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


#include "annotations.hpp"

namespace wcf
{

AnnotationsList::AnnotationsList(const std::string& world)
               : AnnotationCollection(world), tree_item_(NULL)
{
  this->load();
  this->loadData();
  ROS_INFO("Annotation collection ready!");

  // Publish annotations' visual markers
  this->publishMarkers("annotation_markers");
}

AnnotationsList::~AnnotationsList()
{
}

void AnnotationsList::updateWidget(QTreeWidgetItem* tree_item)
{
  tree_item_ = tree_item;
  tree_item_->takeChildren();

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

    tree_item_->addChild(item);
  }
}

bool AnnotationsList::add(const world_canvas_msgs::Annotation& annotation,
                          const world_canvas_msgs::AnnotationData& annot_data)
{
  if (AnnotationCollection::add(annotation, annot_data) == true)
  {
    // Reflect changes on the tree widget
    assert(tree_item_);
    this->updateWidget(tree_item_);

    return true;
  }

  return false;
}

bool AnnotationsList::update(const world_canvas_msgs::Annotation& annotation,
                             const world_canvas_msgs::AnnotationData& annot_data)
{
  if (AnnotationCollection::add(annotation, annot_data) == true)
  {
    // Reflect changes on the tree widget
    assert(tree_item_);
    this->updateWidget(tree_item_);

    return true;
  }

  return false;
}

bool AnnotationsList::remove(const uuid_msgs::UniqueID& id)
{
  if (AnnotationCollection::remove(id) == true)
  {
    // Reflect changes on the tree widget
    assert(tree_item_);
    this->updateWidget(tree_item_);

    return true;
  }

  return false;
}

const world_canvas_msgs::Annotation& AnnotationsList::at(unsigned int index)
{
  if (index >= this->annotations.size())
    throw ros::Exception("Annotation index out of bounds");

  return this->annotations[index];
}

} // namespace wcf
