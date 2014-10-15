/*
 * worlds_list.cpp
 *
 *  Created on: Oct 11, 2014
 *      Author: Jorge Santos
 */

#include <ros/ros.h>

#include <QTreeWidget>
#include <QTreeWidgetItem>

//#include <yocs_math_toolkit/common.hpp>
//#include <world_canvas_client_cpp/unique_id.hpp>

#include <world_canvas_msgs/ListWorlds.h>


#include "worlds_list.hpp"

namespace wcf
{

WorldsList::WorldsList(const std::string& srv_namespace, QTreeWidget* tree_widget)
  : WorldCollection(srv_namespace), tree_widget_(tree_widget), current_world_(-1)
{



  ROS_INFO("World collection ready!");
  updateWidget();
}

void WorldsList::setCurrent(int index)
{
  if (current_world_ != index)
  {
    annotations_.reset(new AnnotationsList(world_names[index]));
    current_world_ = index;
    updateWidget();
  }
}

void WorldsList::updateWidget()
{
  tree_widget_->clear();

  for (unsigned int i = 0; i < world_names.size(); i++)
  {
    QTreeWidgetItem *item = new QTreeWidgetItem();
    item->setText(0, world_names[i].c_str());
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

    tree_widget_->addTopLevelItem(item);

    if (annotations_ && i == current_world_)
    {
      annotations_->updateWidget(item);
      item->setExpanded(true);
    }
  }
}

} // namespace wcf
