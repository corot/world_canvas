/*
 * annotations.hpp
 *
 *  Created on: Sep 21, 2014
 *      Author: Jorge Santos
 */

#include <ros/ros.h>

class QTreeWidget;

#include <world_canvas_client_cpp/annotation_collection.hpp>


class AnnotationsList : public AnnotationCollection
{
public:
  AnnotationsList(const std::string& world, QTreeWidget* treeWidget);

private:
  QTreeWidget* treeWidget_;
  // Prepare the annotation collection
  //FilterCriteria filter(world_name, uuids, names, types, keywords, relationships);
  ////AnnotationCollection ac_;
  void updateWidget();
};
