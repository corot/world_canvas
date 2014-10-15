/*
 * worlds_list.hpp
 *
 *  Created on: Oct 11, 2014
 *      Author: Jorge Santos
 */

#include <ros/ros.h>

#include <world_canvas_client_cpp/world_collection.hpp>

#include "annotations.hpp"


class QTreeWidget;


namespace wcf
{

class WorldsList : public WorldCollection
{
public:
  WorldsList(const std::string& srv_namespace, QTreeWidget* tree_widget);

  void updateWidget();
  void setCurrent(int index);
  int  getCurrent() { return current_world_; }
  std::string& getName(int index) { return world_names[index]; }

  boost::shared_ptr<AnnotationsList> annotations_;

private:
  int        current_world_;
  QTreeWidget* tree_widget_;
};

} // namespace wcf
