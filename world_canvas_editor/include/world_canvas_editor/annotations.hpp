/*
 * annotations.hpp
 *
 *  Created on: Sep 21, 2014
 *      Author: Jorge Santos
 */

#include <ros/ros.h>

class QTreeWidgetItem;

#include <world_canvas_client_cpp/annotation_collection.hpp>


namespace wcf
{

class AnnotationsList : public AnnotationCollection
{
public:
  AnnotationsList(const std::string& world);
  ~AnnotationsList();

  bool add(const world_canvas_msgs::Annotation& annotation,
           const world_canvas_msgs::AnnotationData& annot_data);
  bool update(const world_canvas_msgs::Annotation& annotation,
              const world_canvas_msgs::AnnotationData& annot_data);
  bool remove(const world_canvas_msgs::Annotation& annotation);

  const world_canvas_msgs::Annotation& at(unsigned int index);

  void updateWidget(QTreeWidgetItem* tree_item);

private:
  QTreeWidgetItem* tree_item_;
};

} // namespace wcf
