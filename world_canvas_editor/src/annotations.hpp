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

  bool del(const uuid_msgs::UniqueID& id);
  bool save();
  bool check();

  const world_canvas_msgs::Annotation& at(unsigned int index);
  const world_canvas_msgs::AnnotationData& getData(const world_canvas_msgs::Annotation& ann);

  void updateWidget(QTreeWidgetItem* tree_item);

  bool saved() { return saved_; }

private:
  QTreeWidgetItem* tree_item_;
  std::vector<world_canvas_msgs::Annotation> annots_to_delete_;
  bool saved_;

  bool saveDeletes();
};

} // namespace wcf
