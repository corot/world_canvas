/*
 * worlds_list.hpp
 *
 *  Created on: Oct 11, 2014
 *      Author: Jorge Santos
 */

#include <ros/ros.h>

#include <QPoint>
#include <QTreeWidget>

#include <world_canvas_client_cpp/world_collection.hpp>


class QMenu;
class QAction;
class AnnotationsList;


namespace wcf
{

class WorldsList : public QTreeWidget, WorldCollection
{
Q_OBJECT

public:
  WorldsList(QWidget* parent);

  void updateWidget();
  void setCurrent(int index);
  int  getCurrent() { return current_world_; }
  std::string& getName(int index) { return world_names[index]; }

  boost::shared_ptr<AnnotationsList> annotations_;

Q_SIGNALS:
  void worldSelected(int index);
  void annotSelected(int index);

private:
  int        current_world_;
  int       world_to_clone_;
  QMenu*      context_menu_;
  QAction*   act_new_world_;
  QAction* act_clone_world_;

private Q_SLOTS:
  void contextMenuRequested(QPoint point);
  void treeDoubleClicked(QTreeWidgetItem *item, int column);
  void newWorld();
  void cloneWorld();
};

} // namespace wcf
