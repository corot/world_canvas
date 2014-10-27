/*
 * worlds_list.cpp
 *
 *  Created on: Oct 11, 2014
 *      Author: Jorge Santos
 */

#include <ros/ros.h>

#include <QMenu>
#include <QAction>
#include <QHeaderView>
#include <QTreeWidgetItem>
#include <QLineEdit>
#include <QMessageBox>
#include <QFileDialog>
#include <QInputDialog>

#include <world_canvas_msgs/ListWorlds.h>

#include "world_canvas_editor/map_loader.hpp"
#include "world_canvas_editor/annotations.hpp"
#include "world_canvas_editor/worlds_list.hpp"

namespace wcf
{

WorldsList::WorldsList(QWidget* parent)
  : QTreeWidget(parent), WorldCollection(""), current_world_(-1)
{
  updateWidget();

  // Configure worlds and annotations tree widget
  connect(this, SIGNAL(itemDoubleClicked(QTreeWidgetItem *, int)),
          this, SLOT(treeDoubleClicked(QTreeWidgetItem *, int)));

  // Prepare context menu to allow additional operations with worlds other than selecting
  context_menu_ = new QMenu();
  this->setContextMenuPolicy(Qt::CustomContextMenu);

  act_new_world_ = context_menu_->addAction("New world");
  act_clone_world_ = context_menu_->addAction("Clone world");

  connect(this, SIGNAL(customContextMenuRequested(const QPoint)),
          this, SLOT(contextMenuRequested(QPoint)));
  connect(act_new_world_, SIGNAL(triggered()), this, SLOT(newWorld()));
  connect(act_clone_world_, SIGNAL(triggered()), this, SLOT(cloneWorld()));

  ROS_INFO("World collection ready!");
}

void WorldsList::treeDoubleClicked(QTreeWidgetItem *item, int column)
{
  if (item->parent() == NULL)
  {
    Q_EMIT worldSelected(this->indexOfTopLevelItem(item));
  }
  else
  {
    Q_EMIT annotSelected(item->parent()->indexOfChild(item));
  }
}

void WorldsList::contextMenuRequested(QPoint point)
{
  // Enable clone world option if cursor is over a world
  act_clone_world_->setDisabled(! this->indexAt(point).isValid());

  // Check which world is under cursor in case the user selects "clone world" action
  world_to_clone_ = -1;
  if (this->indexAt(point).isValid() == false)
  {
    ROS_DEBUG("Cursor is not over a world; option disabled");
  }
  else if (this->indexAt(point).parent().isValid() == false)
  {
    // Already at top level, i.e. over a world name
    world_to_clone_ = this->indexAt(point).row();
  }
  else
  {
    // Move to top level to get the world name for the annotation under cursor
    world_to_clone_ = this->indexAt(point).parent().row();
  }

  // Ready to show the popup menu
  context_menu_->popup(this->mapToGlobal(point));
}

void WorldsList::newWorld()
{
  ROS_DEBUG("Add a new world");

  bool ok;
  QString text = QInputDialog::getText(this, "", tr("New world name:"),
                                       QLineEdit::Normal, "", &ok);
  if (ok && !text.isEmpty())
  {
    world_names.push_back(text.toStdString());

    // Ask user to provide a geometric map for the new world; we will go on even if he doesn't or
    // if there's an error, but adding annotations blindly, without a reference would be horrible!
    if (loadGeometricMap(text.toStdString()) == false)
      ROS_WARN("We don't have a geometric map to show for the new world");

    // Emit a signal to the main editor so he can handle the change of world
    Q_EMIT worldSelected(world_names.size() - 1);
  }
}

void WorldsList::cloneWorld()
{
  if (world_to_clone_ == -1)
    ROS_ERROR("Cursor is not over a world? This should be avoided by disabling the clone action!");
  else
    ROS_DEBUG("Clone world '%s'", world_names[world_to_clone_].c_str());

  // Clone world is TODO    :(


  world_to_clone_ = -1;
}

void WorldsList::setCurrent(int index)
{
  if (current_world_ != index)
  {
    annotations_.reset(new AnnotationsList(world_names[index]));
    current_world_ = index;
    updateWidget();

    this->setCurrentItem(this->topLevelItem(index));

    // Request server to publish the geometric map for the selected world
    annotations_->publish("map", "nav_msgs/OccupancyGrid", true, false);

    // TODO: we should notify somehow if this world has no geometric map to show
  }
}

void WorldsList::updateWidget()
{
  QTreeWidget::clear();
  this->header()->resizeSection(0, 160);
  this->header()->resizeSection(1, 160);

  for (unsigned int i = 0; i < world_names.size(); i++)
  {
    QTreeWidgetItem *item = new QTreeWidgetItem();
    item->setText(0, world_names[i].c_str());
    this->addTopLevelItem(item);

    if (annotations_ && i == current_world_)
    {
      annotations_->updateWidget(item);
      item->setExpanded(true);
    }
  }
}

bool WorldsList::loadGeometricMap(const std::string& world_name)
{
  QString fileName = QFileDialog::getOpenFileName(this, tr("Load Map"),
                                                  "~", tr("Map descriptor (*.yaml)"));
  if (fileName.isEmpty() == true)
    return false;

  try
  {
    nav_msgs::OccupancyGrid map;
    MapLoader::load(fileName.toStdString(), map);

    world_canvas_msgs::Annotation map_annot;
    world_canvas_msgs::AnnotationData map_data;

    map_annot.world = world_name;
    map_annot.id = uuid::toMsg(uuid::fromRandom());
    map_annot.data_id = uuid::toMsg(uuid::fromRandom());
    map_annot.name = "2D map";
    map_annot.type = "nav_msgs/OccupancyGrid";
    map_annot.pose.header.frame_id = "/map";  // TODO  part of world????
    map_annot.pose.pose.pose.orientation.w = 1.0;  // Avoid non-normalized quaternions
    map_annot.shape = visualization_msgs::Marker::TEXT_VIEW_FACING; // reasonable default
  //  map_annot.color.a = 0.5;  // Avoid a rather confusing invisible shape
    map_data.id = map_annot.data_id;
    map_data.type = map_annot.type;
  //  map_data.data =

    std::size_t size = ros::serialization::serializationLength(map);
    if (size <= 0)
      throw ros::Exception("Non-positive serialization length");

    // we convert the message into a string because that is easy to sent back & forth with Python
    // This is fine since C0x because &string[0] is guaranteed to point to a contiguous block of memory
    map_data.data.resize(size + 4);
    *reinterpret_cast<uint32_t*>(&map_data.data[0]) = size;
    ros::serialization::OStream stream_arg(reinterpret_cast<uint8_t*>(&map_data.data[4]), size);
    ros::serialization::serialize(stream_arg, map);

    AnnotationCollection ac(world_name);
    ac.add(map_annot, map_data);
    if (ac.save() == false)
      throw ros::Exception("Save map on database failed");

    return true;
  }
  catch (ros::Exception& e)
  {
    QMessageBox::critical(this, tr("Annotations Editor"),
                                tr("Load map failed:\n%s") + QString(e.what()),
                                QMessageBox::Ok);
    return false;
  }
}

} // namespace wcf
