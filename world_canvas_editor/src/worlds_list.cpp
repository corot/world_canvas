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
#include <QInputDialog>

#include <world_canvas_msgs/ListWorlds.h>

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

} // namespace wcf
