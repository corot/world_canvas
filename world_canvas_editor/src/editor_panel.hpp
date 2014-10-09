/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef EDITOR_PANEL_H
#define EDITOR_PANEL_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/message_traits.h>
#include <topic_tools/shape_shifter.h>

#include <rviz/panel.h>

#include "annotations.hpp"

namespace Ui
{
class EditorPanel;
}

class QColor;
class QProcess;
class QTreeWidgetItem;

namespace wcf
{
// Here we declare our new subclass of rviz::Panel.  Every panel which
// can be added via the Panels/Add_New_Panel menu is a subclass of
// rviz::Panel.
//
// RVizPluginEditor will show a text-entry field to set the output topic
// and a 2D control area.  The 2D control area is implemented by the
// DriveWidget class, and is described there.
class RVizPluginEditor: public rviz::Panel
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  RVizPluginEditor( QWidget* parent = 0 );

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

  // Next come a couple of public Qt slots.
public Q_SLOTS:
  void newButtonClicked();
  void updButtonClicked();
  void msgButtonClicked();
  void delButtonClicked();
  void saveButtonClicked();
  void pickColorClicked();
  void annotSelected(QTreeWidgetItem *item, int column);

  // Here we declare some internal slots.
protected Q_SLOTS:

  // Then we finish up with protected member variables.
protected:
  ros::NodeHandle nh_;  // The ROS node handle.

  Ui::EditorPanel *ui_;

  ros::Publisher  marker_pub_;
  ros::Publisher  annotation_pub_;
  ros::Publisher  annot_data_pub_;
  ros::Subscriber annot_data_sub_;
  ros::Subscriber user_action_sub_;
  ros::TransportHints th_;

  boost::shared_ptr<QProcess>        ext_process_;
  boost::shared_ptr<AnnotationsList> annotations_;

  QColor                             current_color_;
  world_canvas_msgs::Annotation::Ptr current_annot_;
  world_canvas_msgs::AnnotationData::Ptr current_data_;
  boost::shared_ptr<topic_tools::ShapeShifter> current_msg_;

  void userActCb(const std_msgs::String::ConstPtr& msg);
  void annDataCb(const ros::MessageEvent<topic_tools::ShapeShifter>& msg_event);

  bool discardCurrentChanges();
  void annot2widgets(world_canvas_msgs::Annotation::Ptr annot);
  void widgets2annot(world_canvas_msgs::Annotation::Ptr annot);
};

} // end namespace wcf

#endif // EDITOR_PANEL_H
