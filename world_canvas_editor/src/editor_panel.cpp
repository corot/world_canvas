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

#include <stdio.h>

#include <tf/tf.h>

#include <QProcess>

#include <unique_id/unique_id.h>

#include <visualization_msgs/Marker.h>

#include <yocs_math_toolkit/geometry.hpp>

#include "editor_panel.hpp"

#include "ui_editor_panel.h"

#define WORLD_NAME_TODO "Maze world"

namespace world_canvas
{

// BEGIN_TUTORIAL
// Here is the implementation of the RVizPluginEditor class.  RVizPluginEditor
// has these responsibilities:
//
// - Act as a container for GUI elements DriveWidget and QLineEdit.
// - Publish command velocities 10 times per second (whether 0 or not).
// - Saving and restoring internal state from a config file.
//
// We start with the constructor, doing the standard Qt thing of
// passing the optional *parent* argument on to the superclass
// constructor, and also zero-ing the velocities we will be
// publishing.
RVizPluginEditor::RVizPluginEditor(QWidget* parent)
  : rviz::Panel(parent), ui_(new Ui::EditorPanel())
{
  // set up the GUI
  ui_->setupUi(this);

  annotations_.reset(new AnnotationsList(WORLD_NAME_TODO, ui_->annsTreeWidget));

  // connect buttons to actions
  connect( ui_->newAnnButton, SIGNAL( clicked() ), this, SLOT( newButtonClicked() ));
  connect( ui_->updateButton, SIGNAL( clicked() ), this, SLOT( updButtonClicked() ));
  connect( ui_->editMsgButton, SIGNAL( clicked() ), this, SLOT( msgButtonClicked() ));

  // Manually build the shape combo so I can get custom values according to Annotation msg
  ui_->shapeComboBox->addItem("CUBE", visualization_msgs::Marker::CUBE);
  ui_->shapeComboBox->addItem("SPHERE", visualization_msgs::Marker::SPHERE);
  ui_->shapeComboBox->addItem("CYLINDER", visualization_msgs::Marker::CYLINDER);
  ui_->shapeComboBox->addItem("TEXT", visualization_msgs::Marker::TEXT_VIEW_FACING);
}

void RVizPluginEditor::newButtonClicked()
{
  ROS_DEBUG("New annotation");

  current_annot_.reset(new world_canvas_msgs::Annotation);
  current_annot_->world = WORLD_NAME_TODO;
  current_annot_->id = unique_id::toMsg(unique_id::fromRandom());
  current_annot_->pose.header.frame_id = "/map";  // TODO
  current_annot_->pose.pose.pose.orientation.w = 1.0;  // Avoid non-normalized quaternions

  annot2widgets(current_annot_);
  publishMarker();
  //annotations_>

  ui_->updateButton->setEnabled(true);
  ui_->editMsgButton->setEnabled(true);
}

void RVizPluginEditor::updButtonClicked()
{
  ROS_DEBUG("Update annotation");

  widgets2annot(current_annot_);
  publishMarker();
}

void RVizPluginEditor::msgButtonClicked()
{
  ROS_DEBUG("Set annotation message");

  ext_process_.reset(new QProcess(this));
  QString command_line = "rosrun rqt_annotation_data rqt_annotation_data";
  QStringList parameters;
  parameters << "param1" << "param2" << "param3";

  ext_process_->start(command_line);//, parameters);

  ann_data_sub_ = nh_.subscribe("/kk", 10, &RVizPluginEditor::annDataCb, this, th_);
}

void RVizPluginEditor::annDataCb(const ros::MessageEvent<topic_tools::ShapeShifter>& msg_event)
{
  boost::shared_ptr<topic_tools::ShapeShifter const> const &msg = msg_event.getConstMessage();
  boost::shared_ptr<const ros::M_string> const& connection_header = msg_event.getConnectionHeaderPtr();

  ROS_DEBUG("Message of type '%s' and size %d bytes received", msg->getDataType().c_str(), msg->size());
  //ROS_DEBUG("%s", msg->getMessageDefinition().c_str());

  current_data_.reset(new world_canvas_msgs::AnnotationData);
  current_data_->id = unique_id::toMsg(unique_id::fromRandom());
  current_annot_->data_id = current_data_->id;
  current_data_->type = msg->getDataType();

  current_data_->data.resize(500);//msg->size());  joder!!!!   4 bytes????
  ROS_DEBUG("1");
  ros::serialization::OStream stream((uint8_t*)&current_data_->data[0], current_data_->data.size());
/////  msg->write(*(new ros::serialization::OStream((uint8_t*)&current_data_->data[0], current_data_->data.size())));

  ROS_DEBUG("2");
  msg->write(stream);
  ROS_DEBUG("%d %d %d %d %d %d %d %d",current_data_->data[0], current_data_->data[1], current_data_->data[2],current_data_->data[3],current_data_->data[4],current_data_->data[5],current_data_->data[6],current_data_->data[7]);

}

void RVizPluginEditor::widgets2annot(  world_canvas_msgs::Annotation::Ptr annot)
{
  annot->name = ui_->nameLineEdit->text().toStdString();
  annot->type = ui_->typeLineEdit->text().toStdString();

  annot->shape = ui_->shapeComboBox->itemData(ui_->shapeComboBox->currentIndex()).toInt();
  annot->color.r = 1.0;
  annot->color.g = 1.0;
  annot->color.b = 1.0;
  annot->color.a = 0.8;
  annot->size.x = ui_->lDoubleSpinBox->value();
  annot->size.y = ui_->wDoubleSpinBox->value();
  annot->size.z = ui_->hDoubleSpinBox->value();

  tf::Transform tf(tf::createQuaternionFromRPY(ui_->rollDoubleSpinBox->value(),
                                               ui_->pitchDoubleSpinBox->value(),
                                               ui_->yawDoubleSpinBox->value()),
                   tf::Vector3(ui_->xDoubleSpinBox->value(),
                               ui_->yDoubleSpinBox->value(),
                               ui_->zDoubleSpinBox->value()));
  tf::poseTFToMsg(tf, annot->pose.pose.pose);

//  annot->keywords
//  annot->relationships


  Q_EMIT configChanged();
}

void RVizPluginEditor::annot2widgets( world_canvas_msgs::Annotation::Ptr annot)
{
  ui_->nameLineEdit->setText(annot->name.c_str());
  ui_->typeLineEdit->setText(annot->type.c_str());

  int index = ui_->shapeComboBox->findData(annot->shape);
  if ( index != -1 )  // -1 for not found
    ui_->shapeComboBox->setCurrentIndex(index);
  else
    ui_->shapeComboBox->setCurrentIndex(visualization_msgs::Marker::TEXT_VIEW_FACING);

  annot->color.r;
  annot->color.g;
  annot->color.b;
  annot->color.a;
  ui_->lDoubleSpinBox->setValue(annot->size.x);
  ui_->wDoubleSpinBox->setValue(annot->size.y);
  ui_->hDoubleSpinBox->setValue(annot->size.z);

  ui_->xDoubleSpinBox->setValue(annot->pose.pose.pose.position.x);
  ui_->yDoubleSpinBox->setValue(annot->pose.pose.pose.position.y);
  ui_->zDoubleSpinBox->setValue(annot->pose.pose.pose.position.z);
  ui_->rollDoubleSpinBox->setValue(mtk::roll(annot->pose.pose.pose));
  ui_->pitchDoubleSpinBox->setValue(mtk::pitch(annot->pose.pose.pose));
  ui_->yawDoubleSpinBox->setValue(tf::getYaw(annot->pose.pose.pose.orientation));  // TODO add yaw() to mtk if I change something else there
  // TODO  // TODO  // TODO  annot->pose.header.frame_id = "/map";
}
//  // Only take action if the name has changed.
//  if( new_topic != output_topic_ )
//  {
//    output_topic_ = new_topic;
//    // If the topic is the empty string, don't publish anything.
//    if( output_topic_ == "" )
//    {
//      velocity_publisher_.shutdown();
//    }
//    else
//    {
//      // The old ``velocity_publisher_`` is destroyed by this assignment,
//      // and thus the old topic advertisement is removed.  The call to
//      // nh_advertise() says we want to publish data on the new topic
//      // name.
//      velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>( output_topic_.toStdString(), 1 );
//    }
//    // rviz::Panel defines the configChanged() signal.  Emitting it
//    // tells RViz that something in this panel has changed that will
//    // affect a saved config file.  Ultimately this signal can cause
//    // QWidget::setWindowModified(true) to be called on the top-level
//    // rviz::VisualizationFrame, which causes a little asterisk ("*")
//    // to show in the window's title bar indicating unsaved changes.
//    Q_EMIT configChanged();
//  }


// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void RVizPluginEditor::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
//  config.mapSetValue( "Topic", output_topic_ );
}

// Load all configuration data for this panel from the given Config object.
void RVizPluginEditor::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
//  QString topic;
//  if( config.mapGetString( "Topic", &topic ))
//  {
//    output_topic_editor_->setText( topic );
//    updateTopic();
//  }
}

bool RVizPluginEditor::publishMarker()
{
  // Advertise a topic for retrieved annotations' visualization markers
  marker_pub_ = nh_.advertise <visualization_msgs::Marker> ("current_annotation", 1, true);

  visualization_msgs::Marker marker;
  marker.id     = 666;
  marker.header = current_annot_->pose.header;
  marker.type   = current_annot_->shape;
  marker.ns     = current_annot_->type;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose   = current_annot_->pose.pose.pose;
  marker.scale  = current_annot_->size;
  marker.color  = current_annot_->color;

  marker_pub_.publish(marker);
  return true;
}

} // end namespace world_canvas


// Tell pluginlib about this class
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(world_canvas::RVizPluginEditor, rviz::Panel )
