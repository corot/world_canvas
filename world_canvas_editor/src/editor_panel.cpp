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
#include <QMessageBox>
#include <QColorDialog>

#include <visualization_msgs/MarkerArray.h>
                                      #include <yocs_msgs/Wall.h>

#include <yocs_math_toolkit/geometry.hpp>
#include <world_canvas_client_cpp/unique_id.hpp>

#include "editor_panel.hpp"

#include "ui_editor_panel.h"

#define WORLD_NAME_TODO "Maze world"

namespace wcf
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
  connect( ui_->delAnnButton, SIGNAL( clicked() ), this, SLOT( delButtonClicked() ));
  connect( ui_->saveAnnButton, SIGNAL( clicked() ), this, SLOT( saveButtonClicked() ));
  connect( ui_->pickColorButton, SIGNAL( clicked() ), this, SLOT( pickColorClicked() ));

  // connect other signals to slots
  connect( ui_->annsTreeWidget, SIGNAL( itemDoubleClicked(QTreeWidgetItem *, int)),
           this, SLOT( annotSelected(QTreeWidgetItem *, int) ));

  // Manually build the shape combo so I can get custom values according to Annotation msg
  ui_->shapeComboBox->addItem("CUBE", visualization_msgs::Marker::CUBE);
  ui_->shapeComboBox->addItem("SPHERE", visualization_msgs::Marker::SPHERE);
  ui_->shapeComboBox->addItem("CYLINDER", visualization_msgs::Marker::CYLINDER);
  ui_->shapeComboBox->addItem("TEXT", visualization_msgs::Marker::TEXT_VIEW_FACING);

//  // Other tweaks to the UI
//  ui_->pickColorButton->setAutoFillBackground(true);
//  ui_->pickColorButton->setFlat(true);

  // Advertise a topic to publish a visual marker for the currently edited annotation
  ///marker_pub_ = nh_.advertise <visualization_msgs::MarkerArray> ("current_annotation", 1, true);
}

void RVizPluginEditor::newButtonClicked()
{
  ROS_DEBUG("New annotation");

  // Confirm that user want to discard current changes (if any)
  if (discardCurrentChanges() == false)
    return;

  current_annot_.reset(new world_canvas_msgs::Annotation);
  current_annot_->world = WORLD_NAME_TODO;
  current_annot_->id = uuid::toMsg(uuid::fromRandom());
  current_annot_->pose.header.frame_id = "/map";  // TODO
  current_annot_->pose.pose.pose.orientation.w = 1.0;  // Avoid non-normalized quaternions
  current_annot_->shape = visualization_msgs::Marker::TEXT_VIEW_FACING; // reasonable default
  current_annot_->color.a = 0.5;  // Avoid a rather confusing invisible shape

  annot2widgets(current_annot_);

  annotations_->publishMarker("current_annotation", -1, *current_annot_);

  ui_->updateButton->setEnabled(true);
  ui_->editMsgButton->setEnabled(true);
  ui_->delAnnButton->setEnabled(false);
  ui_->saveAnnButton->setEnabled(false);
  ui_->pickColorButton->setEnabled(true);
}

void RVizPluginEditor::updButtonClicked()
{
  ROS_DEBUG("Update annotation");

  widgets2annot(current_annot_);

  annotations_->publishMarker("current_annotation", -1, *current_annot_);

  ui_->saveAnnButton->setEnabled(true);
}

void RVizPluginEditor::msgButtonClicked()
{
  ROS_DEBUG("Set annotation message");

  ext_process_.reset(new QProcess(this));
  QString command_line = "rosrun rqt_annotation_data rqt_annotation_data";
  QStringList parameters;
  parameters << "param1" << "param2" << "param3";

  ext_process_->start(command_line);//, parameters);

  ann_data_sub_ = nh_.subscribe("/kkk", 10, &RVizPluginEditor::annDataCb, this, th_);
}

void RVizPluginEditor::annDataCb(const ros::MessageEvent<topic_tools::ShapeShifter>& msg_event)
{
  // TODO protect against more than one incoming message

  boost::shared_ptr<topic_tools::ShapeShifter const> const &msg = msg_event.getConstMessage();
  boost::shared_ptr<const ros::M_string> const& connection_header = msg_event.getConnectionHeaderPtr();

  ROS_DEBUG("Message of type '%s' and size %d bytes received", msg->getDataType().c_str(), msg->size());
  //ROS_DEBUG("%s", msg->getMessageDefinition().c_str());

  current_data_.reset(new world_canvas_msgs::AnnotationData);
  current_data_->id = uuid::toMsg(uuid::fromRandom());
  current_data_->type = msg->getDataType();
  current_data_->data.resize(msg->size());
  ros::serialization::OStream stream((uint8_t*)&current_data_->data[0], current_data_->data.size());
  msg->write(stream);

  current_annot_->data_id = current_data_->id;
  current_annot_->type = current_data_->type;

  // The new annotation have type and data, so it's now complete; we can save it!
  ui_->typeLineEdit->setText(current_data_->type.c_str());
  ui_->saveAnnButton->setEnabled(true);




    for (int i = 0; i < current_data_->data.size(); ++i) {
      printf("%d  ", current_data_->data[i]);

    }

    yocs_msgs::Wall object;
    uint32_t serial_size = current_data_->data.size();
    boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
    memcpy(buffer.get(), &current_data_->data[0], serial_size);
    ros::SerializedMessage sm(buffer, serial_size);
    sm.type_info = &typeid(object);
    sm.message_start;// += 4;
    try
    {
      ROS_DEBUG("Deserializing object '%s' of type '%s'",
                uuid::toHexString(current_data_->id).c_str(), current_data_->type.c_str());
      ros::serialization::deserializeMessage(sm, object);
      marker_pub_ = nh_.advertise <yocs_msgs::Wall> ("funciona", 1, true);
      marker_pub_.publish(object);
//      count++;
    }
    catch (ros::serialization::StreamOverrunException& e)
    {
      ROS_ERROR("Deserialization failed on object: %s", e.what());
    }
}

void RVizPluginEditor::delButtonClicked()
{
  if (!current_annot_ || ! current_data_)
  {
    // This should not happen, as the delete button should be disabled in this case!
    ROS_ERROR("Missing current annotation [data]: %d, %d", current_annot_?true:false, current_data_?true:false);
    assert(current_annot_);
    assert(current_data_);
    return;
  }
  ROS_DEBUG("Delete annotation");

  // Confirmation dialog
  int ret = QMessageBox::question(this, tr("Annotations Editor"),
                                  tr("Do you want to delete annotation '%1'?").arg(current_annot_->name.c_str()) +
                                  tr("\nThis opearation cannot be undone!"),
                                  QMessageBox::Yes | QMessageBox::Cancel,
                                  QMessageBox::Cancel);
  if (ret != QMessageBox::Yes)
    return;

  ui_->updateButton->setEnabled(false);
  ui_->editMsgButton->setEnabled(false);
  ui_->delAnnButton->setEnabled(false);
  ui_->saveAnnButton->setEnabled(true);
  ui_->pickColorButton->setEnabled(false);

  annotations_->del(current_annot_->id);

  current_annot_.reset();
  current_data_.reset();

  world_canvas_msgs::Annotation::Ptr empty(new world_canvas_msgs::Annotation);
  empty->pose.pose.pose.orientation.w = 1.0;  // Avoid non-normalized quaternions
  annot2widgets(empty);
}

void RVizPluginEditor::saveButtonClicked()
{
  if (!current_annot_ || ! current_data_)
  {
    // This should not happen, as the save button should be disabled in this case!
    ROS_ERROR("Missing current annotation [data]: %d, %d", current_annot_?true:false, current_data_?true:false);
    assert(current_annot_);
    assert(current_data_);
    return;
  }
  ROS_DEBUG("Save annotation");

  ui_->delAnnButton->setEnabled(true);
  ui_->saveAnnButton->setEnabled(false);

  // TODO: replace by an "update" method; and provide feedback to user in case of failure
  annotations_->del(current_annot_->id);
  if (annotations_->add(*current_annot_, *current_data_) == false)
  {
    ROS_ERROR("Add annotations failed");
    return;
  }

  if (annotations_->save() == false)
  {
    ROS_ERROR("Save annotations failed");
  }
}


void RVizPluginEditor::annotSelected(QTreeWidgetItem *item, int column)
{
  int sel_index = ui_->annsTreeWidget->indexOfTopLevelItem(item);
  ROS_DEBUG("Annotation %d selected", sel_index);

  // Check that user is not selecting again the current annotation
  if (current_annot_ && (current_annot_->id.uuid == annotations_->at(sel_index).id.uuid))
    return;

  // Confirm that user want to discard current changes (if any)
  if (discardCurrentChanges() == false)
    return;

  current_annot_.reset(new world_canvas_msgs::Annotation);
  *current_annot_ = annotations_->at(sel_index);
  current_data_.reset(new world_canvas_msgs::AnnotationData);
  *current_data_ = annotations_->getData(*current_annot_);

  annot2widgets(current_annot_);

  ui_->updateButton->setEnabled(true);
  ui_->editMsgButton->setEnabled(true);
  ui_->delAnnButton->setEnabled(true);
  ui_->saveAnnButton->setEnabled(false);
  ui_->pickColorButton->setEnabled(true);
}

void RVizPluginEditor::pickColorClicked()
{
  if (! current_annot_)
  {
    // This should not happen, as the pick color button should be disabled!
    ROS_ERROR("Trying to pick a color while we are not editing an annotation");
    assert(current_annot_);
  }

  current_color_ = QColorDialog::getColor(current_color_, this, "Pick annotation color",
                                          QColorDialog::ShowAlphaChannel);
  ui_->colorLabel->setStyleSheet(QString("background-color: %1").arg(current_color_.name()));
}

void RVizPluginEditor::widgets2annot(world_canvas_msgs::Annotation::Ptr annot)
{
  annot->name = ui_->nameLineEdit->text().toStdString();
  annot->type = ui_->typeLineEdit->text().toStdString();

  annot->shape = ui_->shapeComboBox->itemData(ui_->shapeComboBox->currentIndex()).toInt();
  annot->color.r = current_color_.redF();
  annot->color.g = current_color_.greenF();
  annot->color.b = current_color_.blueF();
  annot->color.a = current_color_.alphaF();
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

  annot->keywords.clear();
  QStringList list = ui_->keywordsTextEdit->toPlainText().split("\n", QString::SkipEmptyParts);
  int dup = list.removeDuplicates();
  if (dup > 0)
  {
    ROS_WARN("Discarding %d duplicated keywords", dup);
  }
  for (int i = 0; i < list.count(); ++i)
  {
    annot->keywords.push_back(list[i].toStdString());
  }

  annot->relationships.clear();
  list = ui_->relatedsTextEdit->toPlainText().split("\n", QString::SkipEmptyParts);
  dup = list.removeDuplicates();
  if (dup > 0)
  {
    ROS_WARN("Discarding %d duplicated relationships", dup);
  }
  for (int i = 0; i < list.count(); ++i)
  {
    // Annotations store relationships as uuid, but the user only works with the annotation's name
    std::vector<world_canvas_msgs::Annotation> anns = annotations_->getAnnotations(list[i].toStdString());

    ROS_DEBUG("%lu",anns.size());
    if (anns.size() == 0)
    {
      ROS_WARN("No relationship fetched for name '%s'", list[i].toStdString().c_str());
      continue;
      // TODO show dialog or other info to the user
    }

    for (int j = 0; j < anns.size(); ++j)
    {
      ROS_DEBUG("Add relationship '%s'/'%s' fetched for name '%s'",
                anns[j].name.c_str(), uuid::toHexString(anns[j].id).c_str(), list[i].toStdString().c_str());
      annot->relationships.push_back(anns[j].id);
    }
  }

  Q_EMIT configChanged();
}

void RVizPluginEditor::annot2widgets(world_canvas_msgs::Annotation::Ptr annot)
{
  ui_->nameLineEdit->setText(annot->name.c_str());
  ui_->typeLineEdit->setText(annot->type.c_str());

  int index = ui_->shapeComboBox->findData(annot->shape);
  if (index != -1)  // -1 for not found
    ui_->shapeComboBox->setCurrentIndex(index);
  else
    ui_->shapeComboBox->setCurrentIndex(visualization_msgs::Marker::TEXT_VIEW_FACING);

  current_color_.setRedF(annot->color.r);
  current_color_.setGreenF(annot->color.g);
  current_color_.setBlueF(annot->color.b);
  current_color_.setAlphaF(annot->color.a);
  ui_->colorLabel->setStyleSheet(QString("background-color: %1").arg(current_color_.name()));

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

  ui_->keywordsTextEdit->clear();
  for (int i = 0; i < annot->keywords.size(); ++i)
  {
    ui_->keywordsTextEdit->append(annot->keywords[i].c_str());
  }

  ui_->relatedsTextEdit->clear();
  for (int i = 0; i < annot->relationships.size(); ++i)
  {
    try
    {
      // Annotations store relationships as uuid, but the user only works with the annotation's name
      std::string relatedAnn = annotations_->getAnnotation(annot->relationships[i]).name;
      ui_->relatedsTextEdit->append(relatedAnn.c_str());
    }
    catch (ros::Exception& e)
    {
      ROS_WARN("Discarding relationship: %s", e.what());
    }
  }
}

bool RVizPluginEditor::discardCurrentChanges()
{
  if (! current_annot_ || ! ui_->saveAnnButton->isEnabled()) // no current annotation or already saved
    return true;

  int ret = QMessageBox::warning(this, tr("Annotations Editor"),
                                 tr("Current annotation not saved\n" \
                                    "Do you want to discard changes?"),
                                 QMessageBox::Yes | QMessageBox::Cancel,
                                 QMessageBox::Cancel);
  return (ret == QMessageBox::Yes);
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void RVizPluginEditor::save(rviz::Config config) const
{
  rviz::Panel::save(config);
//  config.mapSetValue( "Topic", output_topic_ );
}

// Load all configuration data for this panel from the given Config object.
void RVizPluginEditor::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
//  QString topic;
//  if( config.mapGetString( "Topic", &topic ))
//  {
//    output_topic_editor_->setText( topic );
//    updateTopic();
//  }
}

} // end namespace wcf


// Tell pluginlib about this class
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(wcf::RVizPluginEditor, rviz::Panel )
