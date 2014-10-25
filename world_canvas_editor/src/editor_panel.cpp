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

#include <yocs_math_toolkit/geometry.hpp>
#include <world_canvas_msgs/EditAnnotationsData.h>

#include "world_canvas_editor/editor_panel.hpp"

namespace wcf
{

RVizPluginEditor::RVizPluginEditor(QWidget* parent)
  : rviz::Panel(parent), ui_(new Ui::EditorPanel())
{
  // set up the GUI
  ui_->setupUi(this);

  // connect buttons to actions
  connect( ui_->newAnnButton, SIGNAL( clicked() ), this, SLOT( newButtonClicked() ));
  connect( ui_->updateButton, SIGNAL( clicked() ), this, SLOT( updButtonClicked() ));
  connect( ui_->editMsgButton, SIGNAL( clicked() ), this, SLOT( msgButtonClicked() ));
  connect( ui_->delAnnButton, SIGNAL( clicked() ), this, SLOT( delButtonClicked() ));
  connect( ui_->saveAnnButton, SIGNAL( clicked() ), this, SLOT( saveButtonClicked() ));
  connect( ui_->pickColorButton, SIGNAL( clicked() ), this, SLOT( pickColorClicked() ));

  // Store worlds and annotations tree widget as a private attribute for easy access
  worlds_list_.reset(ui_->annsTreeWidget);

  // Connect world/annotation selection signals to slots here
  connect( worlds_list_.get(), SIGNAL( worldSelected(int) ), this, SLOT( worldSelected(int) ));
  connect( worlds_list_.get(), SIGNAL( annotSelected(int) ), this, SLOT( annotSelected(int) ));

  // Manually build the shape combo so I can get custom values according to Annotation msg
  ui_->shapeComboBox->addItem("CUBE", visualization_msgs::Marker::CUBE);
  ui_->shapeComboBox->addItem("SPHERE", visualization_msgs::Marker::SPHERE);
  ui_->shapeComboBox->addItem("CYLINDER", visualization_msgs::Marker::CYLINDER);
  ui_->shapeComboBox->addItem("TEXT", visualization_msgs::Marker::TEXT_VIEW_FACING);

  changes_saved_ = true;
}

void RVizPluginEditor::newButtonClicked()
{
  ROS_DEBUG("New annotation");

  // Confirm that user want to discard current changes (if any)
  if (discardCurrentChanges() == false)
    return;

  current_annot_.reset(new world_canvas_msgs::Annotation);
  current_annot_->world = worlds_list_->getName(worlds_list_->getCurrent());
  current_annot_->id = uuid::toMsg(uuid::fromRandom());
  current_annot_->pose.header.frame_id = "/map";  // TODO  part of world????
  current_annot_->pose.pose.pose.orientation.w = 1.0;  // Avoid non-normalized quaternions
  current_annot_->shape = visualization_msgs::Marker::TEXT_VIEW_FACING; // reasonable default
  current_annot_->color.a = 0.5;  // Avoid a rather confusing invisible shape

  // Update GUI
  annot2widgets(current_annot_);
  showCurrentAnnot();

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
  showCurrentAnnot();
  changes_saved_ = false;

  // Now it makes sense to save but only if we have already annotation's data
  if (current_data_)
    ui_->saveAnnButton->setEnabled(true);
}

void RVizPluginEditor::msgButtonClicked()
{
  ROS_DEBUG("Set annotation message");

  ext_process_.reset(new QProcess(this));
  QString command_line = "rosrun rqt_annotation_data rqt_annotation_data";
  // QStringList parameters; parameters << "param1" << "param2" << "param3";

  QApplication::setOverrideCursor(Qt::WaitCursor);
  ext_process_->start(command_line);//, parameters);

  // Prepare call to edit annotation data service on the just started RQT plugin
  ros::NodeHandle nh;
  ros::ServiceClient client =
      nh.serviceClient<world_canvas_msgs::EditAnnotationsData>("/__edit_annotation_data__");

  // Obviously we must wait until the plugin has time to start and create the service
  ROS_INFO("Waiting for edit_annotations_data service...");
  if (client.waitForExistence(ros::Duration(10.0)) == false)
  {
    ROS_ERROR("Service edit_annotations_data not available after 10s");
    QMessageBox::critical(this, tr("Annotations Editor"),
                                tr("Cannot communicate with the data editor"),
                                QMessageBox::Ok);
    QApplication::restoreOverrideCursor();
    return;
  }

  // Prepare request depending on whether current annotation has already associated data
  assert(current_annot_);
  world_canvas_msgs::EditAnnotationsData srv;
  srv.request.annotation = *current_annot_;

  if (current_data_)
  {
    ROS_INFO("Requesting rqt_annotation_data to edit existing annotation's data");
    srv.request.action = world_canvas_msgs::EditAnnotationsData::Request::EDIT;
    srv.request.data = *current_data_;
  }
  else
  {
    ROS_INFO("Requesting rqt_annotation_data to create annotation's data from scratch");
    srv.request.action = world_canvas_msgs::EditAnnotationsData::Request::NEW;
    srv.request.data.id = uuid::toMsg(uuid::fromRandom());
    // We provide the uuid, so the plugin doesn't need extra dependency on uuid library
  }

  if (client.call(srv))
  {
    switch (srv.response.action)
    {
      case world_canvas_msgs::EditAnnotationsData::Response::UPDATE:
        if (!current_data_)
          current_data_.reset(new world_canvas_msgs::AnnotationData);

        *current_data_ = srv.response.data;

        current_annot_->data_id = current_data_->id;
        current_annot_->type = current_data_->type;

        // The current annotation have type and data, so it's now complete; we can save it!
        ui_->typeLineEdit->setText(current_data_->type.c_str());
        ui_->saveAnnButton->setEnabled(true);
        changes_saved_ = false;
        break;
      case world_canvas_msgs::EditAnnotationsData::Response::DELETE:
        current_data_.reset();

        // The current annotation have no type or data anymore; we cannot save it
        ui_->typeLineEdit->setText("");
        ui_->saveAnnButton->setEnabled(false);
        changes_saved_ = false;
        break;
      case world_canvas_msgs::EditAnnotationsData::Response::CANCEL:
        break;
      default:
        ROS_ERROR("Service edit_annotations_data returned an unexpected action: %d", srv.response.action);
        break;
    }
  }
  else
  {
    ROS_ERROR("Call service edit_annotations_data failed");
    QMessageBox::critical(this, tr("Annotations Editor"),
                                tr("Communication error with the data editor"),
                                QMessageBox::Ok);
  }
  QApplication::restoreOverrideCursor();
}

void RVizPluginEditor::delButtonClicked()
{
  ROS_DEBUG("Delete annotation");

  if (!current_annot_ || ! current_data_)
  {
    // This should not happen, as the delete button should be disabled in this case!
    ROS_ERROR("Missing current annotation [data]: %d, %d", current_annot_?true:false, current_data_?true:false);
    assert(current_annot_);
    assert(current_data_);
    return;
  }

  // Confirmation dialog
  int ret = QMessageBox::question(this, tr("Annotations Editor"),
                                  tr("Do you want to delete annotation '%1'?").arg(current_annot_->name.c_str()) +
                                  tr("\nThis opearation cannot be undone!"),
                                  QMessageBox::Yes | QMessageBox::Cancel,
                                  QMessageBox::Cancel);
  if (ret != QMessageBox::Yes)
    return;

  worlds_list_->annotations_->remove(*current_annot_);

  // Reset current annotation and make widgets empty
  current_annot_.reset();
  current_data_.reset();

  world_canvas_msgs::Annotation::Ptr empty(new world_canvas_msgs::Annotation);
  empty->pose.pose.pose.orientation.w = 1.0;  // Avoid non-normalized quaternions

  // Update GUI
  annot2widgets(empty);
  hideCurrentAnnot();

  ui_->updateButton->setEnabled(false);
  ui_->editMsgButton->setEnabled(false);
  ui_->delAnnButton->setEnabled(false);
  ui_->saveAnnButton->setEnabled(true);
  ui_->pickColorButton->setEnabled(false);

  changes_saved_ = false;
}

void RVizPluginEditor::saveButtonClicked()
{
  ROS_DEBUG("Save annotation");

  if (!current_annot_ != ! current_data_)
  {
    // XOR; this should not happen, as the save button should be disabled in this case!
    // The 2 valid cases are:
    //  - true, true: where we are editing an annotations already having data
    //  - false, false: we have deleted the current annotation and we want to remove also from database
    ROS_ERROR("Missing current annotation [data]: %d, %d", current_annot_?true:false, current_data_?true:false);
    assert(current_annot_);
    assert(current_data_);
    return;
  }

  if (current_annot_ && current_data_)
  {
    // Only apply for the 1st valid case
    ui_->delAnnButton->setEnabled(true);

    // Add a new/update current annotation. TODO: provide feedback to user in case of failure
    if (worlds_list_->annotations_->hasAnnotation(current_annot_->id))
    {
      if (worlds_list_->annotations_->update(*current_annot_, *current_data_) == false)
      {
        ROS_ERROR("Update annotations failed");
        return;
      }
    }
    else
    {
      if (worlds_list_->annotations_->add(*current_annot_, *current_data_) == false)
      {
        ROS_ERROR("Add annotations failed");
        return;
      }
    }
  }

  // The rest apply for both
  ui_->saveAnnButton->setEnabled(false);
  changes_saved_ = true;

  if (worlds_list_->annotations_->save() == false)
  {
    ROS_ERROR("Save annotations failed");
  }
}

void RVizPluginEditor::worldSelected(int index)
{
  ROS_DEBUG("World %d selected (%s)", index, worlds_list_->getName(index).c_str());

  // Check that user is not selecting again the current world
  if (index == worlds_list_->getCurrent())
    return;

  // Change world resets current annotation, so confirm that user want to discard changes (if any)
  if (discardCurrentChanges() == false)
    return;

  ui_->newAnnButton->setEnabled(true);
  worlds_list_->setCurrent(index);
  ui_->editAnnGroupBox->setTitle(tr("Edit annotations for world '%1'")
                                 .arg(worlds_list_->getName(index).c_str()));

  // Reset current annotation and make widgets empty
  current_annot_.reset();
  current_data_.reset();

  world_canvas_msgs::Annotation::Ptr empty(new world_canvas_msgs::Annotation);
  empty->pose.pose.pose.orientation.w = 1.0;  // Avoid non-normalized quaternions

  // Update GUI
  annot2widgets(empty);
  hideCurrentAnnot();

  ui_->updateButton->setEnabled(false);
  ui_->editMsgButton->setEnabled(false);
  ui_->delAnnButton->setEnabled(false);
  ui_->saveAnnButton->setEnabled(false);
  ui_->pickColorButton->setEnabled(false);
  changes_saved_ = true;
}

void RVizPluginEditor::annotSelected(int index)
{
  ROS_DEBUG("Annotation %d selected", index);

  // Check that user is not selecting again the current annotation
  if (current_annot_ && (current_annot_->id.uuid == worlds_list_->annotations_->at(index).id.uuid))
    return;

  // Choose another annotation resets current one, so confirm that user want to discard changes (if any)
  if (discardCurrentChanges() == false)
    return;

  current_annot_.reset(new world_canvas_msgs::Annotation);
  *current_annot_ = worlds_list_->annotations_->at(index);
  current_data_.reset(new world_canvas_msgs::AnnotationData);
  *current_data_ = worlds_list_->annotations_->getData(*current_annot_);

  // Update GUI
  annot2widgets(current_annot_);
  showCurrentAnnot();

  ui_->updateButton->setEnabled(true);
  ui_->editMsgButton->setEnabled(true);
  ui_->delAnnButton->setEnabled(true);
  ui_->saveAnnButton->setEnabled(false);
  ui_->pickColorButton->setEnabled(true);
  changes_saved_ = true;
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
  assert(annot);

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
    std::vector<world_canvas_msgs::Annotation> anns = worlds_list_->annotations_->getAnnotations(list[i].toStdString());
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
  assert(annot);

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
      std::string relatedAnn = worlds_list_->annotations_->getAnnotation(annot->relationships[i]).name;
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
  if (! current_annot_ || changes_saved_) // no current annotation or already saved
    return true;

  int ret = QMessageBox::warning(this, tr("Annotations Editor"),
                                 tr("Current annotation not saved\n" \
                                    "Do you want to discard changes?"),
                                 QMessageBox::Yes | QMessageBox::Cancel,
                                 QMessageBox::Cancel);
  return (ret == QMessageBox::Yes);
}

void RVizPluginEditor::showCurrentAnnot()
{
  assert(current_annot_);
  float tmp = current_annot_->color.a;
  current_annot_->color.a = 1.0;
  worlds_list_->annotations_->publishMarker("current_annotation", -1, *current_annot_);
  current_annot_->color.a = tmp;
}

void RVizPluginEditor::hideCurrentAnnot()
{
  worlds_list_->annotations_->clearMarkers("current_annotation");
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
