#!/usr/bin/env python

# Copyright (c) 2011, Dorian Scholz, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import division
import sys
import math
import random
import time

from python_qt_binding.QtGui import QMessageBox
from python_qt_binding.QtCore import Slot, QSignalMapper, QTimer, qWarning

import roslib
import rospy
import genpy
from rqt_gui_py.plugin import Plugin
from .msg_editor_widget import MsgEditorWidget
from rqt_py_common.topic_helpers import get_field_type

import std_msgs.msg
from world_canvas_msgs.msg import *
from world_canvas_msgs.srv import *
from world_canvas_utils.serialization import *

class MsgEditor(Plugin):

    def __init__(self, context):
        super(MsgEditor, self).__init__(context)
        self.setObjectName('MsgEditor')

        # create widget
        self._widget = MsgEditorWidget()
        self._widget.accept.connect(self.accept)
        self._widget.cancel.connect(self.cancel)
        self._widget.clean.connect(self.clean_up_publishers)
        self._widget.msg_type_changed.connect(self.msg_type_changed)
        self._widget.change_publisher.connect(self.change_publisher)
#         self._widget.publish_once.connect(self.publish_once)
#         self._widget.remove_publisher.connect(self.remove_publisher)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # create context for the expression eval statement
        self._eval_locals = {'i': 0}
        for module in (math, random, time):
            self._eval_locals.update(module.__dict__)
        self._eval_locals['genpy'] = genpy
        del self._eval_locals['__name__']
        del self._eval_locals['__doc__']

#         self._timeout_mapper = QSignalMapper(self)
#         self._timeout_mapper.mapped[int].connect(self.publish_once)

        # add our self to the main window
        context.add_widget(self._widget)
        
        self.annot_name = 'UNKNOWN'
        self.edit_data_srv = \
            rospy.Service('/__edit_annotation_data__', EditAnnotationsData, self.on_edit_data_srv)
#         self.user_action_pub = rospy.Publisher('/_annotation_data_user_action_', std_msgs.msg.String,### latch=True,
#                                                 queue_size=1)
# 
#         self.annotation_sub = rospy.Subscriber('/_annotation_data_annotation_', world_canvas_msgs.msg.Annotation,
#                                                 self.on_annotation, queue_size=1)
#         self.annot_data_sub = rospy.Subscriber('/_annotation_data_old_message_', world_canvas_msgs.msg.AnnotationData,
#                                                 self.on_data_annot, queue_size=1)

    def on_edit_data_srv(self, request):
        # TODO: somehow all should be blocked until we receive this request...
        
        self.annot_name = request.annotation.name
        
        if request.action == EditAnnotationsDataRequest.EDIT:
            msg_class = roslib.message.get_message_class(request.data.type)
            if msg_class is None:
                # This could happen if the message type is wrong or not known for this node (i.e. its
                # package is not on ROS_PACKAGE_PATH). Both cases are really weird in the client side.
                message = "Data type '%s' definition not found" % d.type
                rospy.logerr(message)
                raise Exception(message)
            try:
                old_value = deserializeMsg(request.data.data, msg_class)
            except SerializationError as e:
                message = "Deserialization failed: %s" % str(e)
                rospy.logerr(message)
                raise Exception(message)
            
            self.clean_up_publishers()
            self.message_info = {
                'type_name': request.data.type,
                'instance': old_value,
            }
            self._add_publisher(self.message_info)
            
#             response.action = EditAnnotationsDataResponse.UPDATE
#         else:
#             response.action = EditAnnotationsDataResponse.UPDATE

        while not hasattr(self, 'user_action'):
            rospy.sleep(0.5)

        response = EditAnnotationsDataResponse()
        response.action = self.user_action

        if response.action == EditAnnotationsDataResponse.UPDATE:
            self._fill_message_slots(self.message_info['instance'], self.message_info['topic_name'],
                                     self.message_info['expressions'], self.message_info['counter'])
            response.data.id = request.data.id  # keep same uuid
            response.data.type = self.message_info['type_name']
    
            try:
                response.data.data = serializeMsg(self.message_info['instance'])
            except SerializationError as e:
                message = "Serialization failed: %s" % str(e)
                rospy.logerr(message)
                raise Exception(message)

        return response

    @Slot()
    def accept(self):
        if hasattr(self, 'message_info'):
            self.user_action = EditAnnotationsDataResponse.UPDATE
        else:
            answer = QMessageBox.question(self._widget, 'Delete Existing Message', 
                    'No message under edition. Continue will delete any existing data\n'\
                    'Are you sure?', QMessageBox.Yes, QMessageBox.Cancel)
            if answer != QMessageBox.Yes:
                return
            self.user_action = EditAnnotationsDataResponse.DELETE
        rospy.sleep(0.5)
        self.shutdown_plugin()
        sys.exit(0)

    @Slot()
    def cancel(self):
        self.user_action = EditAnnotationsDataResponse.CANCEL
        rospy.sleep(0.5)
        self.shutdown_plugin()
        sys.exit(0)

    @Slot(str, str, float, bool)
    def msg_type_changed(self, type_name):
        if not type_name:
            if hasattr(self, 'message_info'):
                self._widget.msg_type_combo_box.setEditText(self.message_info['type_name'])
            return    # initialization with empty string; just ignore
        if self._create_message_instance(type_name) is None:
            QMessageBox.critical(self._widget, 'Change Message Type', 
                    'Unrecognized message type', QMessageBox.Ok)
            if hasattr(self, 'message_info'):
                self._widget.msg_type_combo_box.setEditText(self.message_info['type_name'])
            else:
                self._widget.msg_type_combo_box.setEditText('')
            return
        if hasattr(self, 'message_info'):
            if self.message_info['type_name'] == type_name:
                return    # selected same type as current, just ignore

            answer = QMessageBox.question(self._widget, 'Change Message Type', 
                    'Are you sure you want to change current message type?\n'\
                    'All changes will be discarded!', QMessageBox.Ok, QMessageBox.Cancel)
            if answer != QMessageBox.Ok:
                self._widget.msg_type_combo_box.setEditText(self.message_info['type_name'])
                return
        
            self.clean_up_publishers()

        self.message_info = {
            'type_name': str(type_name),
            'instance': self._create_message_instance(str(type_name))
        }
        self._add_publisher(self.message_info)

    def _add_publisher(self, message_info):
        message_info['annot_name'] = self.annot_name
        message_info['topic_name'] = '/__TOPIC__'
        message_info['publisher_id'] = 0
        message_info['counter'] = 0
#        message_info['enabled'] = message_info.get('enabled', False)
        message_info['expressions'] = message_info.get('expressions', {})

        if message_info['instance'] is None:
            raise Exception("Cannot create instance of type '%s'" % message_info['type_name'])
        message_info['publisher'] = \
            rospy.Publisher(message_info['topic_name'], type(message_info['instance']), queue_size=1)

#         # create publisher and timer
#         try:
#             message_info['publisher'] = rospy.Publisher(message_info['topic_name'], type(message_info['instance']), latch=False, queue_size=100)
#         except TypeError:
#             message_info['publisher'] = rospy.Publisher(message_info['topic_name'], type(message_info['instance']), latch=False)
#         message_info['timer'] = QTimer(self)

        # add publisher info to _publishers dict and create signal mapping
#         self._publishers[message_info['publisher_id']] = message_info
#         self._timeout_mapper.setMapping(message_info['timer'], message_info['publisher_id'])
#         message_info['timer'].timeout.connect(self._timeout_mapper.map)
#         if message_info['enabled'] and message_info['rate'] > 0:
#             message_info['timer'].start(int(1000.0 / message_info['rate']))

        self._widget.msg_type_combo_box.setEditText(self.message_info['type_name'])
        self._widget.message_tree_widget.model().add_publisher(message_info)

    @Slot(int, str, str, str, object)
    def change_publisher(self, publisher_id, topic_name, column_name, new_value, setter_callback):
        handler = getattr(self, '_change_publisher_%s' % column_name, None)
        if handler is not None:
            new_text = handler(self.message_info, topic_name, new_value)
            if new_text is not None:
                setter_callback(new_text)

#     def _change_publisher_topic(self, message_info, topic_name, new_value):
#         message_info['enabled'] = (new_value and new_value.lower() in ['1', 'true', 'yes'])
#         #qDebug('MsgEditor._change_publisher_enabled(): %s enabled: %s' % (message_info['topic_name'], message_info['enabled']))
#         if message_info['enabled'] and message_info['rate'] > 0:
#             message_info['timer'].start(int(1000.0 / message_info['rate']))
#         else:
#             message_info['timer'].stop()
#         return None
# 
#     def _change_publisher_type(self, message_info, topic_name, new_value):
#         type_name = new_value
#         # create new slot
#         slot_value = self._create_message_instance(type_name)
# 
#         # find parent slot
#         slot_path = topic_name[len(message_info['topic_name']):].strip('/').split('/')
#         parent_slot = eval('.'.join(["message_info['instance']"] + slot_path[:-1]))
# 
#         # find old slot
#         slot_name = slot_path[-1]
#         slot_index = parent_slot.__slots__.index(slot_name)
# 
#         # restore type if user value was invalid
#         if slot_value is None:
#             qWarning('MsgEditor._change_publisher_type(): could not find type: %s' % (type_name))
#             return parent_slot._slot_types[slot_index]
# 
#         else:
#             # replace old slot
#             parent_slot._slot_types[slot_index] = type_name
#             setattr(parent_slot, slot_name, slot_value)
# 
#             self._widget.message_tree_widget.model().update_publisher(message_info)
# 
#     def _change_publisher_rate(self, message_info, topic_name, new_value):
#         try:
#             rate = float(new_value)
#         except Exception:
#             qWarning('MsgEditor._change_publisher_rate(): could not parse rate value: %s' % (new_value))
#         else:
#             message_info['rate'] = rate
#             #qDebug('MsgEditor._change_publisher_rate(): %s rate changed: %fHz' % (message_info['topic_name'], message_info['rate']))
#             message_info['timer'].stop()
#             if message_info['enabled'] and message_info['rate'] > 0:
#                 message_info['timer'].start(int(1000.0 / message_info['rate']))
#         # make sure the column value reflects the actual rate
#         return '%.2f' % message_info['rate']

    def _change_publisher_expression(self, message_info, topic_name, new_value):
        expression = str(new_value)
        if len(expression) == 0:
            if topic_name in message_info['expressions']:
                del message_info['expressions'][topic_name]
                #qDebug('MsgEditor._change_publisher_expression(): removed expression for: %s' % (topic_name))
        else:
            slot_type, is_array = get_field_type(topic_name)
            if is_array:
                slot_type = list
            # strip possible trailing error message from expression
            error_prefix = '# error'
            error_prefix_pos = expression.find(error_prefix)
            if error_prefix_pos >= 0:
                expression = expression[:error_prefix_pos]
            success, _ = self._evaluate_expression(expression, slot_type)
            if success:
                old_expression = message_info['expressions'].get(topic_name, None)
                message_info['expressions'][topic_name] = expression
                #print 'MsgEditor._change_publisher_expression(): topic: %s, type: %s, expression: %s' % (topic_name, slot_type, new_value)
                self._fill_message_slots(message_info['instance'], message_info['topic_name'], message_info['expressions'], message_info['counter'])
                try:
                    message_info['instance']._check_types()
                except Exception, e:
                    error_str = str(e)
                    print 'serialization error:', error_str
                    if old_expression is not None:
                        message_info['expressions'][topic_name] = old_expression
                    else:
                        del message_info['expressions'][topic_name]
                    return '%s %s: %s' % (expression, error_prefix, error_str)
                return expression
            else:
                return '%s %s evaluating as "%s"' % (expression, error_prefix, slot_type.__name__)

    def _extract_array_info(self, type_str):
        array_size = None
        if '[' in type_str and type_str[-1] == ']':
            type_str, array_size_str = type_str.split('[', 1)
            array_size_str = array_size_str[:-1]
            if len(array_size_str) > 0:
                array_size = int(array_size_str)
            else:
                array_size = 0

        return type_str, array_size

    def _create_message_instance(self, type_str):
        base_type_str, array_size = self._extract_array_info(type_str)

        try:
            base_message_type = roslib.message.get_message_class(base_type_str)
        except ValueError:
            base_message_type = None
        if base_message_type is None:
            print 'Could not create message of type "%s".' % base_type_str
            return None

        if array_size is not None:
            message = []
            for _ in range(array_size):
                message.append(base_message_type())
        else:
            message = base_message_type()
        return message

    def _evaluate_expression(self, expression, slot_type):
        successful_eval = True

        try:
            # try to evaluate expression
            value = eval(expression, {}, self._eval_locals)
        except Exception:
            successful_eval = False

        if slot_type is str:
            if successful_eval:
                value = str(value)
            else:
                # for string slots just convert the expression to str, if it did not evaluate successfully
                value = str(expression)
            successful_eval = True

        elif successful_eval:
            type_set = set((slot_type, type(value)))
            # check if value's type and slot_type belong to the same type group, i.e. array types, numeric types
            # and if they do, make sure values's type is converted to the exact slot_type
            if type_set <= set((list, tuple)) or type_set <= set((int, float)):
                # convert to the right type
                value = slot_type(value)

        if successful_eval and isinstance(value, slot_type):
            return True, value
        else:
            qWarning('MsgEditor._evaluate_expression(): failed to evaluate expression: "%s" as Python type "%s"' % (expression, slot_type.__name__))

        return False, None

    def _fill_message_slots(self, message, topic_name, expressions, counter):
        if topic_name in expressions and len(expressions[topic_name]) > 0:

            # get type
            if hasattr(message, '_type'):
                message_type = message._type
            else:
                message_type = type(message)

            self._eval_locals['i'] = counter
            success, value = self._evaluate_expression(expressions[topic_name], message_type)
            if not success:
                value = message_type()
            return value

        # if no expression exists for this topic_name, continue with it's child slots
        elif hasattr(message, '__slots__'):
            for slot_name in message.__slots__:
                value = self._fill_message_slots(getattr(message, slot_name), topic_name + '/' + slot_name, expressions, counter)
                if value is not None:
                    setattr(message, slot_name, value)

        elif type(message) in (list, tuple) and (len(message) > 0) and hasattr(message[0], '__slots__'):
            for index, slot in enumerate(message):
                self._fill_message_slots(slot, topic_name + '[%d]' % index, expressions, counter)

        return None

    @Slot(int)
    def publish_once(self, message_info):
        # create publisher and timer
        try:
            message_info['publisher'] = rospy.Publisher(message_info['topic_name'], type(message_info['instance']), latch=False, queue_size=1)
        except TypeError:
            message_info['publisher'] = rospy.Publisher(message_info['topic_name'], type(message_info['instance']), latch=False)

        self.message_info['counter'] += 1
        self._fill_message_slots(self.message_info['instance'], self.message_info['topic_name'], self.message_info['expressions'], self.message_info['counter'])
        self.message_info['publisher'].publish(self.message_info['instance'])

#     @Slot(int)
#     def remove_publisher(self):
#         if self.message_info is not None:
# #            self.message_info['timer'].stop()
#             self.message_info['publisher'].unregister()
#             del message_info

    def save_settings(self, plugin_settings, instance_settings):
        pass
#         publisher_copies = []
#         for publisher in self._publishers.values():
#             publisher_copy = {}
#             publisher_copy.update(publisher)
#             publisher_copy['enabled'] = False
#             del publisher_copy['timer']
#             del publisher_copy['instance']
#             del publisher_copy['publisher']
#             publisher_copies.append(publisher_copy)
#         instance_settings.set_value('publishers', repr(publisher_copies))

    def restore_settings(self, plugin_settings, instance_settings):
        pass
#         publishers = eval(instance_settings.value('publishers', '[]'))
#         for publisher in publishers:
#             self._add_publisher(publisher)

    def clean_up_publishers(self):
        if hasattr(self, 'message_info'):
            self._widget.msg_type_combo_box.setEditText('')
            self._widget.message_tree_widget.model().clear()
#            self.message_info['timer'].stop()
            try:
                self.message_info['publisher'].unregister()
            except KeyError:
                pass
            del self.message_info

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()
        self.clean_up_publishers()
