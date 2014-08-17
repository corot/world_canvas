#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2014, Yujin Robot
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Jorge Santos

import os
import rospy

from qt_gui.plugin import Plugin
from .map_view import NavViewWidget
from .msg_widget import PublisherWidget

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

class EditorPlugin(Plugin):
    def __init__(self, context):
        super(EditorPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MainGUI')

        # Create NavViewWidget
        self._widget = NavViewWidget()
        # Get path to UI file which is a sibling of this file
#        ui_file = os.path.join(self._rospack.get_path('rqt_world_editor'), 'resource', 'world_editor.ui')
        # Extend the widget with all attributes and children from UI file
#        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('Map View')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        
        # Create a message view widget
        self._msg_widget = PublisherWidget()
#         self._msg_widget.add_publisher.connect(self.add_publisher)
#         self._msg_widget.change_publisher.connect(self.change_publisher)
#         self._msg_widget.publish_once.connect(self.publish_once)
#         self._msg_widget.remove_publisher.connect(self.remove_publisher)
#         self._msg_widget.clean_up_publishers.connect(self.clean_up_publishers)
        if context.serial_number() > 1:
            self._msg_widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        context.add_widget(self._msg_widget)


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        self._widget.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        self._widget.restore_settings(plugin_settings, instance_settings)

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog



#  no command line args by now
        # Process standalone plugin command-line arguments
#         from argparse import ArgumentParser
#         parser = ArgumentParser()
#         # Add argument(s) to the parser.
#         parser.add_argument("-q", "--quiet", action="store_true",
#                       dest="quiet",
#                       help="Put plugin in silent mode")
#         args, unknowns = parser.parse_known_args(context.argv())
#         if not args.quiet:
#             print 'arguments: ', args
#             print 'unknowns: ', unknowns

    
#     def __init__(self, context):
#         super(EditorPlugin, self).__init__(context)
#         self.setObjectName('Action')
#         self._widget = MessagesWidget(rosaction.MODE_ACTION)
#         self._widget.setWindowTitle('Action Type Browser')
#         if context.serial_number() > 1:
#             self._widget.setWindowTitle(self._widget.windowTitle() +
#                                         (' (%d)' % context.serial_number()))
#         context.add_widget(self._widget)
# 
#     def shutdown_plugin(self):
#         self._widget.cleanup_browsers_on_close()
# 
#     def save_settings(self, plugin_settings, instance_settings):
#         # instance_settings.set_value(k, v)
#         pass
# 
#     def restore_settings(self, plugin_settings, instance_settings):
#         # v = instance_settings.value(k)
#         pass
