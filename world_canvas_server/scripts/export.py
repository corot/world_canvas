#!/usr/bin/env python

import rospy
import uuid
import unique_id
import world_canvas_msgs.srv

from rospy_message_converter import message_converter


if __name__ == '__main__':
    rospy.init_node('export')
    
    filename = rospy.get_param('~filename')

    rospy.loginfo("Waiting for yaml_export service...")
    rospy.wait_for_service('yaml_export')

    rospy.loginfo("Export annotations from %s", filename)
    export_srv = rospy.ServiceProxy('yaml_export', world_canvas_msgs.srv.YAMLExport)
    response = export_srv(filename)

    if response.result == True:
        rospy.loginfo("Database successfully exported to %s", filename)
    else:
        rospy.logerr("Export database failed; %s", response.message)
