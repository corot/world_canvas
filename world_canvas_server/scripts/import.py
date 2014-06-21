#!/usr/bin/env python

import rospy
import uuid
import unique_id
import world_canvas_msgs.srv

from rospy_message_converter import message_converter


if __name__ == '__main__':
    rospy.init_node('import')
    
    filename = rospy.get_param('~filename')

    rospy.loginfo("Waiting for yaml_import service...")
    rospy.wait_for_service('yaml_import')

    rospy.loginfo("Import annotations from %s", filename)
    import_srv = rospy.ServiceProxy('yaml_import', world_canvas_msgs.srv.YAMLImport)
    response = import_srv(filename)

    if response.result == True:
        rospy.loginfo("Database successfully imported from %s", filename)
    else:
        rospy.logerr("Import database failed; %s", response.message)

