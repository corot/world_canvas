#!/usr/bin/env python

import rospy
import world_canvas_msgs.srv


if __name__ == '__main__':
    rospy.init_node('export')
    
    filename = rospy.get_param('~filename')

    rospy.loginfo("Waiting for yaml_export service...")
    rospy.wait_for_service('yaml_export')

    rospy.loginfo("Export annotations to %s", filename)
    export_srv = rospy.ServiceProxy('yaml_export', world_canvas_msgs.srv.YAMLExport)
    response = export_srv(filename)

    if response.result == True:
        rospy.loginfo("Database successfully exported to %s", filename)
    else:
        rospy.logerr("Export database failed; %s", response.message)
