#!/usr/bin/env python

PKG = 'annotations_store'
import roslib; roslib.load_manifest(PKG)
import rospy
import sys
import annotations_store.srv

save_as = None
try:
    save_as = sys.argv[1]
except:
    print "You must specify a name for the map on the server"
    sys.exit(1)

print "Waiting for /dynamic_map..."
rospy.wait_for_service('/dynamic_map')
print "Waiting for /save_map..."
rospy.wait_for_service('/save_map')
print "Waiting for /list_maps..."
rospy.wait_for_service('/list_maps')
print "Waiting for /delete_map..."
rospy.wait_for_service('/delete_map')

print "Checking for duplicates..."
list_last_maps = rospy.ServiceProxy('/list_maps', annotations_store.srv.ListMaps)
delete_map = rospy.ServiceProxy('/delete_map', annotations_store.srv.DeleteMap)
maps = []
try:
    maps = list_last_maps().map_list
except:
    print "Getting maps from the annotations_manager has failed"
    sys.exit(2)

for i in maps:
    if (i.name == save_as):
        print "Deleting map", i.map_id
        delete_map(i.map_id)

print "Starting service..."
save_map = rospy.ServiceProxy('/save_map', annotations_store.srv.SaveMap)
print "Saving map as", save_as
save_map(save_as)
print "Done"

