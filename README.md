World canvas server
===================

A component of the world canvas framework, a ROS stack for storing and accessing semantic information about the world, with an initial emphasis on needs and use cases for mobile robots.

World canvas server is a storage manager for semantic maps. Initial version replicates [map_store](https://github.com/ros-planning/map_store) behavior for semantic maps.


Give it a fast try
------------------

**Installing**

Use world_canvas.rosinstall to create a world_canvas workspace. Don't forget to install dependencies:

```
rosdep install --from-paths -i src
```

**Running**

First of all, launch the annotations server (the Python version)

```
roslaunch world_canvas_server py_annotations_server.launch --screen
```

Then you need to populate the database with some annotations (we still don't have an editor :(
Use the save_xxxx.py scripts and the test/annotations/xxx_list.yaml data files from world_canvas_server package, e.g.

```
> rosrun world_canvas_server save_markers.py _world_id:='70a98ad3-78be-45eb-85f7-d2f14e81d95a' _filename:=`rospack find world_canvas_server`/test/annotations/ar_list.yaml
> rosrun world_canvas_server save_walls.py _world_id:='70a98ad3-78be-45eb-85f7-d2f14e81d95a' _filename:=`rospack find world_canvas_server`/test/annotations/wall_list.yaml
```

These scripts will request the server to save in database both the annotations and the associated data the YAML files contain.

And now you are ready to look for annotations. Until we have an operating client library, use the get_any.py script from world_canvas_server package, e.g.
```
> rosrun world_canvas_server get_any.py  _world_id:='70a98ad3-78be-45eb-85f7-d2f14e81d95a' _ids:=[] _types:=['ar_track_alvar/AlvarMarker'] _keywords:=[] _relationships=[] _topic_type:=ar_track_alvar/AlvarMarker _topic_name:=ar_markers _pub_as_list:=False
> rosrun world_canvas_server get_any.py  _world_id:='70a98ad3-78be-45eb-85f7-d2f14e81d95a' _ids:=[] _types:=['yocs_msgs/Wall'] _keywords:=[] _relationships=[] _topic_type:=yocs_msgs/WallList _topic_name:=wall_pose_list _pub_as_list:=True
```
The first 5 parameters provide search criteria:
 * world_id   Retrieved annotations associated to this world. Mandatory
 * ids        Retrieved annotation uuid is within this list
 * types      Retrieved annotation type is within this list
 * keywords   Retrieved annotation has AT LEAST one of these keywords
 * relationships   Still not implemented on server, so just ignored

And the other 3 define how the server must publish the retrieved annotations
 * topic_name    Where server must publish annotations data
 * topic_type    The message type to publish annotations data. Mandatory if pub_as_list is true; ignored otherwise
 * pub_as_list   If true, annotations will be packed in a list before publishing, so topic_type must be an array of the requested annotations

The get_any script calls 3 services in the annotations server:
 * get annotations to retrieve the annotations that satisfy the filter criteria
 * get annotations data to retrieve the data of those annotations
 * pub annotations data to make the server publish such data

You can verify that all went right by echoing topics
 * topic_name                Published by the server with the loaded annotations data
 * topic_name + '_client'    Published by the get_Any script with the retrieved annotations data

Or in RViz showing topic
 * topic_name + '_markers'   Published by the get_Any script with visualization markers


**Import/export database**

The save_xxxx.py scripts are a temporal workaround to play with world canvas. The canonical ways to populate the
database are a graphic tool still do be implemented and a import from YAML file service. To try the later method,
and the reverse operation of exporting to file, you can run the import.py/export.py scripts:

```
rosrun world_canvas_server import.py _filename:=<world canvas workspace>/src/world_canvas/world_canvas_server/test/annotations/full_db.yaml
rosrun world_canvas_server export.py _filename:=<world canvas workspace>/src/world_canvas/world_canvas_server/test/annotations/export_db.yaml
```
