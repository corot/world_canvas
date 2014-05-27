World canvas server
===================

A component of the world canvas framework, a ROS stack for storing and accessing semantic information about the world, with an initial emphasis on needs and use cases for mobile robots.

World canvas server is a storage manager for semantic maps. Initial version replicates [map_store](https://github.com/ros-planning/map_store) behavior for semantic maps.


Give it a fast try
------------------

First you need to populate the database with some annotations (we still don't have an editor :(
Use the save_xxxx.py scripts and the test/annotations/xxx_list.yaml data files from world_canvas_server package, e.g.

```
rosrun world_canvas_server save_markers.py _world_id:='70a98ad3-78be-45eb-85f7-d2f14e81d95a' _filename:=$HOME/semantic_maps/src/world_canvas/world_canvas_server/test/annotations/ar_list.yaml
```

Then, launch the annotations server (the Python version)


```
roslaunch world_canvas_server py_annotations_server.launch --screen
```

And you are ready to pull for annotations. Until we have an operation client library, use the get_any.py script from world_canvas_server package, e.g.
```
rosrun world_canvas_server get_any.py  _world_id:='70a98ad3-78be-45eb-85f7-d2f14e81d95a' _ids:='[]' _types:=['wall','xxx'] _keywords:='[]' _topic_type:='yocs_msgs/WallList' _topic_name:=wall_pose_list _pub_as_list:=True
```
The first 4 parameters provide search criteria:
 * world_id   Retrieved annotations associated to this world
 * ids        Retrieved annotation uuid is within this list
 * types      Retrieved annotation type is within this list
 * keywords   Retrieved annotation has AT LEAST one of these keywords

And the other 3 define how the server must publish the retrieved annotations
 * topic_name
 * topic_type
 * pub_as_list   If true, annotations will be packed in a list before publishing, so topic_type must be an array of the requested annotations

The get_any script calls 3 services in the annotations server:
 * get annotations to retrieve the annotations that satisfy the filter criteria
 * get annotations data to retrieve the data of those annotations
 * pub annotations data to make the server publish such data
