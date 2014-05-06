cmake_minimum_required(VERSION 2.8.3)
project(world_canvas_server)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rostest
  warehouse_ros
  world_canvas_msgs
  visualization_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

#add_executable(world_canvas_saver src/map_saver.cpp)
#add_dependencies(world_canvas_saver world_canvas_msgs_gencpp)
#target_link_libraries(world_canvas_saver uuid ${catkin_LIBRARIES})
#
#add_executable(world_canvas_manager src/map_manager.cpp)
#add_dependencies(world_canvas_manager world_canvas_msgs_gencpp)
#target_link_libraries(world_canvas_manager uuid ${catkin_LIBRARIES})

add_executable(annotations_server src/annotations_server.cpp)
add_dependencies(annotations_server world_canvas_msgs_gencpp)
target_link_libraries(annotations_server uuid ${catkin_LIBRARIES})

add_rostest(test/world_canvas_server.test)

install(TARGETS #world_canvas_saver
	            #world_canvas_manager
	            annotations_server
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS scripts/add_map.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
