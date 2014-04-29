cmake_minimum_required(VERSION 2.8.3)
project(semantic_map_store)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rostest
  warehouse_ros
  semantic_map_msgs
  visualization_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

#add_executable(semantic_map_saver src/map_saver.cpp)
#add_dependencies(semantic_map_saver semantic_map_msgs_gencpp)
#target_link_libraries(semantic_map_saver uuid ${catkin_LIBRARIES})

add_executable(semantic_map_manager src/map_manager.cpp)
add_dependencies(semantic_map_manager semantic_map_msgs_gencpp)
target_link_libraries(semantic_map_manager uuid ${catkin_LIBRARIES})

add_rostest(test/semantic_map_store.test)

install(TARGETS #semantic_map_saver
	            semantic_map_manager
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS scripts/add_map.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
