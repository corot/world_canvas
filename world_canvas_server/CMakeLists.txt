cmake_minimum_required(VERSION 2.8.3)
project(world_canvas_server)

find_package(catkin REQUIRED)

catkin_package()


#install(PROGRAMS scripts/add_map.py
#        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
