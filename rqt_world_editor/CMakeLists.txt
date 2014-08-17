cmake_minimum_required(VERSION 2.8.3)
project(rqt_world_editor)
find_package(catkin REQUIRED)

catkin_package()
catkin_python_setup()

#############
## Install ##
#############

install(PROGRAMS scripts/rqt_world_editor
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
