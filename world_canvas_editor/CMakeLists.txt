cmake_minimum_required(VERSION 2.8.3)
project(world_canvas_editor)
find_package(catkin REQUIRED COMPONENTS rviz roscpp nav_msgs map_server topic_tools world_canvas_msgs world_canvas_client_cpp yocs_math_toolkit)

# pkg-config support for yaml-cpp
find_package(PkgConfig)
pkg_search_module(yaml-cpp REQUIRED yaml-cpp)

catkin_package()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(${catkin_INCLUDE_DIRS} ${QT_INCLUDE_DIR} include)
link_directories(${catkin_LIBRARY_DIRS})

## This package includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

## Add a QT_NO_KEYWORDS definition to avoid Qt signals and slots to define
## "emit", "slots", etc. because they can conflict with boost signals
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt4_wrap_cpp(MOC_FILES
  include/world_canvas_editor/annotations.hpp
  include/world_canvas_editor/worlds_list.hpp
  include/world_canvas_editor/editor_panel.hpp
)

qt4_wrap_ui(UIC_FILES
  src/ui/editor_panel.ui
)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/annotations.cpp
  src/worlds_list.cpp
  src/editor_panel.cpp 
  ${MOC_FILES}
)

## An rviz plugin is just a shared library, so here we declare the library
## to be called ``${PROJECT_NAME}`` (which is "world_canvas_editor") and
## specify the list of source files we collected above in ``${SOURCE_FILES}``.
add_library(${PROJECT_NAME} ${SOURCE_FILES} ${MOC_SOURCES} ${UIC_FILES})

## Link the library with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above, and with whatever libraries
## catkin has included.
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} world_canvas_msgs_gencpp)

## Install rules

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES 
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#install(DIRECTORY media/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)
#
#install(DIRECTORY icons/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)
