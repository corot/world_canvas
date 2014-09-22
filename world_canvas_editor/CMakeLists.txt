cmake_minimum_required(VERSION 2.8.3)
project(world_canvas_editor)
find_package(catkin REQUIRED COMPONENTS rviz roscpp topic_tools world_canvas_msgs world_canvas_client_cpp unique_id uuid_msgs yocs_math_toolkit



yocs_msgs)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS} ${QT_INCLUDE_DIR})
link_directories(${catkin_LIBRARY_DIRS})

## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt4_wrap_cpp(MOC_FILES
  src/annotations.hpp
  src/editor_panel.hpp
)

qt4_wrap_ui(UIC_FILES
  src/ui/editor_panel.ui
)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/annotations.cpp
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
##
## Although this puts "rviz_plugin_tutorials" (or whatever you have
## called the project) as the name of the library, cmake knows it is a
## library and names the actual file something like
## "librviz_plugin_tutorials.so", or whatever is appropriate for your
## particular OS.
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} uuid_msgs_gencpp world_canvas_msgs_gencpp)

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

install(DIRECTORY media/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)

install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)

install(PROGRAMS scripts/send_test_msgs.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

