cmake_minimum_required(VERSION 2.8.3)
project(robot_editor)

find_package(catkin REQUIRED COMPONENTS
  rviz
  urdf
  tf
  kdl_parser
  robot_state_publisher
)

find_package(Boost REQUIRED COMPONENTS thread)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)

# catkin_python_setup()

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${QT_INCLUDES}
  ${CMAKE_CURRENT_BINARY_DIR}
)

qt4_wrap_cpp(MOC_FILES
  src/robot_editor.h
  src/robot_preview.h
)
qt4_wrap_ui(main_window_HEADERS ui/main_window.ui)

set(SOURCE_FILES
  src/main.cpp
  src/robot_editor.cpp
  src/robot_preview.cpp
  ${MOC_FILES}
  ${main_window_HEADERS}
)

add_executable(robot_editor ${SOURCE_FILES})
target_link_libraries(robot_editor ${QT_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

#############
## Testing ##
#############