cmake_minimum_required(VERSION 3.1.0)
project(ros_scxml)

## Compile as C++14, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#### QT setup
# Create code from a list of Qt designer ui files
#set(CMAKE_AUTORCC ON)

# Find includes in corresponding build directories
#set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed
#set(CMAKE_AUTOMOC ON)
# Create code from a list of Qt designer ui files
#set(CMAKE_AUTOUIC ON)

# Find includes in corresponding build directories
#set(CMAKE_INCLUDE_CURRENT_DIR ON)

#list(APPEND CMAKE_PREFIX_PATH /opt/ros/kinetic)
#list(APPEND CMAKE_PREFIX_PATH /opt/qt/5.12.2/gcc_64)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

#include(QtInstallPaths) #to set QT_INSTALL_FOO variables
find_package(Qt5 REQUIRED COMPONENTS
  Core
  Widgets
  Scxml
)

add_message_files(
  FILES
  ActiveStates.msg
)

add_service_files(
  FILES
  TriggerEvent.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    std_msgs
    message_runtime
  DEPENDS
    Qt5Core
    Qt5Widgets
    Qt5Scxml
)

set(ROS_SCXML_HEADERS
  include/ros_scxml/state_machine.h
  include/ros_scxml_state.h
  include/ros_scxml.h
)

qt5_wrap_cpp(ROS_SCXML_MOCS ${ROS_SCXML_HEADERS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5Scxml_INCLUDE_DIRS}
  ${Qt5Scxml_PRIVATE_INCLUDE_DIRS}
)

# State machine library
add_library(${PROJECT_NAME}
  src/state_machine.cpp
  src/ros_scxml.cpp
  src/ros_scxml_state.cpp  
  ${ROS_SCXML_MOCS}
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  ${Qt5Scxml_LIBRARIES}
)

add_executable(demo_scxml_state_machine
  src/demo_scxml_sm.cpp
)
add_dependencies(demo_scxml_state_machine  ${catkin_EXPORTED_TARGETS})

# Link libraries
target_link_libraries(demo_scxml_state_machine
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  ${Qt5Scxml_LIBRARIES}
)

