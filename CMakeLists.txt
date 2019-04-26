cmake_minimum_required(VERSION 3.1.0)
project(ros_scxml)

## Compile as C++14, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
)

#### QT setup
find_package(Qt5 REQUIRED COMPONENTS
  Core
  Widgets
  Scxml
)

catkin_package(
  INCLUDE_DIRS
    include   
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5Scxml_INCLUDE_DIRS}
    ${Qt5Scxml_PRIVATE_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}    
  CATKIN_DEPENDS
    roscpp
  DEPENDS
    Qt5Core
    Qt5Widgets
    Qt5Scxml
)

set(ROS_SCXML_HEADERS
  include/ros_scxml/state_machine.h
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
  ${ROS_SCXML_MOCS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  ${Qt5Scxml_LIBRARIES}
)

# Demo Executable
add_executable(demo_scxml_state_machine
  src/demo_scxml_sm.cpp
)

target_link_libraries(demo_scxml_state_machine
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

