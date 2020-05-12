cmake_minimum_required(VERSION 3.1.0)
project(roscpp_scxml_demos)

if (NOT $ENV{ROS_VERSION} EQUAL "1")
  message(WARNING "ROS Version $ENV{ROS_VERSION} found, skipping project ${PROJECT_NAME}")
  return()
endif()

## Compile as C++14, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
)

find_package(scxml_core REQUIRED)

#### QT setup
find_package(Qt5 REQUIRED COMPONENTS
  Widgets
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES  
  CATKIN_DEPENDS
    roscpp
    std_msgs
    std_srvs
  DEPENDS
  	scxml_core
    Qt5Widgets
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

# Demo Executable
add_executable(demo_scxml_state_machine
  src/demo_scxml_sm.cpp
)
target_link_libraries(demo_scxml_state_machine
  ${catkin_LIBRARIES}
  scxml_core::scxml_core
  Qt5::Widgets
)

install(TARGETS demo_scxml_state_machine
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY resource/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resource
  PATTERN ".svn" EXCLUDE
)
