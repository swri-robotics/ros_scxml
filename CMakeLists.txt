cmake_minimum_required(VERSION 3.1.0)
project(packml)

## Compile as C++14, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Instruct CMake to run moc automatically when needed
#set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTOMOC_MOC_OPTIONS "-I include")
# Create code from a list of Qt designer ui files
set(CMAKE_AUTORCC ON)
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

list(APPEND CMAKE_PREFIX_PATH /opt/ros/kinetic)
list(APPEND CMAKE_PREFIX_PATH /opt/Qt5.12.2/5.12.2/gcc_64)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

find_package(Qt5 REQUIRED COMPONENTS
  Core
  Widgets
  Scxml
)

add_message_files(
  FILES
  PackMLActiveStates.msg
)

generate_messages()


catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_SM
  CATKIN_DEPENDS
    roscpp
    std_msgs
    message_runtime
  DEPENDS
    Qt5
)

qt5_wrap_cpp(PACKML_MOC
  include/packml_state.h
  include/packml.h
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

# State machine library
add_library(${PROJECT_NAME}_SM
  ${PACKML_MOC}
  src/packml.cpp
  src/packml_state.cpp
  resource/resource.qrc
)

target_link_libraries(${PROJECT_NAME}_SM
  ${catkin_LIBRARIES}
  Qt5::Core
  Qt5::Scxml
  Qt5::Widgets
)

add_executable(${PROJECT_NAME}_node
  src/packml_sm.cpp
)
# Link libraries
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_SM
  Qt5::Core
  Qt5::Scxml
  Qt5::Widgets
)

