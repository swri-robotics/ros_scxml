cmake_minimum_required(VERSION 3.5)
project(rclcpp_scxml_demos)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#### QT setup
#find_package(Qt5 REQUIRED COMPONENTS
#  Core
#  Widgets
#  Scxml
#)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(scxml_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

if( Log4cxx_DIR )
  find_package( Log4cxx NO_MODULE )
elseif( NOT Log4cxx_FOUND )
  message(STATUS "Searching for log4cxx/logger.h")
  find_path( Log4cxx_INCLUDE_DIR log4cxx/logger.h )

  message(STATUS "Searching for libLog4cxx")
  find_library( Log4cxx_LIBRARY log4cxx )

  include( FindPackageHandleStandardArgs )
  FIND_PACKAGE_HANDLE_STANDARD_ARGS( Log4cxx Log4cxx_INCLUDE_DIR Log4cxx_LIBRARY )
  if( LOG4CXX_FOUND )
    set( Log4cxx_FOUND TRUE )
  endif()
endif()

#include_directories(
#  include
#  SYSTEM
#  ${Qt5Core_INCLUDE_DIRS}
#  ${Qt5Widgets_INCLUDE_DIRS}
#  ${Qt5Scxml_INCLUDE_DIRS}
#  ${Qt5Scxml_PRIVATE_INCLUDE_DIRS}
#)
#include_directories(
#  SYSTEM
#  scxml_core_INCLUDE_DIRS
#)

list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_14 CXX_FEATURE_FOUND) 

include_directories(
  SYSTEM
  ${scxml_core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

# Demo Executable
add_executable(demo_scxml_state_machine
  src/demo_scxml_sm.cpp
)
target_link_libraries(demo_scxml_state_machine
  ${scxml_core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
)
#target_include_directories(demo_scxml_state_machine PUBLIC
#    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#    "$<INSTALL_INTERFACE:include>")
#target_include_directories(demo_scxml_state_machine SYSTEM PUBLIC ${Qt5Core_INCLUDE_DIRS})
ament_target_dependencies(demo_scxml_state_machine std_msgs std_srvs rclcpp rclcpp_components)

install(TARGETS
    demo_scxml_state_machine
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}/
  PATTERN ".svn" EXCLUDE
)
