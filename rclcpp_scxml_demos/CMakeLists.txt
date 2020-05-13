cmake_minimum_required(VERSION 3.5)
project(rclcpp_scxml_demos)

if (NOT $ENV{ROS_VERSION} EQUAL "2")
  message(WARNING "ROS Version $ENV{ROS_VERSION} found, skipping project ${PROJECT_NAME}")
  return()
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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

# build
list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_14 CXX_FEATURE_FOUND) 

# Demo Executable
add_executable(demo_scxml_state_machine
  src/demo_scxml_sm.cpp
)
target_link_libraries(demo_scxml_state_machine
  scxml_core::scxml_core
  Qt5::Widgets
)

target_include_directories(demo_scxml_state_machine PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(demo_scxml_state_machine SYSTEM PUBLIC)
ament_target_dependencies(demo_scxml_state_machine std_msgs std_srvs rclcpp rclcpp_components)
install(TARGETS demo_scxml_state_machine
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}/
  PATTERN ".svn" EXCLUDE
)

ament_export_dependencies(ament_cmake)
ament_package()
