cmake_minimum_required(VERSION 3.5)
project(ros_scxml)

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
find_package(Qt5 REQUIRED COMPONENTS
  Core
  Widgets
  Scxml
)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

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

include_directories(
  include
  SYSTEM
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5Scxml_INCLUDE_DIRS}
  ${Qt5Scxml_PRIVATE_INCLUDE_DIRS}
)

set(ROS_SCXML_HEADERS
  include/ros_scxml/state_machine.h
)

qt5_wrap_cpp(ROS_SCXML_MOCS ${ROS_SCXML_HEADERS}
)

# State machine library
add_library(${PROJECT_NAME} SHARED
  src/state_machine.cpp
  ${ROS_SCXML_MOCS} 
)
target_link_libraries(${PROJECT_NAME}
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  ${Qt5Scxml_LIBRARIES}  
  ${Log4cxx_LIBRARY}
)

# Demo Executable
add_executable(demo_scxml_state_machine
  src/demo_scxml_sm.cpp
)
target_link_libraries(demo_scxml_state_machine
  ${PROJECT_NAME}
)
ament_target_dependencies(demo_scxml_state_machine std_msgs std_srvs rclcpp rclcpp_components)

install(TARGETS
    ${PROJECT_NAME}
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
