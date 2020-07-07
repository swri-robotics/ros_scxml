cmake_minimum_required(VERSION 3.1.0)
project(test_scxml_core)

if (NOT $ENV{ROS_VERSION} EQUAL "1")
  message(WARNING "ROS Version $ENV{ROS_VERSION} found, skipping project ${PROJECT_NAME}")
  return()
endif()

## Compile as C++14, supported in ROS Kinetic and newer


find_package(catkin REQUIRED COMPONENTS
    roslib
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


if (CATKIN_ENABLE_TESTING)

    
    find_package(scxml_core REQUIRED)



    catkin_add_gtest(test_sm test/test_sm.cpp)
    target_link_libraries(test_sm scxml_core::scxml_core ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
endif()