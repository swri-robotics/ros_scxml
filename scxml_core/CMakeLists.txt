cmake_minimum_required(VERSION 3.5)
project(scxml_core)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#### QT setup
find_package(Qt5 REQUIRED COMPONENTS
  Core
  Widgets
  Scxml
)


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

set(SCXML_CORE_HEADERS
  include/scxml_core/state_machine.h
)
qt5_wrap_cpp(SCXML_CORE_MOCS ${SCXML_CORE_HEADERS}
)

# build
list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_14 CXX_FEATURE_FOUND)

add_library(${PROJECT_NAME} SHARED
  src/state_machine.cpp
  ${SCXML_CORE_MOCS} 
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  ${Qt5Scxml_LIBRARIES}  
  ${Log4cxx_LIBRARY}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${Boost_LIBRARIES}
  ${pluginlib_LIBRARIES}
  ${PCL_LIBRARIES}
  ${xmlrpcpp_LIBRARIES}
)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5Scxml_INCLUDE_DIRS}
  ${Qt5Scxml_PRIVATE_INCLUDE_DIRS}
  ${Log4cxx_INCLUDE_DIRS}
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO)
if(CXX_FEATURE_FOUND EQUAL "-1")
  target_compile_options(${PROJECT_NAME} PRIVATE -std=c++14)
else()
  target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_14)
endif()

#install
install(TARGETS
    ${PROJECT_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

