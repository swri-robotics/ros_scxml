cmake_minimum_required(VERSION 3.5)
project(scxml_core VERSION 0.2.0 LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

find_package(cmake_common_scripts REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Scxml)
find_package(TinyXML2 REQUIRED)

###########
## Build ##
###########

add_library(${PROJECT_NAME} SHARED
  src/scxml_sm_interface.cpp
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Qt5::Scxml
  ${TinyXML2_LIBRARIES}
)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION 14)

#############
## Install ##
#############

configure_package(NAMESPACE scxml_core TARGETS ${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
)

##########
## Test ##
##########

if(SCXML_CORE_ENABLE_TESTING)
  add_subdirectory(test)
endif()
