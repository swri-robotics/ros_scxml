cmake_minimum_required(VERSION 3.5)

# Extract package name and version from package.xml
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)

project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

find_package(Qt5 REQUIRED COMPONENTS Core Scxml)
find_package(TinyXML2 REQUIRED)

# ######################################################################################################################
# Build ##
# ######################################################################################################################

add_library(${PROJECT_NAME} SHARED src/scxml_sm_interface.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC Qt5::Scxml Qt5::Core ${TinyXML2_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include>")
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION 14)

# Demo executable
add_executable(${PROJECT_NAME}_demo src/demo.cpp)
target_link_libraries(${PROJECT_NAME}_demo PUBLIC ${PROJECT_NAME} Qt5::Core)
target_cxx_version(${PROJECT_NAME}_demo PUBLIC VERSION 14)

# ######################################################################################################################
# Install ##
# ######################################################################################################################

install(DIRECTORY include/ DESTINATION include)

install(FILES "${CMAKE_CURRENT_LIST_DIR}/cmake/FindTinyXML2.cmake" DESTINATION lib/cmake/${PROJECT_NAME})

configure_package(NAMESPACE scxml_core DEPENDENCIES "Qt5 REQUIRED COMPONENTS Core Scxml" TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_demo)

# ######################################################################################################################
# Test ##
# ######################################################################################################################

if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()
