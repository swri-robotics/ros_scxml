find_package(Qt5Test REQUIRED)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

add_executable(${PROJECT_NAME}_utest utest.cpp)
target_link_libraries(${PROJECT_NAME}_utest PRIVATE ${PROJECT_NAME} Qt5::Test)
target_compile_definitions(${PROJECT_NAME}_utest PRIVATE SCXML_FILE="${CMAKE_CURRENT_SOURCE_DIR}/demo_sm.scxml")
target_cxx_version(${PROJECT_NAME}_utest PRIVATE VERSION 14)

add_test(NAME run_tests COMMAND ${PROJECT_NAME}_utest)

install(TARGETS ${PROJECT_NAME}_utest
  RUNTIME DESTINATION bin/tests
  LIBRARY DESTINATION lib/tests
  ARCHIVE DESTINATION lib/tests
)
