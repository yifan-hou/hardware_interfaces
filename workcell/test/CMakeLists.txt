cmake_minimum_required(VERSION 3.5)
#######################
project(test)

find_package(Eigen3 REQUIRED)
find_library(RUT Utilities HINTS ${CMAKE_INSTALL_PREFIX}/lib/RobotUtilities)

include_directories(
  include/
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(TestServer SHARED src/test_server.cc)
target_link_libraries(TestServer PRIVATE
  ${RUT}
)

# c++ library install
install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
)
install(TARGETS TestServer
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
)

pybind11_add_module(test_server_pybind 
  src/test_server_pybind.cc
)
target_link_libraries(test_server_pybind PRIVATE
  TestServer
  ${RUT}
)

# set the output directory for the pybind module
set_target_properties(test_server_pybind PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/python")
set_target_properties(test_server_pybind PROPERTIES PREFIX "")
set_target_properties(test_server_pybind PROPERTIES OUTPUT_NAME "test_server_pybind")