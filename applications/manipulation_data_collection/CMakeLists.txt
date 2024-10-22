cmake_minimum_required(VERSION 3.5)
#######################
project(manipulation_data_collection)

find_package(OpenCV REQUIRED)

include_directories(
  include/
  yaml-cpp
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(manipulation_data_collection src/main.cc)
target_link_libraries(manipulation_data_collection
  ManipServer
  ${RUT}
  ${YAMLLib}
  ${OpenCV_LIBS}
)
