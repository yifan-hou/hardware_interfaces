cmake_minimum_required(VERSION 3.5)
#######################
project(force_control_demo)

include_directories(
  include/
  yaml-cpp
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(force_control_demo src/main.cc)
target_link_libraries(force_control_demo
  HI_COMMON
  UR_RTDE
  ${RUT}
  ${FORCE_CONTROLLERS}
  ${YAMLLib}
)