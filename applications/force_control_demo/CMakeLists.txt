cmake_minimum_required(VERSION 3.5)
#######################
project(force_control_demo)

include_directories(
  include/
  yaml-cpp
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(ur_demo src/demo_with_ur.cc)
target_link_libraries(ur_demo
  HI_COMMON
  UR_RTDE
  ${RUT}
  ${FORCE_CONTROLLERS}
  ${YAMLLib}
)

add_executable(coinft_demo src/demo_with_coinft.cc)
target_link_libraries(coinft_demo
  HI_COMMON
  UR_RTDE
  COIN_FT
  ${RUT}
  ${FORCE_CONTROLLERS}
  ${YAMLLib}
)