cmake_minimum_required(VERSION 3.5)
#######################
project(ft_calibration)

find_package(Boost REQUIRED)

include_directories(
  include/
  yaml-cpp
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(ft_calibration src/main.cc)
target_link_libraries(ft_calibration
  HI_COMMON
  UR_RTDE
  ROBOTIQ_FT_HW
  ATI_NETFT
  ${RUT}
  ${FORCE_CONTROLLERS}
  ${YAMLLib}
)