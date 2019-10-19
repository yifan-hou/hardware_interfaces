cmake_minimum_required(VERSION 2.8.3)
#######################
project(hardware_interfaces)
set (CMAKE_CXX_FLAGS "--std=gnu++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  diagnostic_updater
  diagnostic_msgs
  geometry_msgs
  message_generation
)

find_package(Boost REQUIRED COMPONENTS
  system
  thread
  program_options
)

find_library(RUT Utilities HINTS /usr/local/lib/RobotUtilities)


include_directories(
  include/
  robots/abb_egm/include
  robots/ati_netft/include
  /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
)

#
#   Installation for ROS.
#
catkin_package(
  INCLUDE_DIRS include robots/abb_egm/include robots/ati_netft/include robots/ur_socket/include ${PROJECT_BINARY_DIR}
  LIBRARIES ABB_EGM ATI_NETFT_HW UR_SOCKET
  CATKIN_DEPENDS roscpp diagnostic_updater diagnostic_msgs geometry_msgs message_runtime
)

add_library(DELTA_INTERFACES src/delta_interfaces.cpp)

#add_subdirectory(robots/abb_egm)
#add_subdirectory(robots/ati_netft)
#add_subdirectory(robots/ur_socket)
add_subdirectory(robots/delta_dji)
