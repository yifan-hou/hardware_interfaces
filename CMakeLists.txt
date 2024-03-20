cmake_minimum_required(VERSION 2.8.3)
#######################
project(hardware_interfaces)
set (CMAKE_CXX_FLAGS "--std=gnu++17 ${CMAKE_CXX_FLAGS}")

find_package(Boost REQUIRED COMPONENTS
  system
  thread
  program_options
)

find_library(RUT Utilities HINTS ${CMAKE_INSTALL_PREFIX}/lib/RobotUtilities)

include_directories(
  include/
  robots/abb_egm/include
  robots/ati_netft/include
  /usr/include/eigen3
)

# comment out / uncomment the library you need
#add_subdirectory(robots/abb_egm)
add_subdirectory(robots/ati_netft)
add_subdirectory(robots/ur_socket)
