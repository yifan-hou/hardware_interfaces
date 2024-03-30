cmake_minimum_required(VERSION 3.2)
#######################
project(hardware_interfaces)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_CXX_EXTENSIONS Off)

find_package(Boost REQUIRED COMPONENTS
  system
  thread
  program_options
)

find_library(RUT Utilities HINTS ${CMAKE_INSTALL_PREFIX}/lib/RobotUtilities)

include_directories(
  include/
  robots/ati_netft/include
  robots/ur_rtde/include
  /usr/include/eigen3
)

# comment out / uncomment the library you need
add_subdirectory(robots/ati_netft)
add_subdirectory(robots/ur_rtde)
