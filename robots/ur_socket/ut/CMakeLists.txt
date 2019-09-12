cmake_minimum_required(VERSION 2.8.3)
project(ur_utility)

add_compile_options(-std=c++11)

include_directories(
  /usr/include/eigen3
)

find_library(RUT Utilities HINTS /usr/local/lib/RobotUtilities)

##
## Build
##

add_executable(ur_client client.cpp)