cmake_minimum_required(VERSION 3.5)
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

find_package(Eigen3 REQUIRED)
find_package(pybind11 REQUIRED)
find_library(YAMLLib yaml-cpp HINTS /usr/local/lib)
find_library(RUT Utilities HINTS ${CMAKE_INSTALL_PREFIX}/lib/RobotUtilities)

include_directories(
  include/
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_INSTALL_PREFIX}/include/arx5-sdk/
)
add_library(HI_COMMON SHARED
  src/types.cpp
)

# hardware to build
add_subdirectory(hardware/ati_netft)
# add_subdirectory(hardware/arx_can)
add_subdirectory(hardware/realsense)
add_subdirectory(hardware/gopro)
add_subdirectory(hardware/ur_rtde)
add_subdirectory(hardware/robotiq_ft_modbus)

# workcell to build
# include every "include" directory under /hardware/*
file(GLOB HARDWARE_INCLUDE_DIRS "hardware/*/include")
include_directories(${HARDWARE_INCLUDE_DIRS})
add_subdirectory(workcell/table_top_manip)

# install common
install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
)
install(TARGETS HI_COMMON
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
)

# Test executable
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
add_executable(test_hardware_interfaces
  test/main.cc
)
target_include_directories(test_hardware_interfaces PRIVATE
  hardware/ati_netft/include/
  hardware/arx_can/include/
  hardware/realsense/include/
  hardware/ur_rtde/include/
  hardware/robotiq_ft_modbus/include/
)
target_link_libraries(test_hardware_interfaces
  ${Boost_LIBRARIES}
  ${RUT}
  ATI_NETFT
  REALSENSE
  ROBOTIQ_FT_HW
  spdlog::spdlog
  # ARX_CAN
  # ${CMAKE_INSTALL_PREFIX}/lib/arx5-sdk/libhardware.so 
  # ${CMAKE_INSTALL_PREFIX}/lib/arx5-sdk/libsolver.so
)