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
find_library(YAMLLib yaml-cpp HINTS /usr/local/lib)
find_library(RUT Utilities HINTS ${CMAKE_INSTALL_PREFIX}/lib/RobotUtilities)
find_library(TIMER_LIB TimerLinux HINTS ${CMAKE_INSTALL_PREFIX}/lib/RobotUtilities)

include_directories(
  include/
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_INSTALL_PREFIX}/include/arx5-sdk/
)

# comment out / uncomment the library you need
add_subdirectory(robots/ati_netft)
add_subdirectory(robots/arx_can)
add_subdirectory(robots/realsense)
add_subdirectory(robots/gopro)
add_subdirectory(robots/ur_rtde)
add_subdirectory(robots/robotiq_ft_modbus)

add_library(HI_COMMON
  src/types.cpp
)

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
  robots/ati_netft/include/
  robots/arx_can/include/
  robots/realsense/include/
  robots/ur_rtde/include/
  robots/robotiq_ft_modbus/include/
)
target_link_libraries(test_hardware_interfaces
  ${Boost_LIBRARIES}
  ${RUT}
  ${TIMER_LIB}
  ATI_NETFT_HW
  NETFT_DRIVER
  ARX_CAN
  REALSENSE
  ROBOTIQ_FT_HW
  spdlog::spdlog
  ${CMAKE_INSTALL_PREFIX}/lib/arx5-sdk/libhardware.so 
  ${CMAKE_INSTALL_PREFIX}/lib/arx5-sdk/libsolver.so
)