cmake_minimum_required(VERSION 3.5)
#######################
project(table_top_manip)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(realsense2 REQUIRED )
find_package(Boost REQUIRED COMPONENTS
  system
  thread
  program_options
)
find_library(YAMLLib yaml-cpp HINTS /usr/local/lib)

find_library(RUT Utilities HINTS ${CMAKE_INSTALL_PREFIX}/lib/RobotUtilities)
find_library(FORCE_CONTROLLERS FORCE_CONTROLLERS HINTS ${CMAKE_INSTALL_PREFIX}/lib/)

include_directories(
  include/
  yaml-cpp
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(ManipServer SHARED src/manip_server.cc src/manip_server_loops.cc src/perturbation_generator.cc)
target_link_libraries(ManipServer
  # REALSENSE
  ATI_NETFT
  GoPro
  OAK
  ROBOTIQ_FT_HW
  RQSensorLinux
  UR_RTDE
  WSG_GRIPPER
  # COIN_FT
  HI_COMMON
  ${RUT}
  ${FORCE_CONTROLLERS}
  # ${realsense2_LIBRARY}
  ${OpenCV_LIBS}
  ${YAMLLib}
)

# c++ library install
install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
)
install(TARGETS ManipServer
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
)

pybind11_add_module(manip_server_pybind 
  src/manip_server_pybind.cc
)
target_link_libraries(manip_server_pybind PRIVATE
  ManipServer
)

# set the output directory for the pybind module
set_target_properties(manip_server_pybind PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/python")
set_target_properties(manip_server_pybind PROPERTIES PREFIX "")
set_target_properties(manip_server_pybind PROPERTIES OUTPUT_NAME "manip_server_pybind")