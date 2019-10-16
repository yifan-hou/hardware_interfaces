find_package(catkin REQUIRED COMPONENTS
  robomaster_hardware
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include/
)

add_library(DELTA_DJI SHARED src/delta_dji.cpp)
target_link_libraries(DELTA_DJI
	DELTA_INTERFACES
	robomaster_hardware
	${Boost_LIBRARIES}
)

add_executable(test_DELTA_DJI src/test_delta_dji.cpp)
target_link_libraries(test_DELTA_DJI
    DELTA_DJI
)
