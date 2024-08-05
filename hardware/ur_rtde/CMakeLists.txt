include_directories(
  include/
  /usr/include/eigen3
)

find_package(ur_rtde REQUIRED)

add_library(UR_RTDE SHARED src/ur_rtde.cpp)
target_link_libraries(UR_RTDE ${RUT} ${YAMLLib} ur_rtde::rtde
)

# test
add_executable(test_ur_rtde src/test.cpp)
target_link_libraries(test_ur_rtde UR_RTDE ${RUT} ${TIMER_LIB} ur_rtde::rtde
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
)
install(TARGETS UR_RTDE
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
)