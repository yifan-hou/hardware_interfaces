include_directories(
  include/
  /usr/include/eigen3
)


add_library(UR_SOCKET SHARED src/ur_socket.cpp)
target_link_libraries(UR_SOCKET
    ${catkin_LIBRARIES} ${RUT}
)

add_executable(UR_TEST src/test.cpp)
target_link_libraries(UR_TEST UR_SOCKET)
