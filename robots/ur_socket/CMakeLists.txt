include_directories(
  include/
  /usr/include/eigen3
)

add_library(UR_SOCKET SHARED src/ur_socket.cpp)
target_link_libraries(UR_SOCKET
    ${catkin_LIBRARIES} ${RUT}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
)
install(TARGETS UR_SOCKET
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
)