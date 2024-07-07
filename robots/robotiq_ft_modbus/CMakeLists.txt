include_directories(
  include
  driver/include
)

add_library(RQ_THREAD SHARED driver/src/rq_thread.c driver/src/mutex.c)

add_library(RQ_SENSOR_SOCKET SHARED driver/src/rq_sensor_socket.c)
target_link_libraries(RQ_SENSOR_SOCKET RQ_THREAD)

add_library(RQSensorLinux SHARED
  driver/src/rq_sensor_com.c
  driver/src/rq_sensor_state.c
)

add_executable(test_rq driver/src/main.c)
target_link_libraries(test_rq
  RQ_THREAD
  RQ_SENSOR_SOCKET
  RQSensorLinux
)

add_library(ROBOTIQ_FT_HW SHARED src/robotiq_ft_modbus.cpp)
target_link_libraries(ROBOTIQ_FT_HW
  ${RUT}
  RQ_THREAD
  RQ_SENSOR_SOCKET
  RQSensorLinux
)



install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
)
install(TARGETS ROBOTIQ_FT_HW
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
)