find_package(OpenCV REQUIRED)
find_package(depthai CONFIG REQUIRED)

include_directories(
  include/
)

add_library(OAK SHARED src/oak.cpp)
target_link_libraries(OAK ${OpenCV_LIBS} depthai::core depthai::opencv)

add_executable(TEST_OAK src/test.cpp)
target_link_libraries(TEST_OAK ${OpenCV_LIBS} ${RUT} ${TIMER_LIB} depthai::core depthai::opencv)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
)
install(TARGETS OAK
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
)
