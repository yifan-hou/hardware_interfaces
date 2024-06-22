find_package(realsense2 REQUIRED )
find_package(OpenCV REQUIRED)

include_directories(
  include/
  ${realsense2_INCLUDE_DIR}
)

add_library(REALSENSE SHARED src/realsense.cpp)
target_include_directories(REALSENSE PUBLIC include/ /usr/include/librealsense2/)
  target_link_libraries(REALSENSE ${RUT} ${realsense2_LIBRARY} ${OpenCV_LIBS}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
)
install(TARGETS REALSENSE
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
)