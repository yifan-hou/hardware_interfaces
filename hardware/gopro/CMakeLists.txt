find_package(OpenCV REQUIRED)

include_directories(
  include/
)

add_executable(USBRESET src/usbreset.cpp)

add_executable(TEST src/test.cpp)
target_link_libraries(TEST ${OpenCV_LIBS}
)

add_library(GoPro SHARED src/gopro.cpp)
target_link_libraries(GoPro ${OpenCV_LIBS})

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
)
install(TARGETS GoPro
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
)