include_directories(
  include/
)

include_directories(
  ${CMAKE_INSTALL_PREFIX}/include/arx5-sdk/
)

find_library(ARX ArxJointController HINTS ${CMAKE_INSTALL_PREFIX}/lib/arx5-sdk)

add_library(ARX_CAN SHARED src/arx_can.cpp)
target_link_libraries(ARX_CAN ${RUT} ${ARX} ${YAMLLib}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
)
install(TARGETS ARX_CAN
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
)