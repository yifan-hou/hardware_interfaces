include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

add_library(NETFT_DRIVER SHARED src/netft_rdt_driver.cpp)
target_link_libraries(NETFT_DRIVER ${Boost_LIBRARIES})

add_library(ATI_NETFT_HW SHARED src/ati_netft.cpp)
target_link_libraries(ATI_NETFT_HW NETFT_DRIVER ${Boost_LIBRARIES} ${RUT})

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
)
install(TARGETS NETFT_DRIVER ATI_NETFT_HW
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
)