find_package(Boost REQUIRED COMPONENTS
  system
  thread
  program_options
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

add_library(ATI_NETFT SHARED src/ati_netft.cpp src/netft_rdt_driver.cpp)
target_link_libraries(ATI_NETFT ${Boost_LIBRARIES} ${RUT} ${YAMLLib})

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
)
install(TARGETS ATI_NETFT
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
)