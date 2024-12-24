include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

add_library(WSG_GRIPPER SHARED
  src/wsg_gripper.cpp
  src/wsg_gripper_driver.cpp
)
target_link_libraries(WSG_GRIPPER
  pthread
  rt
  ${Boost_LIBRARIES}
)


add_executable(wsg_test
    src/test.cpp
)
target_link_libraries(wsg_test WSG_GRIPPER ${RUT})


install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
)
install(TARGETS WSG_GRIPPER
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
)