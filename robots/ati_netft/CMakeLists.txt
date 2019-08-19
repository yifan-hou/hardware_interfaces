include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

find_library(RUT Utilities HINTS /usr/local/lib/RobotUtilities)

add_library(NETFT_DRIVER src/netft_rdt_driver.cpp)
target_link_libraries(NETFT_DRIVER ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_library(ATI_NETFT_HW src/ati_netft.cpp)
target_link_libraries(ATI_NETFT_HW NETFT_DRIVER ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${RUT})