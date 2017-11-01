cmake_minimum_required(VERSION 2.8.3)
#######################
project(egm)
set (CMAKE_CXX_FLAGS "--std=gnu++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  )

find_package(Boost REQUIRED COMPONENTS system)


include_directories(
  # ${catkin_INCLUDE_DIRS}
  ${PROJECT_BINARY_DIR}/ ## point to build directory where egm.pb.h will be generated
  include/
  /usr/include/eigen3
)
MESSAGE(STATUS "PROJECT_BINARY_DIR: " ${PROJECT_BINARY_DIR})

####################
## Protobuf Stuff ##
####################
include(FindProtobuf)
find_package(Protobuf REQUIRED)
set_source_files_properties(${PROTO_SRC} ${PROTO_HEADER} PROPERTIES GENERATED TRUE)
set(PROTOBUF_INCLUDE_DIRS ${PROTOBUF_INCLUDE_DIRS} PARENT_SCOPE)
set(PROTOBUF_LIBRARIES ${PROTOBUF_LIBRARIES} PARENT_SCOPE)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS proto/egm.proto)
MESSAGE(STATUS "Protobuf Src: " ${PROTO_SRCS})
MESSAGE(STATUS "Protobuf Hdr: " ${PROTO_HDRS})
MESSAGE(STATUS "Protobuf include_directories : " ${PROTOBUF_INCLUDE_DIRS})
add_library(egm_proto ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(egm_proto ${PROTOBUF_LIBRARIES})

#######################
## Compile Libraries ##
#######################

add_library(practical_socket src/practical_socket/PracticalSocket.cpp)
add_library(${PROJECT_NAME} src/EGMClass.cpp)
target_link_libraries(${PROJECT_NAME}
	egm_proto
	practical_socket
  # ${catkin_LIBRARIES}
)


#######################
## Install Libraries ##
#######################

# # 
# #   Installation for ROS. Comment out if not using ROS.
# # 
# catkin_package(
#   INCLUDE_DIRS include ${PROJECT_BINARY_DIR}/
#   LIBRARIES ${PROJECT_NAME}
#   CATKIN_DEPENDS roscpp
#   )

# install(DIRECTORY include/${PROJECT_NAME}/ ${PROJECT_BINARY_DIR}/
#     DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#     FILES_MATCHING PATTERN "*.h"
#     )


# 
#   Uncomment this part if used as an independent library.
#   Installation to /usr/local. 
# 
# headers
install(DIRECTORY include/
    DESTINATION /usr/local/include/${PROJECT_NAME}
    )

# binaries
install(TARGETS ${PROJECT_NAME}
    DESTINATION /usr/local/lib/${PROJECT_NAME}
    )
