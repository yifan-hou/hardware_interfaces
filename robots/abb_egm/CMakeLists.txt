include_directories(
  include/
  /usr/include/eigen3
)
MESSAGE(STATUS "PROJECT_BINARY_DIR: " ${PROJECT_BINARY_DIR})

# ####################
# ## Protobuf Stuff ##
# ####################
include(FindProtobuf)
find_package(Protobuf REQUIRED)
set_source_files_properties(${PROTO_SRC} ${PROTO_HEADER} PROPERTIES GENERATED TRUE)
set(PROTOBUF_INCLUDE_DIRS ${PROTOBUF_INCLUDE_DIRS} PARENT_SCOPE)
set(PROTOBUF_LIBRARIES ${PROTOBUF_LIBRARIES} PARENT_SCOPE)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS proto/egm.proto)
MESSAGE(STATUS "Protobuf Src: " ${PROTO_SRCS})
MESSAGE(STATUS "Protobuf Hdr: " ${PROTO_HDRS})
MESSAGE(STATUS "Protobuf include_directories : " ${PROTOBUF_INCLUDE_DIRS})
add_library(egm_proto SHARED ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(egm_proto ${PROTOBUF_LIBRARIES})

include_directories(
  ${PROJECT_BINARY_DIR}/ ## egm.pb.h will be generated at ${PROJECT_BINARY_DIR}/robots/abb_egm/
)

#######################
## Compile Libraries ##
#######################

add_library(practical_socket SHARED src/practical_socket/PracticalSocket.cpp)
add_library(ABB_EGM SHARED src/abb_egm.cpp)
target_link_libraries(ABB_EGM
    egm_proto
    practical_socket
)