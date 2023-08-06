find_package (Protobuf)

include_directories(${Protobuf_INCLUDE_DIRS})
list(APPEND THIRD_PART_LIBRARIES ${Protobuf_LIBRARIES})