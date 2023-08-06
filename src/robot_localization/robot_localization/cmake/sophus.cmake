find_package (Sophus REQUIRED)

include_directories(${Sophus_INCLUDE_DIRS})
list(APPEND THIRD_PART_LIBRARIES ${Sophus_LIBRARIES} fmt)
list(APPEND THIRD_PART_LIBRARIES Sophus::Sophus)
list(APPEND ROS2_DEPENDENCIES ${Sophus_LIBRARIES} fmt)

