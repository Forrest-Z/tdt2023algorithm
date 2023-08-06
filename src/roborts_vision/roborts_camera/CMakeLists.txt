cmake_minimum_required(VERSION 3.8)
project(roborts_camera)
add_definitions(-w)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include/${PROJECT_NAME})

find_package(libhikcamera REQUIRED)
include_directories(${libhikcamera_INCLUDE_DIR})

# ament_auto_add_executable(${PROJECT_NAME}
#  DIRECTORY src)

ament_auto_add_library(${PROJECT_NAME} SHARED
DIRECTORY src
)

target_link_libraries(${PROJECT_NAME}
  ${libhikcamera_LIBRARY}
  -lpthread -lm
)

# install(
#   TARGETS ${PROJECT_NAME}
#   EXPORT ${PROJECT_NAME}
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
#   # INCLUDES DESTINATION include
# )

ament_auto_package()

