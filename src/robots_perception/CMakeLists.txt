cmake_minimum_required(VERSION 3.8)
project(robots_perception)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()


find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
include_directories(include/${PROJECT_NAME})
find_package(libhikcamera REQUIRED)
include_directories(${libhikcamera_INCLUDE_DIR})

# Find CUDA
find_package(CUDA)
include(cmake/modules/libnvinferConfig.cmake)

if(CUDA_FOUND AND nvinfer_FOUND)
    message("Using Cuda Version: ${CUDA_VERSION} and NvInfer Version: ${nvinfer_VERSION}")
    add_definitions(-D use_tensorrt)
    enable_language(CUDA)
    include_directories(${CUDA_INCLUDE_DIRS})
    include_directories(${nvinfer_INCLUDE_DIRS})
    add_subdirectory(tensorrt)
    include_directories(${PROJECT_SOURCE_DIR}/tensorrt)
    ament_auto_add_executable(${PROJECT_NAME}
        DIRECTORY 
        src
    )
    # target_link_libraries(${PROJECT_NAME} tensorrt cudart nvonnxparser fmt ${PCL_LIBRARIES} ${nvinfer_LIBRARY})
    target_link_libraries(${PROJECT_NAME} tensorrt cudart nvonnxparser fmt ${nvinfer_LIBRARY})

else()
find_package(OpenVINO REQUIRED)
    message("Using Openvino Version: ${OpenVINO_VERSION}")
    # add_definitions(-D use_openvino)
    # add_subdirectory(openvino)
    # include_directories(${PROJECT_SOURCE_DIR}/openvino)
    # ament_auto_add_executable(${PROJECT_NAME}
    #  DIRECTORY 
    #  src
    # )
    # target_link_libraries(${PROJECT_NAME}  
    #   openvino::runtime
    #   fmt
    #   openvino
    # )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
  
ament_auto_package()
