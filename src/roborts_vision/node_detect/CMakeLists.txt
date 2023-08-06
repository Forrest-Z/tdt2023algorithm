cmake_minimum_required(VERSION 3.8)
project(node_detect)

find_package(ament_cmake_auto REQUIRED)
find_package(libhikcamera REQUIRED)

include_directories(${libhikcamera_INCLUDE_DIR})
ament_auto_find_build_dependencies()
find_package(CUDA)
include(cmake/modules/libnvinferConfig.cmake)
if(CUDA_FOUND AND nvinfer_FOUND)
    message("Using Cuda Version: ${CUDA_VERSION} and TensorRT Version: ${nvinfer_VERSION}")
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
    target_link_libraries(${PROJECT_NAME} tensorrt ${nvinfer_LIBRARY} cudart nvonnxparser fmt ${PCL_LIBRARIES})
else()
find_package(OpenVINO REQUIRED)
    message("Using Openvino Version: ${OpenVINO_VERSION}")
    add_definitions(-D use_openvino)
    add_subdirectory(openvino)
    include_directories(${PROJECT_SOURCE_DIR}/openvino)
    ament_auto_add_executable(${PROJECT_NAME}
    DIRECTORY 
    src
    )
    target_link_libraries(${PROJECT_NAME}  
      openvino::runtime
      fmt
      openvino
    )
endif()

ament_auto_package()
