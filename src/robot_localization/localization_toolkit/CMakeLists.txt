cmake_minimum_required(VERSION 3.8)
project(localization_toolkit)


# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# 设置 C++ 标准为 C++14，并要求项目必须使用该标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON) # 要求项目必须使用设置的 C++ 标准，如果编译器不支持该标准，CMake 会报错

# 禁用编译器特定的 C++ 语言扩展
set(CMAKE_CXX_EXTENSIONS OFF)

# 检查编译器类型，为 GNU 或 Clang 编译器添加额外的警告选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 为编译器添加编译选项，-pthread（启用多线程支持），-fexceptions（启用异常支持），-O3（启用最高级别的编译器优化）
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -fexceptions")

# 添加预处理器宏定义 ROOT_DIR
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

#为 C 编译器添加编译选项 -fexceptions，表示启用异常支持
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" ) 

add_definitions(-w)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# #第三方库配置
set(THIRD_PART_LIBRARIES "")
include(cmake/glog.cmake) 
include(cmake/pcl.cmake) 
include(cmake/yamlcpp.cmake)
include(cmake/sophus.cmake)
include(cmake/g2o.cmake)
# include(cmake/ceres.cmake)
find_package(Eigen3 QUIET)
find_package(VTK 6.3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Python3 COMPONENTS Interpreter)
if(NOT Python3_FOUND)
    message(FATAL_ERROR "Python3 not found, required for YAML parsing.")
endif()
find_package(OpenMP)
# 调用Python脚本解析YAML文件##############################
execute_process(
    COMMAND 
    ${Python3_EXECUTABLE} 
    ${CMAKE_CURRENT_SOURCE_DIR}/../compilation_config/parse_yaml.py 
    ${CMAKE_CURRENT_SOURCE_DIR}/../compilation_config/compilation.yaml
    OUTPUT_VARIABLE yaml_output
)
# 将解析后的变量设置为CMake变量
string(REPLACE "\n" ";" yaml_output_list "${yaml_output}")
foreach(line IN LISTS yaml_output_list)
    if(NOT "${line}" STREQUAL "")
        string(REPLACE "=" ";" pair ${line})
        list(GET pair 0 key)
        list(GET pair 1 value)
        set(${key} ${value})
    endif()
endforeach()

# 使用YAML文件中的变量
message("use_CUDA from YAML: ${use_CUDA}")
#################解析YAML文件##############################

# 使用 use_CUDA 变量作为条件判断
if(use_CUDA STREQUAL "False")
    message("CUDA is not enabled on this divice, lsc says.")
elseif(use_CUDA STREQUAL "True")
    message("CUDA is enabled on this divice, lsc says.")
    # 添加其他处理 CUDA 的代码
endif()

if(use_CUDA STREQUAL "True")
  find_package(CUDA)
endif()
set(cuPCL_LIBS "")
if(CUDA_FOUND)
  message("Using Cuda Version: ${CUDA_VERSION} ")
  enable_language(CUDA)
  include_directories(${CUDA_INCLUDE_DIRS})
  
endif()

#自定义库设置
file(GLOB_RECURSE MODELS_SRCS "src/models/*.cpp")
list(APPEND       MODELS_SRCS "include/localization_models/ikd-Tree/ikd_Tree.cpp")
file(GLOB_RECURSE TOOLS_SRCS "src/tools/*.cpp" )
file(GLOB_RECURSE SENSOR_SRCS "src/sensor_data/*.cpp" )

# list(REMOVE_ITEM MODELS_SRCS ${TOOLS_SRCS} )


# Find CUDA,考虑主机上可以部分使用cuda（目前是cudandt无法配准，但都可正常运行）
# 部分设备如笔记本、NUC上不再进行cuda部分的使用
# 计算设备AGX上必须使用CUDA加速，设置变量选择如下,生成静态库，不须包含CUDA

# find_package(CUDA)
# if(CUDA_FOUND)
#     message("Using Cuda Version: ${CUDA_VERSION} ")
#     enable_language(CUDA)
#     include_directories(${CUDA_INCLUDE_DIRS})

# endif()
message("Found Eigen3: " ${EIGEN3_INCLUDE_DIR})

######### 生成静态库，供定位的外部包调用########
include_directories(
include/localization_models
)
ament_auto_add_library(localization_models STATIC
${MODELS_SRCS} 
)
# ament_target_dependencies(localization_models ${Sophus_LIBRARIES})
target_link_libraries(localization_models Sophus::Sophus)

include_directories(
include/sensor_data
)

ament_auto_add_library(sensor_data STATIC
${SENSOR_SRCS}
)

include_directories(
include/localization_tools
)
ament_auto_add_library(localization_tools STATIC
${TOOLS_SRCS} 
# ${PROJECT_SOURCE_DIR}/include/localization_toolkit/registration/ndt_cuda/libcudandt_x86_64.so
)

if(use_CUDA STREQUAL "True")

    set(ndt_gpu_srcs
        src/models/registration/ndt_gpu/MatrixDevice.cu
        src/models/registration/ndt_gpu/MatrixHost.cu
        src/models/registration/ndt_gpu/NormalDistributionsTransform.cu
        src/models/registration/ndt_gpu/Registration.cu
        src/models/registration/ndt_gpu/VoxelGrid.cu
        src/models/registration/ndt_gpu/SymmetricEigenSolver.cu
      )
    set(ndt_gpu_incs
        include/localization_models/registration/ndt_gpu/common.h
        include/localization_models/registration/ndt_gpu/debug.h
        include/localization_models/registration/ndt_gpu/Matrix.h
        include/localization_models/registration/ndt_gpu/MatrixDevice.h
        include/localization_models/registration/ndt_gpu/MatrixHost.h
        include/localization_models/registration/ndt_gpu/NormalDistributionsTransform.h
        include/localization_models/registration/ndt_gpu/Registration.h
        include/localization_models/registration/ndt_gpu/SymmetricEigenSolver.h
        include/localization_models/registration/ndt_gpu/VoxelGrid.h
      )
    cuda_add_library(ndt_gpu  SHARED
                    ${ndt_gpu_srcs}  ${ndt_gpu_incs})
    target_link_libraries(ndt_gpu localization_models )
endif()
######### 生成静态库，供外部包调用########

if(use_CUDA STREQUAL "True")
  # install ndt_gpu headers and library
  install(TARGETS ndt_gpu
      LIBRARY DESTINATION lib
  )
endif()

install(FILES ${ndt_gpu_incs}
    DESTINATION include/localization_models/registration/ndt_gpu
)
if(use_CUDA STREQUAL "True")
ament_export_libraries(ndt_gpu)
endif()
ament_export_include_directories(include)
ament_export_dependencies(rclcpp sensor_msgs)
ament_auto_package()




