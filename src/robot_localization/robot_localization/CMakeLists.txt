cmake_minimum_required(VERSION 3.8)
project(robot_localization)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_definitions(-w)
find_package(OpenMP)
# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()



# 解释 .h.in文件里面制定的路径，解释到.h里
configure_file(
                ${PROJECT_SOURCE_DIR}/include/robot_localization/global_path_defination/global_path.h.in 
                ${PROJECT_SOURCE_DIR}/include/robot_localization/global_path_defination/global_path.h
              )
              
# #第三方库配置
set(THIRD_PART_LIBRARIES "")
include(cmake/glog.cmake) 
include(cmake/pcl.cmake) 
include(cmake/yamlcpp.cmake)
include(cmake/sophus.cmake)
include(cmake/g2o.cmake)
# include(cmake/ceres.cmake)
include(cmake/protobuf.cmake)
include(cmake/gtsam.cmake)

# set(EIGEN3_INCLUDE_DIR "/usr/local/eigen-3.3.7/share/eigen3")
find_package(Eigen3 3.3.7)
find_package(VTK 6.3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
message(Eigen: ${EIGEN3_INCLUDE_DIR})

# set(PCL_DIR "/usr/local/pcl-1.10.1/share/pcl-1.10")
# find_package( 1.10 )
# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINATIONS})



#自定义库设置
file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/apps/*_node.cpp" )
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS} )


message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

# message("Python Include = ${PYTHON_INCLUDE_DIRS}")
# # -DPYTHON_INCLUDE_DIR=/usr/include/python2.7 -DPYTHON_LIBRARY=/usr/lib/python2.7/config/libpython2.7.so ..
# if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" )
#   message("aarch64 to set pythonlib path")
#   # set(PYTHON_INCLUDE_DIRS "/usr/include/python3.10") 
#   # set(PYTHON_LIBRARY "/usr/lib/python3.10/config-3.10-aarch64-linux-gnu/libpython3.10.so")
#   include(/usr/share/cmake-3.22/Modules/FindPythonLibs.cmake)

# endif()
# message("Python Include = ${PYTHON_INCLUDE_DIRS}")
# find_package(PythonLibs REQUIRED)

# Find CUDA,考虑主机上可以部分使用cuda（目前是cudandt无法配准，但都可正常运行）
# 部分设备如笔记本、NUC上不再进行cuda部分的使用
# 计算设备AGX上必须使用CUDA加速，设置变量选择如下,生成静态库，不须包含CUDA
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
#在一个localization_toolkit包内生成了3个静态库，如果不手动include_directories，无法识别其目录下的3个名字不同的静态库
include_directories(${localization_toolkit_INCLUDE_DIRS})
#在一个localization_toolkit包内生成了3个静态库，如果不手动include_directories，无法识别其目录下的3个名字不同的静态库

# 使用YAML文件中的变量
message("use_CUDA from YAML: ${use_CUDA}")
#################解析YAML文件##############################
if(use_CUDA STREQUAL "True")
  find_package(CUDA)
endif()
set(cuPCL_LIBS "")
if(CUDA_FOUND)
  message("Using Cuda Version: ${CUDA_VERSION} ")
  enable_language(CUDA)
  include_directories(${CUDA_INCLUDE_DIRS})
  add_definitions(-DBUILDING_WITH_CUDA) #自定义的用于区分具体代码编译内容的宏定义
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" )
    list(APPEND cuPCL_LIBS ${PROJECT_SOURCE_DIR}/include/robot_localization/models/libcudandt_main.so)
    list(APPEND cuPCL_LIBS ${PROJECT_SOURCE_DIR}/include/robot_localization/models/libcudaicp_main.so)
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    list(APPEND cuPCL_LIBS ${PROJECT_SOURCE_DIR}/include/robot_localization/models/libcudandt_x86_64.so)
    list(APPEND cuPCL_LIBS ${PROJECT_SOURCE_DIR}/include/robot_localization/models/libcudaicp_x86_64.so)
    # list(APPEND cuPCL_LIBS ndt_gpu)


  endif()
  link_directories(
    ${CUDA_LIBRARY_DIRS}
    ${PCL_LIBRARY_DIRS}
    ${Boost_LIBRARY_DIRS}
    )
    list(APPEND THIRD_PART_LIBRARIES ${Boost_LIBRARIES})
    list(APPEND THIRD_PART_LIBRARIES ${VTK_LIBRARIES})
    list(APPEND THIRD_PART_LIBRARIES -lpthread)
  include_directories(
      include
      ${PCL_INCLUDE_DIRS}
      ${VTK_INCLUDE_DIRS}
      ${Boost_INCLUDE_DIRS}
    )
    
    
endif()

                      

list(APPEND THIRD_PART_LIBRARIES OpenMP::OpenMP_CXX)


include_directories(${EIGEN3_INCLUDE_DIR})
message(Eigen: ${EIGEN3_INCLUDE_DIR})
######### 生成静态库，供定位的外部包调用########
include_directories(
include/robot_localization
)
ament_auto_add_library(${PROJECT_NAME} STATIC
${ALL_SRCS} 
${cuPCL_LIBS}
)
######### 生成静态库，供定位的外部包调用########
# 如果使用CUDA，将CUDA库添加到链接库列表
if(use_CUDA STREQUAL "True")
    list(APPEND THIRD_PART_LIBRARIES ${CUDA_LIBRARIES})
endif()

# 数据预处理端节点
ament_auto_add_executable(data_pretreat_end_node
  src/apps/data_pretreat_end_node.cpp  )
target_link_libraries(data_pretreat_end_node 
  ${THIRD_PART_LIBRARIES}  
  ${cuPCL_LIBS}
)


# 点云图匹配节点
ament_auto_add_executable(matching_end_node
  src/apps/matching_end_node.cpp  )
target_link_libraries(matching_end_node 
  ${THIRD_PART_LIBRARIES}
  ${cuPCL_LIBS}
)
# cuda_add_executable(matching_end_node
#   src/apps/matching_end_node.cpp
#   ${ALL_SRCS} 
  
# )
# target_link_libraries(matching_end_node 
#   ${THIRD_PART_LIBRARIES}
#   ${cuPCL_LIBS}
# )
# ament_target_dependencies(
#   matching_end_node
#   geometry_msgs
#   rclcpp
#   nav_msgs
#   pcl_msgs
#   sensor_msgs
#   std_msgs
#   tf2
#   tf2_geometry_msgs
#   tf2_ros
#   tf2_sensor_msgs
#   localization_toolkit
# )



# 基于图优化之滑动窗口模型的后端优化节点
ament_auto_add_executable(alaserPGO
  src/optimizing_end/optimizing_end_node.cpp  )
target_link_libraries(alaserPGO 
  ${THIRD_PART_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
)

# 读取pcd文件，rviz广播节点
ament_auto_add_executable(pcd_process_end
  src/apps/pcd_process_node.cpp )
target_link_libraries(pcd_process_end 
  ${THIRD_PART_LIBRARIES}
  ${cuPCL_LIBS}
)
# Add CUDA support
set_target_properties(${PROJECT_NAME} PROPERTIES CUDA_SEPARABLE_COMPILATION ON)



install(TARGETS data_pretreat_end_node
  DESTINATION lib/${PROJECT_NAME})
# install(TARGETS lidar_odom_end_node
#   DESTINATION lib/${PROJECT_NAME})
install(TARGETS matching_end_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS alaserPGO
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS pcd_process_end
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_auto_package()

