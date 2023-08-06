cmake_minimum_required(VERSION 3.8)
project(front_end)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置 C++ 标准为 C++14，并要求项目必须使用该标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON) # 要求项目必须使用设置的 C++ 标准，如果编译器不支持该标准，CMake 会报错

# 禁用编译器特定的 C++ 语言扩展
set(CMAKE_CXX_EXTENSIONS OFF)

# 为编译器添加编译选项，-pthread（启用多线程支持），-fexceptions（启用异常支持），-O3（启用最高级别的编译器优化）
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -fexceptions")

# 添加预处理器宏定义 ROOT_DIR
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

#为 C 编译器添加编译选项 -fexceptions，表示启用异常支持
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" ) 

add_definitions(-w)

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 5)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=4)
    message("core for MP:  3")
  elseif(N GREATER 3)
    math(EXPR PROC_NUM "${N} - 2")
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM="${PROC_NUM}")
    message("core for MP:  ${PROC_NUM}")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")


message("Python Include = ${PYTHON_INCLUDE_DIRS}")
# -DPYTHON_INCLUDE_DIR=/usr/include/python2.7 -DPYTHON_LIBRARY=/usr/lib/python2.7/config/libpython2.7.so ..
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" )   #专为agx写，电脑上只需要一行find_package(PythonLibs REQUIRED)
  message("aarch64 to set pythonlib path")
  # set(PYTHON_INCLUDE_DIRS "/usr/include/python3.10") 
  # set(PYTHON_LIBRARY "/usr/lib/python3.10/config-3.10-aarch64-linux-gnu/libpython3.10.so")
  include(/usr/share/cmake-3.22/Modules/FindPythonLibs.cmake)

endif()

message("Python Include = ${PYTHON_INCLUDE_DIRS}")
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")
find_package(PythonLibs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL  REQUIRED)
# find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")
find_package (yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIRS})


message(Eigen: ${EIGEN3_INCLUDE_DIR})

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  include)
  

ament_auto_add_executable(frontend_mapping 
                          src/laserMapping.cpp 
                          # include/ikd-Tree/ikd_Tree.cpp 
                          src/parameters.cpp 
                          src/preprocess.cpp 
                          src/Estimator.cpp
                         )
target_link_libraries(frontend_mapping 
    ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} ${YAML_CPP_LIBRARIES}
)
target_include_directories(frontend_mapping PRIVATE ${PYTHON_INCLUDE_DIRS} )




install(TARGETS frontend_mapping
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_auto_package()
