cmake_minimum_required(VERSION 3.8)
project(node_debug)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(${PROJECT_NAME}
 DIRECTORY 
 src
)

ament_auto_package()
