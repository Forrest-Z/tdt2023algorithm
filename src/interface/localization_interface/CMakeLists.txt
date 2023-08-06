cmake_minimum_required(VERSION 3.8)
project(localization_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
# find_package(ament_cmake REQUIRED)
# # find_package(rclcpp REQUIRED)
# # find_package(std_msgs REQUIRED)
# find_package(geometry_msgs)
# find_package(sensor_msgs )
# find_package(rosidl_default_generators REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(base_interface REQUIRED)




file(GLOB_RECURSE 
     MSG_FILES 
     RELATIVE 
     
     ${CMAKE_CURRENT_SOURCE_DIR}
     msg/*.msg
)
file(GLOB_RECURSE 
     SRV_FILES 
     RELATIVE 
     ${CMAKE_CURRENT_SOURCE_DIR}
     srv/*.srv
)

file(GLOB_RECURSE 
     ACTION_FILES 
     RELATIVE 
     ${CMAKE_CURRENT_SOURCE_DIR}
     action/*.action
)

rosidl_generate_interfaces(${PROJECT_NAME}

  ${MSG_FILES}
  ${SRV_FILES}
   DEPENDENCIES 
   geometry_msgs
   std_msgs
   sensor_msgs

   LIBRARY_NAME ${PROJECT_NAME}
 )


ament_package()
