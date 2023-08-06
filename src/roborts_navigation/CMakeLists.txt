cmake_minimum_required(VERSION 3.8)
project(roborts_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-w)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(OpenMP)

include_directories(
  include/${PROJECT_NAME}
  include/${PROJECT_NAME}/frontend
  include/${PROJECT_NAME}/backend
)

ament_auto_add_executable(${PROJECT_NAME} DIRECTORY src src/frontend src/backend)

target_link_libraries(${PROJECT_NAME}
  -lpthread
  osqp::osqp
  OsqpEigen::OsqpEigen
  OpenMP::OpenMP_CXX
)

find_package(Boost REQUIRED COMPONENTS filesystem thread)
ament_auto_add_executable(utils_calc_map_con utils/utils_calc_map_con.cpp)
ament_auto_add_executable(utils_calc_map_towords_dis utils/utils_calc_map_towords_dis.cpp)
target_link_libraries(utils_calc_map_con ${Boost_LIBRARIES} -lpthread OpenMP::OpenMP_CXX)
target_link_libraries(utils_calc_map_towords_dis ${Boost_LIBRARIES} -lpthread OpenMP::OpenMP_CXX)

# 添加Python脚本目录
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  utils/utils_draw_show_map
  utils/utils_calc_map_min_dis
  utils/utils_draw_slope_map
  utils/utils_draw_command_map
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY show_map cost_map map_info con_map command_map
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

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

ament_auto_package()

