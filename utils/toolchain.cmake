#File toolchain.cmake for ROS and system packages to cross compile.
SET(CMAKE_SYSTEM_NAME Linux)

SET(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc)
SET(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++)

set(CMAKE_C_COMPILER_WORKS   1)
set(CMAKE_CXX_COMPILER_WORKS 1) 

# Below call is necessary to avoid non-RT problem.
set(CMAKE_LIBRARY_ARCHITECTURE aarch64)
set(CMAKE_SYSTEM_PROCESSOR aarch64) 
set(CMAKE_CROSSCOMPILING TRUE)


SET(CROSSCOMPILE_ROOT_PATH /cross_compiler)
SET(CROSSCOMPILE_ROS_PATH ${CROSSCOMPILE_ROOT_PATH}/opt/ros/humble)

SET(CMAKE_FIND_ROOT_PATH ${CROSSCOMPILE_ROOT_PATH} ${CATKIN_DEVEL_PREFIX} /tdt2023algorithm/install)

#If you have installed cross compiler to somewhere else, please specify that path.
SET(COMPILER_ROOT /usr/aarch64-linux-gnu) 

#Have to set this one to BOTH, to allow CMake to find rospack
#This set of variables controls whether the CMAKE_FIND_ROOT_PATH and CMAKE_SYSROOT are used for find_xxx() operations.
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BOTH)
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

SET(CMAKE_PREFIX_PATH ${CROSSCOMPILE_ROS_PATH} ${CROSSCOMPILE_ROOT_PATH}/usr ${CROSSCOMPILE_ROOT_PATH}/usr/lib/aarch64-linux-gnu/ ${CROSSCOMPILE_ROOT_PATH}/usr/lib/aarch64-linux-gnu/cmake/ ${CMAKE_PREFIX_PATH} )

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --sysroot=${CROSSCOMPILE_ROOT_PATH}")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --sysroot=${CROSSCOMPILE_ROOT_PATH}")
SET(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} --sysroot=${CROSSCOMPILE_ROOT_PATH}")
SET(CMAKE_CXX_LINK_FLAGS "${CMAKE_CXX_LINK_FLAGS} --sysroot=${CROSSCOMPILE_ROOT_PATH}")

link_directories(${CROSSCOMPILE_ROS_PATH}/lib ${CROSSCOMPILE_ROOT_PATH}/usr/lib ${CROSSCOMPILE_ROOT_PATH}/usr/lib/aarch64-linux-gnu/)
# SET(LD_LIBRARY_PATH ${CROSSCOMPILE_ROS_PATH}/lib ${CROSSCOMPILE_ROOT_PATH}/usr/lib ${CROSSCOMPILE_ROOT_PATH}/usr/lib/aarch64-linux-gnu/ ${LD_LIBRARY_PATH})


set(MRT_ARCH aarch64)