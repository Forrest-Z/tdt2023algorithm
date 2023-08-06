#!/bin/bash
pushd `pwd` > /dev/null
cd `dirname $0`
echo "Working Path: "`pwd`
# 执行当前目录下的脚本
# cd src/robot_localization/livox_ros_driver2
# ./build.sh humble

# cd ../../../
# 执行colcon build命令
pushd `pwd` > /dev/null
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-ignore   findx_pkg robot_localization

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select livox_ros_driver2 localization_interfaces front_end robot_localization localization_toolkit
# localization_toolkit
# livox_ros_driver2
# front_end
# fast_lio_slam
popd > /dev/null


