#!/bin/bash

path=$(dirname $(realpath $0))
cd $path/..
echo "当前路径为：$PWD"
source /opt/ros/humble/setup.bash
source ./install/setup.bash
ulimit -c unlimited
trap "ps aux|grep roborts_navigation|grep -v grep|awk '{ print $2 }' | xargs kill -s SIGINT ;echo 'Exit By Ctrl+C';exit;" SIGINT
trap "exit;" SIGHUP
while true;do
    if pgrep -f roborts_navigation > /dev/null;then
        echo "roborts_navigation is running"
    else
        echo "roborts_center_usart is not running"

        export time=$(date +%Y-%m-%d-%H-%M-%S)
        if [ ! -d "$path/../runtime_log" ];then
            mkdir $path/../runtime_log
        fi
        if [ ! -d "$path/../runtime_log/roborts_navigation" ];then
            mkdir $path/../runtime_log/roborts_navigation
        fi
        touch $path/../runtime_log/roborts_navigation/$time.log

        ros2 run roborts_navigation roborts_navigation 2>&1 | tee $path/../runtime_log/roborts_navigation/$time.log

        echo "Program Crash, On transport log file"
        if [ ! -d "$path/../error_log" ];then
            mkdir $path/../error_log
        fi
        if [ ! -d "$path/../error_log/roborts_navigation" ];then
            mkdir $path/../error_log/roborts_navigation
        fi
        cp $path/../runtime_log/roborts_navigation/$time.log $path/../error_log/roborts_navigation/$time.log
        echo "Program Crash, On transport core file"
        if [ ! -d "$path/../core_dump" ];then
            mkdir $path/../core_dump
        fi
        if [ ! -d "$path/../core_dump/roborts_navigation" ];then
            mkdir $path/../core_dump/roborts_navigation
        fi
        core_file=$(find /var/lib/apport/coredump -name "core*" -type f | grep roborts_navigation | tail -n 1)
        if [ -f "$core_file" ];then
            cp $core_file $path/../core_dump/roborts_navigation/$time.core
        fi
    fi
    sleep 1
done