#! /bin/sh

roborts_center_usart=${1:-1}
node_detect=${2:-1}
node_resolve=${3:-1}
node_predict=${4:-1}


export CURRENT_PATH=".."
cd ${CURRENT_PATH}
echo "当前路径为：${CURRENT_PATH}"

	
    echo "Running in sh or bash"
    # 激活ros的环境
    . /opt/ros/humble/setup.sh
    # 激活我们自己编写的ros程序的环境
    . install/setup.sh



while true;do

#使用 & 将程序作为后台进程启动，这样脚本就可以继续执行而不会阻塞。

  if [ $roborts_center_usart -eq 1 ];
  then  
    if pgrep -f roborts_center_usart > /dev/null;then
        echo "roborts_center_usart is running"
    else
        echo "roborts_center_usart is not running"
        ros2 run roborts_center_usart roborts_center_usart &
        # sleep 1

    fi
  fi  


  if [ $node_detect -eq 1 ]; 
  then
    if pgrep -f node_detect > /dev/null;then
        echo "node_detect is running"
    else
        echo "node_detect is not running"
        ros2 run node_detect node_detect &
        # sleep 2

    fi
  fi

if [ $node_resolve -eq 1 ];     
then
    if pgrep -f node_resolve > /dev/null;then
        echo "node_resolve is running"
    else
        echo "node_resolve is not running"
        ros2 run node_resolve node_resolve &
        #sleep 1

    fi
    fi

if [ $node_predict -eq 1 ];     
    then
    if pgrep -f node_predict > /dev/null;then
        echo "node_predict is running"
    else
        echo "node_predict is not running"
        ros2 run node_predict node_predict &
        # sleep 1

    fi

  fi  
    echo "\n"

    sleep 1

done

