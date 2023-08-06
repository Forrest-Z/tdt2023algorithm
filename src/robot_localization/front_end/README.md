### 定位接口
1、定位给到导航的信息：
```
Type:       localization_interface::msg::LAM2Nav
Topic Name: /tdt_LAM_001

```

2、低频的，只供调试时于rviz显示的定位前端位姿：
```
Type:       geometry_msgs::msg::PoseStamped
Topic Name: /tdt_localization_cheng

```
3、定位启动服务：
```
Type:         std_srvs/srv/SetBool
Service Name: /tdt_localization_start

```
4、定位呼救视觉云台控制
```
Type:         std_srvs/srv/SetBool
Service Name: /tdt_LAM_MAYDAY       

std_srvs/srv/SetBool类型描述如下
bool data # e.g. for hardware enabling / disabling
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages

如果定位发出的data为true，表明LAM lost，需要云台配合重定位，如果为false，云台控制不必理会
返回的信息：success若为true，则表明视觉已控制云台到指定角度附近
message赋空值
```
### 启动定位
```bash
ros2 launch robot_localization matching_LAM_launch.py
# <!-- 默认由定位启动服务启动定位，但也提供手动启动命令： -->
ros2 service call /localization_start std_srvs/srv/SetBool data:\ true
```

#### gtsam优化库的安装
虽然可以直接使用 ``sudo apt install ros-humble-gtsam`` 但是实践表面，这与 ``libeigen3-dev``并不兼容:
```
error: static assertion failed: Error: GTSAM was built against a different version of Eigen
```

原因是GTSAM自带的eigen和系统安装的eigen之间有冲突。GTSAM编译的时候默认使用了自带的eigen，而系统中如果还手动安装过一个eigen的话，就会出现两个eigen的冲突
```
sudo apt install intel-mkl
git clone -b develop https://github.com/borglab/gtsam.git
必须选择develop分支
修改 gtsam/cmake 目录下的 HandleEigen.cmake 文件中
option(GTSAM_USE_SYSTEM_EIGEN "Find and use system-installed Eigen. If 'off', use the one bundled with GTSAM" OFF)
修改为option(GTSAM_USE_SYSTEM_EIGEN "Find and use system-installed Eigen. If 'off', use the one bundled with GTSAM" ON)
```

<!-- cmake -DGTSAM_BUILD_WITH_MARCH_NAITVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON .. -->
这样就相当于GTSAM编译也使用了系统eigen。

/opt/ros/humble/lib/python3.10/site-packages/launch/actions
目录下sudo gedit execute_local.py ，time_out 修改信号捕获延迟11s