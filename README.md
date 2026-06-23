# 说明
本项目是为萃同电机科技有限公司搭建的移动双臂机器人的**机械臂**部分写的说明文档

操作系统是 ubuntu22.04 版本（ros2 humble）
# 机械臂硬件连接
两条机械臂各自分别占用jetson orin nano的一个usb接口

连接时需要确保can1接左臂，can2接右臂

![alt text](image.png)

# 部署驱动
### 安装依赖
前提：用户使用设备已有ros2 humble
```bash
sudo apt update
# 安装ros2_control
sudo apt install ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers ros-${ROS_DISTRO}-controller-manager
# 安装moveit
sudo apt install ros-humble-moveit 
sudo apt install ros-humble-moveit-resources-panda-moveit-config
```
### 安装功能包
```bash
# 创建工作空间
mkdir -p ros2_ws/src
cd ros2_ws/src
# 拉取功能包（三个）
git clone https://github.com/wddycn/ct_arms_pkg.git
cd ..
# 编译
colcon build
```
# 驱动方式

## 方式一：ros2_control
三个功能包编译完成后在终端输入

```bash
source ros2_ws/install/setup.bash
ros2 launch eyou_ros2_control eyou_control.launch.py 
```
启动成功后，正常情况下机械臂会回到初始位置，并使能（变硬）

<video controls src="de9f1a8d189bd0b130a977590630f0b6.mp4" title="Title"></video>

新建终端，输入以下命令进行机械臂角度控制（角度可更改，单位：rad）
```bash

```