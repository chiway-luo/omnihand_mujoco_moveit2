<!-- # 基于moveit的路径规划实现自动规划手指移动的功能 -->
# 基于 ROS2 + MoveIt2 搭建灵巧手手型规划与执行系统

官方文档 [Omnihand-O10](https://www.zhiyuan-robot.com/DOCS/OS/Omnihand-O10)

> 手型请求 -> 角度预设读取 -> MoveIt 规划 -> 执行结果反馈 -> 底层驱动下发
---

- 开发背景
> 在当前的大部分灵巧手sdk中并没有引入手指之间的碰撞检测和路径规划算法,导致在一些复杂的手型切换过程中,可能会出现手指之间的碰撞或者无法规划出合理的路径的问题,因此引入moveit的路径规划算法可以有效的解决这些问题,实现更加智能和安全的手指移动控制

- 实现原理
> 通过配置不同手型,发送请求到服务端,调用moveit的路径规划算法,实现自动规划手指移动的功能,并通过底层驱动接口控制灵巧手执行相应动作。

- 注意事项
> 该实现中,将moveit在rviz的本地插件(目标姿态)设置为透明,仅支持通过发送服务请求的方式进行路径规划,不支持在rviz中直接设置目标姿态进行路径规划(修改后可以取消此限制)。


---
class_doc中为公开课讲义文件，包含了每一章的内容介绍和代码实现细节，供同学们参考学习。
## 课程文件导览
[第一章 环境准备](class_doc/第一章.md)

[第二章 灵巧手介绍](class_doc/第二章.md)

[第三章 测试功能包编写](class_doc/第三章.md)
## 示例视频
<video src="https://github.com/user-attachments/assets/ac17ca8b-bb46-4fef-8a15-a6e34bdc93ea" controls="controls" width="600">
</video>

## 系统要求
- ubuntu 22.04
- ros2 humble
- moveit2 humble

## 配置依赖
安装moveit和ros_control相关的包
```bash
sudo apt update
sudo apt install cmake
sudo apt install ros-${ROS_DISTRO}-moveit ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers
sudo apt install python3.10-dev
sudo apt install ros-${ROS_DISTRO}-joint-state-publisher-gui 
sudo apt install ros-${ROS_DISTRO}-joint-state-publisher
pip3 install build setuptools wheel

```
## 安装灵巧手
> 我使用的为RS485串口驱动,如果使用can驱动,请参考SDK文档中关于can驱动的安装说明
```
sudo chmod 666 /dev/ttyACM0
```
## 功能包说明
- omnihand_node 灵巧手底层驱动功能包,提供底层通信接口

- omnihand_node_msgs 灵巧手底层驱动消息定义功能包,包含电机控制模式消息定义

- hand_description 描述文件

- hand_moveit moveit路径规划算法

- hand_control 桥接moveit和底层驱动的控制节点

- hand_test_bag 测试功能包,包含测试节点和launch文件(robot_state_publisher_gui控制灵巧手)

- hand_shape 手型库功能包,包含不同手型的描述文件和moveit配置文件
## 编译SDK
```bash
./build.sh -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=./build/install -DBUILD_PYTHON_BINDING=ON -DBUILD_CPP_EXAMPLES=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5
```
## 启动测试功能包
```bash
ros2 launch hand_test_bag hand_test.launch.py
```
## 启动moveit配置助手
```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```
## 启动moveit控制节点(实现节点)
```bash
ros2 launch hand_control hand_control.launch.py
```
## 规划不同手型
<!-- ```bash
ros2 param set /hand_shape current_shape default  # 默认手型
ros2 param set /hand_shape current_shape catch    # 抓握
ros2 param set /hand_shape current_shape joke     # 竖中指
``` -->
```bash
ros2 service call /hand_shape/set_shape omnihand_node_msgs/srv/SetHandShape "{shape_name: catch}"
```
- 详细预定义手型见 [hand_shape.yaml](src/hand_test/hand_shape/param/hand_shape.yaml)

> 我将退学在家,专心研究这个代码是怎么写的,感谢copilot && claude && gpt 的帮助,让我能在短时间内完成这个功能包的编写,并且在这个过程中学到了很多ROS2和moveit的知识,非常感谢他们的帮助和支持!
