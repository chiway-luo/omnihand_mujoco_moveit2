<!-- # 基于moveit的路径规划实现自动规划手指移动的功能 -->
# 基于 ROS2 + MoveIt2 + mujoco 的灵巧手仿真

## 项目介绍
> 本项目基于 ROS2 Humble + MoveIt2 Humble + Mujoco 2.3.0 实现了一个灵巧手的仿真系统。该系统能够通过 MoveIt2 进行路径规划，实现自动规划手指移动的功能。同时，项目还提供了一个 hand_shape 服务端节点，可以根据不同的手型需求进行转换。

![](.doc/image/image.jpg)

## 系统要求
- ROS2 Humble
- MoveIt2 Humble
- Mujoco 2.3.0
- ubuntu 22.04

## 配置流程
> 当前已经并入src中,可以删除`src/mujoco_ros2_control/`并重新clone

- 需要从github上克隆mujoco_ros2_control仓库,并放置在src目录下
```bash
cd src/
git clone https://github.com/dfki-ric/mujoco_ros2_control
```

- 编译
```bash
colcon build
```

## 启动指令
1. 启动mujoco仿真 + moveit控制 + hand_shape手型转换服务端节点
```bash
ros2 launch hand_mujoco_launch hand_mujoco.launch.py 
```

2. 使用指令呼叫hand_shape服务端,发出请求(更多手型见[hand_shape.yaml](src/hand_shape/param/hand_shape.yaml))
```bash
ros2 service call /hand_shape/set_shape omnihand_node_msgs/srv/SetHandShape "{shape_name: joke}"
```



## 依赖图
