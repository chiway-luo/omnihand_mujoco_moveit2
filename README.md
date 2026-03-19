<!-- # 基于moveit的路径规划实现自动规划手指移动的功能 -->
# 基于 ROS2 + MoveIt2 + mujoco 的灵巧手仿真

## 项目介绍
> 本项目基于 ROS2 Humble + MoveIt2 Humble + Mujoco 2.3.0 实现了一个灵巧手的仿真系统。该系统能够通过 MoveIt2 进行路径规划，实现自动规划手指移动的功能。同时，项目还提供了一个 hand_shape 服务端节点，可以根据不同的手型需求进行转换。

> 项目集成了手部模型描述、MoveIt2 规划配置、MuJoCo 仿真启动、手型服务转换以及控制接口，适用于灵巧手动作规划、手型切换与仿真验证。


<video src="https://github.com/user-attachments/assets/5eac4ab7-fb61-4218-af0c-b66531f1a26a" controls="controls" width="600">
</video>

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



## 关节控制器
> 目前的控制器配置为一个整体的hand_controller,如果需要单独控制每个手指,可以在`moveit_controllers.yaml`和`ros2_controllers.yaml`中取消注释对应的控制器配置,并在`hand_mujoco.launch.py`中添加对应的控制器spawner节点。

```bash
ros2 control list_controllers

hand_controller         joint_trajectory_controller/JointTrajectoryController  active
joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster          active
```

