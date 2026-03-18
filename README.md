<!-- # 基于moveit的路径规划实现自动规划手指移动的功能 -->
# 基于 ROS2 + MoveIt2 + mujoco 的灵巧手仿真

## 配置流程

cd src/
git clone https://github.com/dfki-ric/mujoco_ros2_control


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
