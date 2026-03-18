#install(DIRECTORY config params launch DESTINATION share/${PROJECT_NAME}) #cmake配置
#<exec_depend>ros2launch</exec_depend> <!--package.xml配置-->
#from glob import glob #用于setup.py配置多个launch文件
#('share/' + package_name + '/launch', glob('launch/*launch.py')),
#('share/' + package_name + '/launch', glob('launch/*launch.xml')),
#('share/' + package_name + '/launch', glob('launch/*launch.yaml')),
#(os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    ld = LaunchDescription()

    # ========== 加载MoveIt配置 ==========
    # 使用MoveItConfigsBuilder从hand_moveit功能包中加载所有MoveIt配置
    # 包括：URDF机器人描述、SRDF语义描述、关节限制、运动学求解器等
    # "omnihand"为SRDF中定义的机器人名称，"hand_moveit"为配置包名
    moveit_config = MoveItConfigsBuilder(
        "omnihand", package_name="hand_moveit"
    ).to_moveit_configs()

    # 获取hand_moveit功能包的share目录路径，后续用于定位launch和config文件
    hand_moveit_dir = get_package_share_directory("hand_moveit")

    # ========== 1. 静态TF发布（虚拟关节） ==========
    # 发布base_link到world之间的静态坐标变换
    # 该变换在moveit配置助手中定义，用于将机器人固定在世界坐标系中
    # 如果该launch文件不存在（未配置虚拟关节），则跳过
    static_tf_launch = os.path.join(
        hand_moveit_dir, "launch", "static_virtual_joint_tfs.launch.py"
    )
    if os.path.exists(static_tf_launch):
        ld.add_action(
            IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource(static_tf_launch)
            )
        )

    # ========== 2. robot_state_publisher ==========
    # 加载URDF模型并订阅/joint_states话题
    # 根据关节角度计算所有连杆的TF坐标变换并发布
    # 这样RViz和MoveIt才能正确显示机器人的实时姿态
    ld.add_action(
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                os.path.join(hand_moveit_dir, "launch", "rsp.launch.py")
            )
        )
    )

    # ========== 3. ros2_control_node（硬件抽象层） ==========
    # 启动ros2_control控制器管理节点，加载fake hardware模拟硬件接口
    # 参数1：robot_description - URDF中定义的ros2_control硬件接口
    # 参数2：ros2_controllers.yaml - 各关节控制器的配置参数
    #
    # 关键：将/joint_states重映射为/controller_joint_states
    # 原因：fake controller的轨迹插值输出会发布关节状态，
    #       如果不重映射就会覆盖真实驱动反馈的/joint_states，
    #       导致MoveIt拿到的是插值数据而非真实关节角度
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                os.path.join(hand_moveit_dir, "config", "ros2_controllers.yaml"),
            ],
            remappings=[("/joint_states", "/controller_joint_states")],
            output="screen",
        )
    )

    # ========== 4. 启动关节控制器 ==========
    # 为每根手指启动一个JointTrajectoryController，负责接收MoveIt规划的轨迹
    # 并进行时间插值，输出平滑的关节角度序列
    # - thumb_controller:   大拇指轨迹控制器
    # - index_controller:   食指轨迹控制器
    # - middle_controller:  中指轨迹控制器
    # - ring_controller:    无名指轨迹控制器
    # - little_controller:  小指轨迹控制器
    # - joint_state_broadcaster: 将控制器内部的关节状态广播到/controller_joint_states
    #   hand_control桥接节点订阅该话题，将轨迹数据转发给真实灵巧手驱动
    controller_names = [
        "thumb_controller",
        "index_controller",
        "middle_controller",
        "ring_controller",
        "little_controller",
        "joint_state_broadcaster",
    ]
    for controller in controller_names:
        ld.add_action(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
                output="screen",
            )
        )

    # ========== 5. move_group（MoveIt核心规划节点） ==========
    # MoveIt的核心节点，提供运动规划、碰撞检测、逆运动学求解等功能
    # 接收来自RViz交互界面的规划请求，计算无碰撞轨迹
    # 并通过action接口将轨迹发送给对应的JointTrajectoryController执行
    ld.add_action(
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                os.path.join(hand_moveit_dir, "launch", "move_group.launch.py")
            )
        )
    )

    # ========== 6. RViz可视化 ==========
    # 启动带有MoveIt插件的RViz2，提供交互式运动规划界面
    # 用户可以在RViz中拖拽目标姿态、执行规划和发送执行指令
    # 加载hand_moveit中预配置的rviz配置文件
    ld.add_action(
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                os.path.join(hand_moveit_dir, "launch", "moveit_rviz.launch.py")
            )
        )
    )

    # ========== 7. 灵巧手底层驱动节点 ==========
    # 启动驱动节点
    # 负责通过串口与灵巧手硬件通信，接收电机控制指令并反馈真实关节角度
    # 订阅：/agihand/omnihand/left/motor_angle_cmd（电机角度指令）
    # 订阅：/agihand/omnihand/left/control_mode_cmd（电机控制模式）
    # 发布：真实关节状态（由hand_control节点计算并发布到/joint_states）
    ld.add_action(
        Node(
            package="omnihand_node",
            executable="hand_node",
            name="hand_driver_node",
            output="screen",
        )
    )

    # ========== 8. hand_control桥接节点 ==========
    # 连接MoveIt规划系统和灵巧手底层驱动的桥梁
    # 数据流向：
    #   MoveIt轨迹执行 → /controller_joint_states → hand_control → 电机角度指令 → 灵巧手驱动
    #   灵巧手驱动 → 真实关节反馈 → hand_control → /joint_states → MoveIt/RViz
    ld.add_action(
        Node(
            package="hand_control",
            executable="hand_control",
            name="hand_control_node",
            output="screen",
        )
    )

    # =========== 9. hand_shape 手型改变节点========
    hand_shape_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("hand_shape"), "launch", "hand_shape.launch.py")
        )
    )
    ld.add_action(hand_shape_launch)

    return ld

