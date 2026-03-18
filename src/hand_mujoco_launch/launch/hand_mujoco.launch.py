#install(DIRECTORY config params launch DESTINATION share/${PROJECT_NAME}) #cmake配置
#<exec_depend>ros2launch</exec_depend> <!--package.xml配置-->
#from glob import glob #用于setup.py配置多个launch文件
#('share/' + package_name + '/launch', glob('launch/*launch.py')),
#('share/' + package_name + '/launch', glob('launch/*launch.xml')),
#('share/' + package_name + '/launch', glob('launch/*launch.yaml')),
#(os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
    OpaqueFunction,
)
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node
import os
# 封装终端指令相关类--------------
from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable   #FindExecutable(name="ros2")
# 参数声明与获取-----------------
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition #判断是否执行
# from launch.conditions import UnlessCondition #取反
# from launch.substitutions import PythonExpression #运行时计算表达式
# 文件包含相关-------------------
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关----------------------
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
# 事件相关----------------------
from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo
# 获取功能包下share目录路径-------
from ament_index_python.packages import get_package_share_directory
# urdf文件处理相关--------------
# from launch_ros.parameter_descriptions import ParameterValue
# from launch.substitutions import Command
# 组件相关-------------
# from launch_ros.actions import ComposableNodeContainer
# from launch_ros.descriptions import ComposableNode
from launch_ros.actions import SetParameter
from moveit_configs_utils import MoveItConfigsBuilder #从moveit_configs_utils包中导入MoveItConfigsBuilder类，用于加载MoveIt配置

def create_nodes(context):
    namespace = ""
    mujoco_model_path = "/tmp/mujoco_hand"
    mujoco_model_file = os.path.join(mujoco_model_path, "main.xml")

    # rviz = LaunchConfiguration("rviz")

    # 读取 URDF（已内嵌 ros2_control MujocoSystemInterface）
    urdf_file = os.path.join(
        get_package_share_directory("hand_description"),
        "urdf",
        "omnihand_left.urdf",
    )
    with open(urdf_file, "r") as f:
        robot_description_content = f.read()

    robot_description = {"robot_description": robot_description_content}

    # scene.xml 作为额外输入文件
    additional_files = [
        os.path.join(get_package_share_directory("hand_description"), "mjcf", "scene.xml")
    ]

    # xacro2mjcf: 将 URDF 转为 MuJoCo XML
    xacro2mjcf = Node(
        package="mujoco_ros2_control",
        executable="xacro2mjcf.py",
        parameters=[
            {"robot_descriptions": [robot_description_content]},
            {"input_files": additional_files},
            {"output_file": mujoco_model_file},
            {"mujoco_files_path": mujoco_model_path},
            {"base_link": "base_link"},
            {"floating": False},
        ],
    )

    # robot_state_publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        parameters=[robot_description],
    )

    # ros2_control 控制器配置文件
    ros2_control_params_file = os.path.join(
        get_package_share_directory("hand_moveit"),
        "config",
        "ros2_controllers.yaml",
    )

    # mujoco_ros2_control 仿真节点
    mujoco = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        namespace=namespace,
        parameters=[
            robot_description,
            ros2_control_params_file,
            {"simulation_frequency": 500.0},
            {"realtime_factor": 1.0},
            {"robot_model_path": mujoco_model_file},
            {"show_gui": True},
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )

    # xacro2mjcf 完成后启动 mujoco
    start_mujoco = RegisterEventHandler(
        OnProcessExit(
            target_action=xacro2mjcf,
            on_exit=[
                LogInfo(msg="Created mujoco xml, starting mujoco node..."),
                mujoco,
            ],
        )
    )

    # 控制器 spawner
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    thumb_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["thumb_controller", "--controller-manager", "/controller_manager"],
    )
    index_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["index_controller", "--controller-manager", "/controller_manager"],
    )
    middle_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["middle_controller", "--controller-manager", "/controller_manager"],
    )
    ring_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ring_controller", "--controller-manager", "/controller_manager"],
    )
    little_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["little_controller", "--controller-manager", "/controller_manager"],
    )

    # RViz
    # rviz_config_file = os.path.join(
    #     get_package_share_directory("hand_description"),
    #     "rviz",
    #     "omnihand_left.rviz",
    # )
    # rviz_node = Node(
    #     condition=IfCondition(rviz),
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_config_file],
    #     parameters=[{"use_sim_time": True}],
    # )

    # mujoco 启动后加载控制器
    load_controllers = RegisterEventHandler(
        OnProcessStart(
            target_action=mujoco,
            on_start=[
                LogInfo(msg="Starting controllers..."),
                load_joint_state_broadcaster,
                thumb_controller,
                index_controller,
                middle_controller,
                ring_controller,
                little_controller,
            ],
        )
    )

    return [
        robot_state_publisher,
        xacro2mjcf,
        start_mujoco,
        load_controllers,
    ]


def generate_launch_description():
    ld = LaunchDescription()

    # 全局设置 use_sim_time，所有节点都使用仿真时钟
    ld.add_action(SetParameter(name="use_sim_time", value=True))


    # 参数声明
    # ld.add_action(DeclareLaunchArgument(
    #         "rviz",
    #         default_value="true",
    #         description="Start rviz.",
    #     ))
    
    # 创建节点
    ld.add_action(OpaqueFunction(function=create_nodes))

    # 启动moveit2的launch文件
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

    # =========== 9. hand_shape 手型改变节点========
    hand_shape_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("hand_shape"), "launch", "hand_shape.launch.py")
        )
    )
    ld.add_action(hand_shape_launch)

    return ld



