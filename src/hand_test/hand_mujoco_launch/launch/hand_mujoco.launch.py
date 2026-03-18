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
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关----------------------
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
# 事件相关----------------------
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo
# 获取功能包下share目录路径-------
from ament_index_python.packages import get_package_share_directory
# urdf文件处理相关--------------
# from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
# 组件相关-------------
# from launch_ros.actions import ComposableNodeContainer
# from launch_ros.descriptions import ComposableNode

def create_nodes(context):
    namespace = ""
    mujoco_model_path = "/tmp/mujoco_hand"
    mujoco_model_file = os.path.join(mujoco_model_path, "main.xml")

    rviz = LaunchConfiguration("rviz")

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
    rviz_config_file = os.path.join(
        get_package_share_directory("hand_description"),
        "rviz",
        "omnihand_left.rviz",
    )
    rviz_node = Node(
        condition=IfCondition(rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
    )

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
                rviz_node,
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

    # mujoco模型文件路径
    mujoco_model_path = os.path.join(get_package_share_directory("hand_description"),"mjcf")
    # 模型文件
    mujoco_model_file = os.path.join(
        mujoco_model_path,
        # "my_hand.xml"
        "scene.xml"
    )

    # cmd 启动mujoco节点
    mujoco_gui = ExecuteProcess(
        cmd=["mujoco", mujoco_model_file],
        output="screen"
    )
    # ld.add_action(mujoco_gui)

    # 参数声明
    ld.add_action(DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Start rviz.",
        ))
    
    # 创建节点
    ld.add_action(OpaqueFunction(function=create_nodes))

    return ld



