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
# 封装终端指令相关类--------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable   #FindExecutable(name="ros2")
# 参数声明与获取-----------------
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch.conditions import IfCondition #判断是否执行
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

def generate_launch_description():
    ld = LaunchDescription()

    # 启动灵巧手底层驱动节点
    sdk_omnihand_node = Node(
        package="omnihand_node",
        executable="hand_node",
        name="omnihand_node",
        output="screen",
    )
    ld.add_action(sdk_omnihand_node)

    # 获取urdf文件路径
    urdf_file = os.path.join(
        get_package_share_directory('hand_description'),
        'urdf',
        'omnihand_left.urdf'
    )

    # 读取urdf文件
    urdf_txt = Command(['xacro ', urdf_file])

    # 创建描述节点
    description_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="hand_description_node",
        output="screen",
        parameters=[{"robot_description": urdf_txt}]
    )
    ld.add_action(description_node)

    # 创建关节状态发布节点（提供 TF 变换给 rviz）gui
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )
    ld.add_action(joint_state_publisher_gui_node)

    # 启动灵巧手测试节点
    hand_test_node = Node(
        package="hand_test_bag",
        executable="hand_test",
        name="hand_test",
        output="screen",
    )
    ld.add_action(hand_test_node)

    # 启动 rviz2 可视化工具
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz_node",
        output="screen",
        arguments=["-d", os.path.join(get_package_share_directory('hand_test_bag'), 'rviz', 'default.rviz')]
    )
    ld.add_action(rviz_node)

    return ld
