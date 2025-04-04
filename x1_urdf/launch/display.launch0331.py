from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # 获取当前包的共享目录
    x1_urdf_dir = get_package_share_directory("x1_urdf")
    
    # 设置默认的URDF文件路径
    default_model_path = os.path.join(x1_urdf_dir, "urdf", "x1_25dof.urdf")
    # 设置默认的rviz配置文件路径
    default_rviz_path = os.path.join(x1_urdf_dir, "rviz", "display.rviz")

    # 声明一个launch参数，用于指定URDF文件路径
    model = DeclareLaunchArgument(
        name="model",
        default_value=default_model_path,
        description="Path to the URDF file"
    )

    # 加载机器人模型
    # 1. 启动 robot_state_publisher 节点并以参数方式加载 urdf 文件
    robot_description = ParameterValue(LaunchConfiguration("model"))
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    	arguments=["--ros-args", "--log-level", "debug"]
    )
    # 2. 启动 joint_state_publisher 节点发布非固定关节状态
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher"
    )
    # rviz2 节点
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", default_rviz_path]
    )

    return LaunchDescription([
        model,
        robot_state_publisher,
        joint_state_publisher,
        rviz2
    ])
