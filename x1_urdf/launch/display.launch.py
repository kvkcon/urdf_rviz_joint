# Copyright 2023 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start Rviz2 and Joint State Publisher gui automatically \
        with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "urdf_path",
            default_value="robot/x1_29dof/urdf/x1_ros.urdf",
            description="Path to the URDF file relative to the package share directory.",
        )
    )
    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    urdf_path = LaunchConfiguration("urdf_path")

    # Get URDF via xacro
    # robot_description_content = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="xacro")]),
    #         " ",
    #         PathJoinSubstitution(
    #             [
    #                 FindPackageShare("x1_urdf"),
    #                 "arm/urdf",
    #                 "humanoid_dualArm_Jan02.urdf",
    #             ]
    #         ),
    #     ]
    # )
    import launch_ros.descriptions
    import os
    # robot_description_content = launch_ros.descriptions.ParameterValue(Command(['xacro ',os.path.join(FindPackageShare(package="x1_urdf").find("x1_urdf") ,'robot/x1_29dof/urdf/x1_ros.urdf')]), value_type=str)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("x1_urdf"),
                    urdf_path,
                ]
            ),
        ]
    )
    

    # robot_description_content = launch_ros.descriptions.ParameterValue(Command(['xacro ',os.path.join(FindPackageShare(package="pd_robot_description").find("pd_robot_description") ,'asap_g1/urdf/g1_29dof_anneal_23dof.urdf')]), value_type=str)

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }
    # robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("x1_urdf"), "rviz", "view_robot.rviz"]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(gui),
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )
    #
    nodes_to_start = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)

