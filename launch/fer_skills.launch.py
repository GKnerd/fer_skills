#!/usr/bin/env python3
"""Launch the fer_skills action server.

The skill server uses MoveIt's MoveGroupInterface, which means it must be
launched into the same process tree where the MoveIt config (robot_description,
robot_description_semantic, kinematics, joint_limits) is loaded — the skill
server inherits those parameters via /move_group's parameter discovery.

So: bring up MoveIt first (or together with this), then launch this. This
launch file does NOT start MoveIt itself.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    arm_group = LaunchConfiguration("arm_group")

    skill_server = Node(
        package="fer_skills",
        executable="skill_server_node",
        name="fer_skill_server",
        output="both",
        emulate_tty=True,
        parameters=[
            {"use_sim_time": use_sim_time},
            {"arm_group": arm_group},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use the simulated /clock.",
        ),
        DeclareLaunchArgument(
            "arm_group",
            default_value="fer_arm",
            description="MoveIt planning group for the arm.",
        ),
        skill_server,
    ])
