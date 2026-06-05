#!/usr/bin/env python3
"""Launch the fer_skills action server.

MoveGroupInterface only reads robot_description / robot_description_semantic /
robot_description_kinematics from the *local* node's parameters (or a topic
fallback). It does NOT call /move_group's parameter service. So we replicate
the same xacro + yaml loading that fer_moveit_launch.py does for move_group,
and set those parameters on fer_skill_server too.

move_group itself must still be running for planning/execution.
"""
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    arm_group = LaunchConfiguration("arm_group")
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    load_gripper = LaunchConfiguration("load_gripper")
    ee_id = LaunchConfiguration("ee_id")

    franka_description_share = get_package_share_directory("franka_description")

    fer_description = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([franka_description_share, "robots", "fer", "fer.urdf.xacro"]),
        " hand:=", load_gripper,
        " robot_ip:=", robot_ip,
        " ee_id:=", ee_id,
        " use_fake_hardware:=", use_fake_hardware,
        " fake_sensor_commands:=", fake_sensor_commands,
        " ros2_control:=false",
    ])
    robot_description = {
        "robot_description": ParameterValue(fer_description.perform(context), value_type=str)
    }

    fer_semantic_description = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([franka_description_share, "robots", "fer", "fer.srdf.xacro"]),
        " hand:=", load_gripper,
        " ee_id:=", ee_id,
    ])
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            fer_semantic_description.perform(context), value_type=str
        )
    }

    kinematics_yaml = load_yaml("panda_moveit_config", "config/kinematics.yaml")
    kinematics = {"robot_description_kinematics": kinematics_yaml}

    # Joint limits (velocity + acceleration) must be on this node's parameter
    # tree under `robot_description_planning`. MTC's AddTimeOptimalParameterization
    # adapter reads them from there; without it, every Connect plan fails with
    # "No acceleration limit was defined for joint <name>".
    joint_limits_yaml = load_yaml("panda_moveit_config", "config/joint_limits.yaml")
    joint_limits = {"robot_description_planning": joint_limits_yaml}

    # MTC's PipelinePlanner reads parameters from the LOCAL node's parameter
    # tree under the pipeline name ("ompl" by default). Without this block,
    # task_.init() throws "Planning plugin name is empty or not defined in
    # namespace 'ompl'".
    ompl_planning_pipeline_config = {
        "ompl": {
            "planning_plugins": ["ompl_interface/OMPLPlanner"],
            "request_adapters": [
                "default_planning_request_adapters/ResolveConstraintFrames",
                "default_planning_request_adapters/ValidateWorkspaceBounds",
                "default_planning_request_adapters/CheckStartStateBounds",
                "default_planning_request_adapters/CheckStartStateCollision",
            ],
            "response_adapters": [
                "default_planning_response_adapters/AddTimeOptimalParameterization",
                "default_planning_response_adapters/ValidateSolution",
                "default_planning_response_adapters/DisplayMotionPath",
            ],
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("panda_moveit_config", "config/ompl_planning.yaml")
    if ompl_planning_yaml:
        ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)

    skill_server = Node(
        package="fer_skills",
        executable="skill_server_node",
        name="fer_skill_server",
        output="both",
        emulate_tty=True,
        parameters=[
            {"use_sim_time": use_sim_time},
            {"arm_group": arm_group},
            robot_description,
            robot_description_semantic,
            kinematics,
            joint_limits,
            ompl_planning_pipeline_config,
        ],
    )

    return [skill_server]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("arm_group", default_value="fer_arm"),
        DeclareLaunchArgument("robot_ip", default_value=""),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("fake_sensor_commands", default_value="false"),
        DeclareLaunchArgument("load_gripper", default_value="true"),
        DeclareLaunchArgument("ee_id", default_value="franka_hand"),
        OpaqueFunction(function=launch_setup),
    ])
