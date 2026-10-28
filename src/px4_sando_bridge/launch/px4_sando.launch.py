#!/usr/bin/env python3
"""Bring up the PX4 <-> SANDO companion bridge stack."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() in ("true", "1")
    map_frame_id = LaunchConfiguration("map_frame_id").perform(context)
    ros_domain_id = LaunchConfiguration("ros_domain_id").perform(context)
    agent_port = LaunchConfiguration("agent_port").perform(context)
    agent_lib_path = LaunchConfiguration("agent_lib_path").perform(context)

    actions = [
        SetEnvironmentVariable("ROS_DOMAIN_ID", ros_domain_id),
        SetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4"),
    ]

    if LaunchConfiguration("launch_agent").perform(context).lower() in ("true", "1"):
        env = {}
        if agent_lib_path:
            env["LD_LIBRARY_PATH"] = (
                agent_lib_path + ":" + os.environ.get("LD_LIBRARY_PATH", "")
            )
        actions.append(
            ExecuteProcess(
                cmd=["MicroXRCEAgent", "udp4", "-p", agent_port],
                additional_env=env,
                output="screen",
            )
        )

    actions.append(
        Node(
            package="px4_sando_bridge",
            executable="px4_sando_bridge_node",
            name="px4_sando_bridge_node",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "use_sim_time": use_sim_time,
                    "drone_namespace": namespace,
                    "world_frame_id": map_frame_id,
                    "body_frame_id": f"{namespace}/base_link",
                    "auto_takeoff": LaunchConfiguration("auto_takeoff"),
                    "takeoff_altitude": LaunchConfiguration("takeoff_altitude"),
                    "enable_acceleration_feedforward": LaunchConfiguration("enable_acceleration_feedforward"),
                    "yaw_mode": LaunchConfiguration("yaw_mode"),
                },
            ],
        )
    )

    actions.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_world_to_map",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "world",
                "--child-frame-id", "map",
            ],
        )
    )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("namespace", default_value="NX01"),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("px4_sando_bridge"),
                        "config",
                        "bridge_params.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "map_frame_id",
                default_value="world",
                description="World/map frame shared by the bridge and SANDO",
            ),
            DeclareLaunchArgument(
                "ros_domain_id",
                default_value=os.environ.get("ROS_DOMAIN_ID", "0"),
                description="DDS domain used by all ROS 2 nodes in this stack",
            ),
            DeclareLaunchArgument(
                "agent_port",
                default_value="8888",
                description="UDP port of MicroXRCEAgent (must match PX4_UXRCE_DDS_PORT)",
            ),
            DeclareLaunchArgument(
                "agent_lib_path",
                default_value="/home/hasan-fashtouk/code/lib",
                description="Directory containing the libtinyxml2 compatibility library",
            ),
            DeclareLaunchArgument("launch_agent", default_value="false"),
            DeclareLaunchArgument(
                "auto_takeoff",
                default_value="true",
                description="Automatically arm, takeoff, and transition to offboard mode",
            ),
            DeclareLaunchArgument(
                "takeoff_altitude",
                default_value="2.5",
                description="Target altitude in meters for automatic takeoff",
            ),
            DeclareLaunchArgument(
                "enable_acceleration_feedforward",
                default_value="true",
                description="Enable acceleration feedforward in offboard control mode",
            ),
            DeclareLaunchArgument(
                "yaw_mode",
                default_value="none",
                description="Yaw control mode for PX4 setpoint: 'none', 'zero', or 'follow'",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
