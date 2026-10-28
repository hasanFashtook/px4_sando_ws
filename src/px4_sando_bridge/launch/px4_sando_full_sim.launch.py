#!/usr/bin/env python3
"""Full SITL simulation launch for PX4 Autopilot + SANDO Motion Planner.

This launch file coordinates:
1. Micro XRCE-DDS Agent & px4_sando_bridge_node (via px4_sando.launch.py)
2. SANDO motion planner (onboard_sando.launch.py from sando underlay)
3. Optional RViz2 visualization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace").perform(context)
    launch_rviz = LaunchConfiguration("launch_rviz").perform(context).lower() in ("true", "1")
    launch_planner = LaunchConfiguration("launch_planner").perform(context).lower() in ("true", "1")

    actions = []

    # 1. Include PX4 SANDO Bridge Launch
    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("px4_sando_bridge"),
                    "launch",
                    "px4_sando.launch.py",
                ]
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "auto_takeoff": LaunchConfiguration("auto_takeoff"),
            "takeoff_altitude": LaunchConfiguration("takeoff_altitude"),
            "launch_agent": LaunchConfiguration("launch_agent"),
            "agent_port": LaunchConfiguration("agent_port"),
            "yaw_mode": LaunchConfiguration("yaw_mode"),
        }.items(),
    )
    actions.append(bridge_launch)

    # 2. Include SANDO Onboard Planner Launch (from underlay workspace)
    if launch_planner:
        try:
            sando_share = get_package_share_directory("sando")
            sando_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(sando_share, "launch", "onboard_sando.launch.py")
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_hardware": "false",
                    "sim_env": "fake_sim",
                    "publish_odom": "false",
                    "use_onboard_localization": "false",
                    "skip_initial_yawing": "true",
                }.items(),
            )
            actions.append(sando_launch)
        except Exception as e:
            print(f"[px4_sando_full_sim] Warning: Could not locate sando package share: {e}")

    # 3. Optional RViz Visualization
    if launch_rviz:
        rviz_config = os.path.join(
            get_package_share_directory("sando"), "rviz", "default.rviz"
        )
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(LaunchConfiguration("launch_rviz")),
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="NX01"),
            DeclareLaunchArgument("launch_agent", default_value="false"),
            DeclareLaunchArgument("agent_port", default_value="8888"),
            DeclareLaunchArgument("launch_planner", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="false"),
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
                "yaw_mode",
                default_value="none",
                description="Yaw control mode for PX4 setpoint: 'none', 'zero', or 'follow'",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
