"""Bridge the Gazebo 3D LiDAR into the ROS 2 topics used by the stack.

MIGHTY and ``global_mapper_ros`` resolve their point-cloud inputs relative to
the drone namespace (``/NX01/mid360_PointCloud2``).  The integration contract
also exposes the plain ``/mid360_PointCloud2`` topic, so the cloud is published
on both names.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gz_topic_arg = DeclareLaunchArgument(
        'gz_lidar_topic',
        default_value='/lidar/points/points',
        description='Gazebo Transport topic name for the 3D LiDAR point cloud'
    )

    ros_topic_arg = DeclareLaunchArgument(
        'ros_lidar_topic',
        default_value='/mid360_PointCloud2',
        description='Global ROS 2 topic name for the bridged PointCloud2'
    )

    ns_topic_arg = DeclareLaunchArgument(
        'namespaced_lidar_topic',
        default_value='/NX01/mid360_PointCloud2',
        description='Namespaced ROS 2 topic consumed by MIGHTY and the mapper'
    )

    bridge_type = '@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'

    global_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_lidar_bridge',
        output='screen',
        arguments=[[LaunchConfiguration('gz_lidar_topic'), bridge_type]],
        remappings=[
            (LaunchConfiguration('gz_lidar_topic'),
             LaunchConfiguration('ros_lidar_topic'))
        ]
    )

    namespaced_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_lidar_bridge_namespaced',
        output='screen',
        arguments=[[LaunchConfiguration('gz_lidar_topic'), bridge_type]],
        remappings=[
            (LaunchConfiguration('gz_lidar_topic'),
             LaunchConfiguration('namespaced_lidar_topic'))
        ]
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    return LaunchDescription([
        gz_topic_arg,
        ros_topic_arg,
        ns_topic_arg,
        clock_bridge,
        global_bridge,
        namespaced_bridge,
    ])
