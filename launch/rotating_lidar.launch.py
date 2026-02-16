import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rotating_lidar'),
        'config',
        'rotating_lidar_params.yaml'
    )

    auto_start_arg = DeclareLaunchArgument(
        'auto_start_motor',
        default_value='false',
        description='Automatically start the motor on launch'
    )

    motor_controller = Node(
        package='rotating_lidar',
        executable='motor_controller_node.py',
        name='motor_controller_node',
        output='screen',
        parameters=[config],
    )

    hall_sensor = Node(
        package='rotating_lidar',
        executable='hall_sensor_node.py',
        name='hall_sensor_node',
        output='screen',
        parameters=[config],
    )

    pointcloud_rotator = Node(
        package='rotating_lidar',
        executable='pointcloud_rotator_node',
        name='pointcloud_rotator_node',
        output='screen',
    )

    return LaunchDescription([
        auto_start_arg,
        motor_controller,
        hall_sensor,
        pointcloud_rotator,
    ])
