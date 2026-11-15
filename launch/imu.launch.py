import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory("imu_yzf143_ros2"), "config", "imu.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="Path to the IMU driver parameter YAML file",
        ),
        Node(
            package="imu_yzf143_ros2",
            executable="imu_node",
            name="imu_yzf143_node",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])
