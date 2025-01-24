import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config: str = os.path.join(
        get_package_share_directory("rerun-insider"),
        "config",
        "insider.yaml",
    )

    desc = LaunchDescription(
        [
            Node(
                package="rerun-insider",
                executable="insider",
                name="rerun_insider",
                namespace="",
                output="screen",
                parameters=[config],
            ),
        ]
    )

    return desc
