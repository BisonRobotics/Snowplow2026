from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

import os

def generate_launch_description():
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        parameters=[
            ParameterFile(os.path.join(get_package_share_directory('sensors_pkg'),'apriltag_params.yaml')),
        ],
        remappings=[
            ('image_rect', '/hyflex/camera/image_color'),
            ('camera_info', '/hyflex/camera/camera_info'),
        ]
    )

    return LaunchDescription([
        apriltag_node
    ])