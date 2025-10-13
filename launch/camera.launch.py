from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_pkg',
            executable='camera_node',
            name='camera',
            parameters=[
                {'fx':1071.1362274102335},
                {'fy':1102.1406887400624},
                {'cx':953.030188084331},
                {'cy':468.0382502048589}
            ],
            namespace='camera'
        )
    ])
