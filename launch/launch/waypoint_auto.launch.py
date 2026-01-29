from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    start_waypoint = Node(
        package='waypoint',
        executable='waypoint',
        name='waypoint'
    )

    #Auto control node
    start_auto_node = Node(
        package='control_pkg',
        executable='auto',
        name='auto'
    )

    return LaunchDescription([start_waypoint, start_auto_node])
