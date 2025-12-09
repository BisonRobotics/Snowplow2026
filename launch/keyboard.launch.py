from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Keyboard conversion node
    start_keyboard_conv_node = Node(
        package='joy_conv',
        executable='keyboard_conv',
        name='keyboard_conv',
    )

    return LaunchDescription([
        start_keyboard_conv_node
    ])