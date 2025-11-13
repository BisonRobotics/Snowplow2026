from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    joy_name_param = DeclareLaunchArgument(
        "joy_name",
        default_value="/dev/input/js0",
        description="Full path to Joystick Device",
    )

    joy_name = LaunchConfiguration("joy_name")

    #Joystick address option
    declare_joy_address = DeclareLaunchArgument(
        name='joy_name',
        default_value=joy_name,
        description="Full path to Joystick Device"
    )

    #Base Joystick node
    start_joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        parameters=[{'dev_name': joy_name, 'deadzone':0.0}]
    )

    #Joystick conversion for Hyflex node
    start_joy_conv_node = Node(
        package='joy_conv',
        executable='joy_conv',
        name='joy_conv',
    )

    return LaunchDescription([
        joy_name_param,
        declare_joy_address,
        start_joy_node,
        start_joy_conv_node
    ])