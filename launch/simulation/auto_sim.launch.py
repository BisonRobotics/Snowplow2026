import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    package_dir = get_package_share_directory('simulation')
    robot_description_path = os.path.join(package_dir, 'resource', 'hyflex.urdf')

    # Auto control node
    start_auto_node = Node(
        package='control_pkg',
        executable='auto',
        name='auto'
    )

    # Webots launcher
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'hyflex_world.wbt'),
    )

    # Hyflex robot controller
    hyflex_driver = WebotsController(
        robot_name='hyflex',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    return LaunchDescription([
        start_auto_node,
        webots,
        hyflex_driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])