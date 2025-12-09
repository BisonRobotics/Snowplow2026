import os
import launch
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('simulation')
    robot_description_path = os.path.join(package_dir, 'resource', 'hyflex.urdf')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'hyflex_world.wbt')
    )

    hyflex_driver = WebotsController(
        robot_name='hyflex',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    # Include keyboard launch
    keyboard_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(os.path.dirname(__file__), '..', 'keyboard.launch.py')
        )
    )

    return LaunchDescription([
        webots,
        hyflex_driver,
        keyboard_launch,
        launch.actions.RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=Shutdown())],
            )
        )
    ])