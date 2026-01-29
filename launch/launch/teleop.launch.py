from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os.path

def generate_launch_description() -> LaunchDescription:
    joystick_launch_file_path = os.path.join(
        os.path.dirname(__file__), 'joystick.launch.py'
    )

    # Define the included launch description
    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joystick_launch_file_path),
    )

    #Axle manager for Hyflex node
    start_axle_manager_node = Node(
        package='axle_manager',
        executable='axle_manager',
        name='axle_manager',
        arguments=['--params-file '+os.path.join(get_package_share_directory('axle_manager'),'config','params.yaml')]
    )

    #VN100 Sensor IMU Node
    start_imu_node = Node(
        package='sensors_pkg',
        executable='imu',
        name='imu'
    )

    #Declare launch description and populate
    ld = LaunchDescription()

    #declare launch actions
    ld.add_action(joystick_launch)
    ld.add_action(start_axle_manager_node)

    return ld