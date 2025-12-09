from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

def generate_launch_description():
    #VN100 Sensor IMU Node
    ld = LaunchDescription()
    start_imu_node = Node(
      package='sensors_pkg',
      executable='imu',
      name='imu'  )
    ld.add_action(start_imu_node)

    return ld
