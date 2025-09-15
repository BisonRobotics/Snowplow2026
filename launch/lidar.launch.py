import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    sick_scan_pkg_prefix = get_package_share_directory('sick_scan_xd')
    lms_launch_file_path = os.path.join(sick_scan_pkg_prefix, 'launch/sick_lms_1xx.launch')
    laser_scan_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        output='screen',
        arguments=[
            lms_launch_file_path,
            'hostname:=192.168.1.134',
            'tf_base_frame_id:=lidar_laser',
            'scan_freq:=25',
            'ang_res:=0.5',
            ]
    )
    
    obs_node = Node(
        package='sensors_pkg',
        executable='obs',
        name='obs',
    )
    
    return LaunchDescription([
        laser_scan_node,
        obs_node
    ])

