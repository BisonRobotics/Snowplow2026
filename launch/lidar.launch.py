from launch_ros.actions import Node
from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    sick_scan_pkg_prefix = get_package_share_directory('sick_scan_xd')
    lms_launch_file_path = os.path.join(sick_scan_pkg_prefix, 'launch/sick_lms_1xx.launch')
    laserscan_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        output='screen',
        arguments=[
            lms_launch_file_path,
            #'nodename:=laserscan1',
            'hostname:=192.168.1.134',
            'tf_base_frame_id:=lidar_laser',
            'scan_freq:=25',
            'ang_res:=0.5',
            #'cloud_topic:=cloud1a',
            #'frame_id:=laserscan1/lidardata'
        ]
    )
    return LaunchDescription([
        laserscan_node])