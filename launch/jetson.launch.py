from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os

def generate_launch_description():
    dirname = os.path.dirname(__file__)

    # Include camera launch file
    camera_launch_file_path = os.path.join(dirname, 'camera.launch.py')
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_file_path)
    )
    
    # Include apriltag detection launch file
    apriltag_launch_file_path = os.path.join(dirname, 'apriltag_detection.launch.py')
    apriltag_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(apriltag_launch_file_path))

    return LaunchDescription([
        camera_launch, 
        apriltag_launch
    ])
