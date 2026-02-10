from launch import LaunchDescription
from launch_ros.actions import Node

import os.path

def generate_launch_description():
    
    
    

    #location calculate node
    start_location_node = Node(
        package='control_pkg',
        executable='location_calculate',
        name='location_node'
    )



    #Declare launch description and populate
    ld = LaunchDescription()

    #declare launch actions
    ld.add_action(start_location_node)

    return ld