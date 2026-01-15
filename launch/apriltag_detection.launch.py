from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

import yaml
import os

def generate_launch_description():
    package_share_directory = get_package_share_directory('sensors_pkg')

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        parameters=[
            ParameterFile(os.path.join(package_share_directory, 'apriltag_params.yaml')),
        ],
        remappings=[
            ('image_rect', '/hyflex/camera/image_color'),
            ('camera_info', '/hyflex/camera/camera_info'),
        ]
    )
    
    # Define the path to your tag poses YAML file
    tag_poses_yaml_path = os.path.join(package_share_directory, 'arena_tag_poses.yaml')

    # Load the YAML data
    with open(tag_poses_yaml_path, 'r') as f:
        tag_config = yaml.safe_load(f)
        tag_poses = tag_config.get('tag_poses', {})

    # --- 2. Create Static TF Publishers from Config ---
    
    static_tf_nodes = []
    
    # Iterate through the dictionary to create a publisher for each tag
    for tag_name, pose_data in tag_poses.items():
        translation = [str(x) for x in pose_data['translation']]
        rotation = [str(x) for x in pose_data['rotation']]
        parent_frame = pose_data['parent_frame']

        # Arguments order: X Y Z Yaw Pitch Roll | OR | X Y Z Qx Qy Qz Qw | Parent Frame Child Frame
        tf_args = translation + rotation + [parent_frame, tag_name]

        node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{tag_name.replace(":", "_")}_tf_publisher', # Ensure node name is ROS-compliant
            arguments=tf_args,
            output='screen'
        )
        static_tf_nodes.append(node)
        
    # --- 3. Return Launch Description ---
    return LaunchDescription([
        apriltag_node,
        *static_tf_nodes
    ])