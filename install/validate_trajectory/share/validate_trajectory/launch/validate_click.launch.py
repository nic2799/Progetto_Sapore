from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
  #  Nodo per Rastrelliera (braccio destro - right_arm)
    valid_trajectory_click_node_rastrelliera = Node(
        package='validate_trajectory',
        executable='Valid_trajectory_click',
        name='valid_trajectory_click_rastrelliera',
        output='screen',
        parameters=[{
            'yaml_file': '/home/nicola/Desktop/Progetto_Sapore/Rastrelliera_plans_.yaml',
            'group_name': 'right_arm',
            'max_click_distance': 0.5,  # 50cm soglia
        }]
    )
    
    # Nodo per Bimby Center (braccio sinistro - left_arm)
    valid_trajectory_click_node_bimby_center = Node(
        package='validate_trajectory',
        executable='Valid_trajectory_click',
        name='valid_trajectory_click_bimby_center',
        output='screen',
        parameters=[{
            'yaml_file': '/home/nicola/Desktop/Progetto_Sapore/Bimby_center_plans_.yaml',
            'group_name': 'left_arm',
            'max_click_distance': 0.5,  # 50cm soglia
        }]
    )

    return LaunchDescription([
        valid_trajectory_click_node_rastrelliera,
        valid_trajectory_click_node_bimby_center,
    ])
