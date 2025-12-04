from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Argomenti configurabili
    yaml_file_arg = DeclareLaunchArgument(
        'yaml_file',
        default_value='plans_20251107_143536.yaml',
        description='Nome del file YAML con le traiettorie'
    )
    
    plan_id_arg = DeclareLaunchArgument(
        'plan_id',
        default_value='1',
        description='ID del piano da validare'
    )
    
    playback_speed_arg = DeclareLaunchArgument(
        'playback_speed',
        default_value='1.0',
        description='Velocità di riproduzione (1.0 = normale, 2.0 = doppia velocità)'
    )
    
    # Nodo di validazione
    validate_node = Node(
        package='validate_trajectory',
        executable='validate_trajectory_node',
        name='validate_trajectory_node',
        output='screen',
        parameters=[{
            'yaml_file': LaunchConfiguration('yaml_file'),
            'plan_id': LaunchConfiguration('plan_id'),
            'playback_speed': LaunchConfiguration('playback_speed'),
        }]
    )
    
    return LaunchDescription([
        yaml_file_arg,
        plan_id_arg,
        playback_speed_arg,
        validate_node,
    ])
