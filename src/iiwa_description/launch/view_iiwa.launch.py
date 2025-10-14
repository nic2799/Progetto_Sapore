from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_description_content = Command([
        'xacro',
        ' ',
        PathJoinSubstitution([
            FindPackageShare('iiwa_description'),
            'config',
            'iiwa.config.xacro'
        ])
    ])
    robot_description = {'robot_description': robot_description_content}

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('iiwa_description'),
        'rviz',
        'iiwa.rviz'
    ])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[robot_description]
        )
    ])