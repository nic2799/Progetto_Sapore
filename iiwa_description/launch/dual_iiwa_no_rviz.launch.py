from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Load the dual IIWA robot description
    robot_description_content = ParameterValue(
        Command([
            'xacro',
            ' ',
            PathJoinSubstitution([
                FindPackageShare('iiwa_description'),
                'config',
                'dual_iiwa.config.xacro'
            ])
        ]),
        value_type=str
    )
    robot_description = {'robot_description': robot_description_content}

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),
        
        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
            parameters=[robot_description]
        )
    ])