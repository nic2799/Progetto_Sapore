from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    
    # Declare launch argument
    jsp_gui_arg = DeclareLaunchArgument(
        'jsp_gui',
        default_value='true',
        description='Use joint_state_publisher_gui (true) or joint_state_publisher (false)'
    )
    
    # Robot description (only robot, no obstacles)
    robot_description_content = ParameterValue(
        Command([
            'xacro',
            ' ',
            PathJoinSubstitution([
                FindPackageShare('iiwa_description'),
                'urdf',
                'dual_iiwa_robot.urdf.xacro'
            ])
        ]),
        value_type=str
    )
    robot_description = {'robot_description': robot_description_content}
    
    # Environment description (only obstacles/scene)
    environment_description_content = ParameterValue(
        Command([
            'xacro',
            ' ',
            PathJoinSubstitution([
                FindPackageShare('iiwa_description'),
                'urdf',
                'scene_environment.urdf.xacro'
            ])
        ]),
        value_type=str
    )
    environment_description = {'robot_description': environment_description_content}

    # RViz configuration file
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('iiwa_description'),
        'rviz',
        'iiwa.rviz'
    ])

    return LaunchDescription([
        jsp_gui_arg,
        
        # Robot State Publisher (per il robot)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),
        
        # Environment State Publisher (per gli ostacoli)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='environment_state_publisher',
            namespace='environment',
            output='screen',
            parameters=[environment_description],
            remappings=[
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static')
            ]
        ),
        #per far comandare il robot da nodes va messo jsp_gui a false ros2 launch iiwa_description dual_iiwa.launch.py jsp_gui:=false
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=UnlessCondition(LaunchConfiguration('jsp_gui')),
            output='screen',
            parameters=[
                {'source_list': ['left_arm/cmd/joint_position', 'right_arm/cmd/joint_position']}
                #joint state publisher legge gli stati dei giunti da questi topic e li pubblica su /joint_states
            ]
        ),
        
        # Joint State Publisher GUI (with GUI)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('jsp_gui')),
            output='screen',
            parameters=[
                robot_description
            ]
        ),

        
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[robot_description]
        )
        ,
        # JointState inspector (helps debug invalid JointState messages)
     #   ExecuteProcess(
      #      cmd=['/usr/bin/python3', '/home/nicola/OneDrive/Magistrale/AIRP/AIPR/ProgrammazionedeiRobot/EsercitazioneairpMio/ros_esercitazione/IIWA/src/nodes/scripts/joint_state_inspector.py'],
       #     output='screen'
        # )
    ])