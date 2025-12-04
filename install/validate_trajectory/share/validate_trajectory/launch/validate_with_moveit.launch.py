from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Argomenti
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
        default_value='0.5',
        description='Velocità di riproduzione (0.5 = metà velocità per analisi)'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config", 
        default_value="moveit.rviz", 
        description="RViz config file"
    )
    
    # Configurazione MoveIt
    moveit_config = (
        MoveItConfigsBuilder("iiwa", package_name="iiwa_config_moveit_config")
        .robot_description(file_path="config/urdf/dual_iiwa_robot.urdf.xacro")
        .robot_description_semantic(file_path="config/dual_iiwa_robot.srdf")
        .planning_scene_monitor(publish_robot_description=True, 
                               publish_robot_description_semantic=True)
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    
    # RViz config path
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [get_package_share_directory("iiwa_config_moveit_config"), "launch", rviz_base]
    )
    
    # Nodo move_group
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )
    
    # Nodo RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits
        ],
    )
    
    # Nodo robot_state_publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )
    
    # Nodo ros2_control
    ros2_controllers_path = PathJoinSubstitution([
        get_package_share_directory("iiwa_config_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    ])
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path
        ],
        output="both",
    )
    
    # Spawner dei controller
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller", "--controller-manager", "/controller_manager"],
    )
    
    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller", "--controller-manager", "/controller_manager"],
    )
    
    left_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_hand_controller", "--controller-manager", "/controller_manager"],
    )
    
    right_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_hand_controller", "--controller-manager", "/controller_manager"],
    )
    
    # Nodo di validazione (con ritardo per aspettare MoveIt)
    validate_node = Node(
        package='validate_trajectory',
        executable='validate_trajectory_node',
        name='validate_trajectory_node',
        output='screen',
        parameters=[{
            'yaml_file': LaunchConfiguration('yaml_file'),
            'plan_id': LaunchConfiguration('plan_id'),
            'playback_speed': LaunchConfiguration('playback_speed'),
        }],
        # Avvia dopo un po' per dare tempo a MoveIt di inizializzare
        prefix='bash -c "sleep 3 && $0 $@"'
    )
    
    return LaunchDescription([
        yaml_file_arg,
        plan_id_arg,
        playback_speed_arg,
        rviz_config_arg,
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        left_arm_controller_spawner,
        right_arm_controller_spawner,
        left_hand_controller_spawner,
        right_hand_controller_spawner,
        move_group_node,
        rviz_node,
        validate_node,
    ])
