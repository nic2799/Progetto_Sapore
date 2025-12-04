import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # Dichiarazione dell'argomento rviz_config
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config", default_value="moveit.rviz", description="RViz config file"
    )

    # Usa l'argomento rviz_config
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [get_package_share_directory("iiwa_config_moveit_config"), "launch", rviz_base]
    )

    # Configurazione MoveIt
    moveit_config = (
        MoveItConfigsBuilder("iiwa", package_name="iiwa_config_moveit_config")
        .robot_description(file_path="config/urdf/dual_iiwa_robot.urdf.xacro")
        .robot_description_semantic(file_path="config/dual_iiwa_robot.srdf")
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

#     # Dichiarazione hardware ros2_control
#     ros2_control_hardware_type = DeclareLaunchArgument(
#     "ros2_control_hardware_type",
#     default_value="mock_components",
#     description="Type of ros2_control hardware interface"
# )


    # Nodo move_group
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(),
                   {"use_sim_time": True},
        ],
    )

    # Nodo RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz_demo",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
           
        ],
        ros_arguments=['--log-level', 'rviz2:=FATAL']
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
    ros2_controllers_path = os.path.join(
        get_package_share_directory("iiwa_config_moveit_config"),
        "config/",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path
                    
                    , moveit_config.robot_description],
        name="controller_manager",
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
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

    build_scene = Node(
        package="build_scene_package",
        executable="build_scene_simple",
        name="build_scene_mesh_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits
        ],

    )

    # Reachability_interactive = Node(
    #     package="reachability_package",
    #     executable="Reac_marker",
    #     name="reachability_tavolo", 
    #     output="screen",
    #     parameters=[
    #         moveit_config.robot_description,
    #         moveit_config.robot_description_semantic,
    #         moveit_config.robot_description_kinematics,
    #         {'arm_side': 'right'},
            
    #     ],
    # )
    # Interactive_Marker_Node = Node(
    #     package="reachability_package",
    #     executable="interactive_reach",
    #     name="interactive_marker_server",
    #     output="screen",
    # )

# Nodo Joint State Publisher GUI (per mock_components)
    # joint_state_publisher_gui = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui",
    #     name="joint_state_publisher_gui",
    #     output="screen",
    # )

  


    # Ritorna la LaunchDescription
    return LaunchDescription([
    rviz_config_arg,
   # ros2_control_hardware_type,
    robot_state_publisher,
    ros2_control_node,
    joint_state_broadcaster_spawner,
    left_arm_controller_spawner,
    right_arm_controller_spawner,
    left_hand_controller_spawner,
    right_hand_controller_spawner,
    move_group_node,
    rviz_node,
    build_scene,
   # Reachability_interactive,
    #Interactive_Marker_Node,
   # joint_state_publisher_gui,
])
