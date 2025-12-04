from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import xacro

def generate_launch_description():

    urdf_path = Path(
        get_package_share_directory("iiwa_description")
    ) / "urdf" / "dual_iiwa_robot.urdf.xacro"

    moveit_config = MoveItConfigsBuilder("iiwa_config").to_moveit_configs()
    robot_description_content = xacro.process_file(str(urdf_path)).toxml()
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

#     # Nodo Reachability per la pedana
#     reachability_node_tavolo = Node(
#         package="nodes",
#         executable="Reach",
#         name="reachability_tavolo", 
#         output="screen",
#         parameters=[
#             robot_description,
#             moveit_config.robot_description_semantic,
#             moveit_config.robot_description_kinematics,
#             {
#                 "group_name": "right_",
#                 "xmin": -1.05, "xmax": -0.78,
#                 "ymin": -2.90, "ymax": -1.30,
#                 "zmin": 0.85,  "zmax": 1.0,
#                 "roll": 1.57, "pitch": 1.57, "yaw": 0.0,
#                 "step": 0.1,
#             },
#         ],
#     )

  #  Nodo Reachability per il tavolo
    reachability_node_rastrelliera = Node(
        package="nodes",
        executable="Reach",
        name="reachability_rastrelliera",  
        output="screen",
        parameters=[
            robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {
                "group_name": "right_",
                "xmin": -0.10, "xmax": -0.09,
                "ymin": -1.4,  "ymax": -0.6,
                "zmin": 0.2,   "zmax": 1.8,
                "roll": 0.0, "pitch": 1.57, "yaw": 0.0,
                "step": 0.1, "name_file": "Rastrelliera"
            },
             {"use_sim_time": True},
        ],
    )

    # reachability_node_bimby_right = Node(
    #     package="nodes",
    #     executable="Reach",
    #     name="reachability_Bimby_right",  
    #     output="screen",
    #     parameters=[
    #         robot_description,
    #         moveit_config.robot_description_semantic,
    #         moveit_config.robot_description_kinematics,
    #         {
    #             "group_name": "right_",
    #             "xmin": -1.8, "xmax": -1.0,
    #             "ymin": -1.7,  "ymax": -0.9,
    #             "zmin": 0.65,   "zmax": 1.0,
    #             "roll": 1.57, "pitch": 1.57, "yaw": -0.78,
    #             "step": 0.1,
    #         },
    #     ],
    # )

    reachability_node_Bimby_left = Node(
        package="nodes",
        executable="Reach",
        name="reachability_Bimby_left",  
        output="screen",
        parameters=[
            robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {
                "group_name": "left_",
                "xmin": -1.55, "xmax": -1.0,
                "ymin": 0.20,  "ymax": 1.0,
                "zmin": 0.65,   "zmax": 1.0,
                "roll": 0.0, "pitch": 1.57, "yaw": 3.14,
                "step": 0.1, "name_file": "Bimby_left"
            },
             {"use_sim_time": True},
        ],
    )

    reachability_node_Bimby_center = Node(
        package="nodes",
        executable="Reach",
        name="reachability_Bimby_center",  
        output="screen",
        parameters=[
            robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {
                "group_name": "left_",
                "xmin": -1.45, "xmax": -1.1,
                "ymin": -0.70,  "ymax": 0.10,
                "zmin": 0.75,   "zmax": 1.0,
                "roll": 0.0, "pitch": 1.57, "yaw": -1.57,
                "step": 0.1, "name_file": "Bimby_center"
            },
                {"use_sim_time": True},
        ],
    )



    return LaunchDescription([
        #reachability_node_tavolo,
        reachability_node_rastrelliera,
        # reachability_node_bimby_right,
        #reachability_node_Bimby_left,
        reachability_node_Bimby_center,
        #reachability_node_Bimby_left,
       # reachability_node_Bimby_center,
    ])