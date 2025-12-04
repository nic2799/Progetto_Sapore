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

    # Nodo Reachability per la pedana
    reachability_node_tavolo = Node(
        package="nodes",
        executable="Raggiungibilita",
        name="reachability_tavolo", 
        output="screen",
        parameters=[
            robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
          
        ],
    )

    # Nodo Reachability per il tavolo
   



    return LaunchDescription([
        reachability_node_tavolo,
        #reachability_node_rastrelliera,
    ])