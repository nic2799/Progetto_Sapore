from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  
    return LaunchDescription([
        # Interactive marker per definire target pose
        # Node(
        #     package='reachability_package',
        #     executable='interactive_marker',
        #     name='interactive_marker',
        #     output='screen'
        # ),


        # Piaino di pick e place per il braccio sinistro
        Node(
            package='robot_skills',
            executable='pick_place_server',
            name='pick_place_server',
            output='screen',
            parameters=[{
                'plans_output_file'   : 'PickPouringLeft.yaml',
                    'hand'                : 'left_hand',
                'planning_group': 'left_arm',  
            },
             {"use_sim_time": True},

            ]
        ),
    ])
