from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="axolotl",
            executable="axolotl",
            name="axolotl",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "fleet_name": "dolly",

                    # Vehicle traits
                    "linear_velocity":"",
                    "angular_velocity":"",
                    "linear_acceleration":"",
                    "angular_acceleration":"",
                    "footprint_radius":"",
                    "vicinity_radius":"",
                    "reversible":"",

                    "nav_graph_file":"",

                    "perform_loop":"",
                    
                    "perform_deliveries":"",
                    
                    "perform_cleaning":"",
                    
                    "disable_delay_threshold":"",
                    
                    "experimental_lift_watchdog_service":""
                
                }
            ]
        )
    ])
