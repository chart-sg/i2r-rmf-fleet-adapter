from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="i2r-rmf-fleet-adapter",
            executable="i2r-rmf-fleet-adapter",
            name="i2r-rmf-fleet-adapter",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"fleet_name": "dolly"},
                ### Vehicle traits ###
                {"linear_velocity": 0.0},
                {"angular_velocity": 0.0},
                {"linear_acceleration": 0.0},
                {"angular_acceleration": 0.0},
                {"footprint_radius": 1.0},
                {"vicinity_radius": 1.0},
                {"reversible": True},
                ###############################
                {"nav_graph_file": \
                    "/home/jh/rmf_ws/install/rmf_demos_maps/share/rmf_demos_maps/maps/office/nav_graphs/0.yaml"},
                {"perform_loop": False},
                {"perform_deliveries": False},
                {"perform_cleaning": False},
                {"disable_delay_threshold": False},
                {"experimental_lift_watchdog_service": \
                    ""}
                ]
            )
    ])
