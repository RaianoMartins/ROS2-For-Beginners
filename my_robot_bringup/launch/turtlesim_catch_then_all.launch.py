from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    package_name = "turtlesim_catch_then_all"

    turtle_sim_node = Node(
        package = "turtlesim",
        executable = "turtlesim_node"
    )

    turtle_controller_node = Node(
        package="turtlesim_catch_then_all",
        executable="turtle_controller",
        parameters=[
            {"catch_closest_trutle_first": True},
        ]
    )

    turtle_spawner_node = Node(
        package = package_name,
        executable = "turtle_spawner",
        parameters = [
            {"spawn_frequency": 1.0},
            {"turtle_name_prefix": "minha_tartaruga"}
        ]
    )

    ld.add_action(turtle_sim_node)
    ld.add_action(turtle_controller_node)
    ld.add_action(turtle_spawner_node)

    return ld