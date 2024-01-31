from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    remap_number_topic = ("number","my_number")
    number_publisher_node = Node(
        package="my_py_pkg",
        executable="number_publisher",
        name="number_publisher",
        remappings=[ #Renomeando os topicos
           remap_number_topic 
        ],
        parameters=[
            {"published_number":12},
            {"publish_frequency":15.0}
        ]
    )

    number_counter_node = Node(
        package="my_py_pkg",
        executable="number_counter",
        name="number_counter",
        remappings=[ #Renomeando os topicos
            remap_number_topic,
            ("number_count","my_number_count")
        ]
    )

    ld.add_action(number_publisher_node)
    ld.add_action(number_counter_node)
    return ld