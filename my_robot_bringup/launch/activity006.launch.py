from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    robot_news_station_giskard = Node(
        package="my_py_pkg",
        executable="robot_news_station",
        name="robot_news_station_giskard"
    )

    robot_news_station_bb8 = Node(
        package="my_py_pkg",
        executable="robot_news_station",
        name="robot_news_station_bb8"
    )

    robot_news_station_daneel = Node(
        package="my_py_pkg",
        executable="robot_news_station",
        name="robot_news_station_daneel"
    )

    robot_news_station_jander= Node(
        package="my_py_pkg",
        executable="robot_news_station",
        name="robot_news_station_jander"
    )

    robot_news_station_c3po = Node(
        package="my_py_pkg",
        executable="robot_news_station",
        name="robot_news_station_c3po"
    )

    smartphone = Node(
        package="my_py_pkg",
        executable="smartphone"
    )

    ld.add_action(robot_news_station_giskard)
    ld.add_action(robot_news_station_bb8)
    ld.add_action(robot_news_station_daneel)
    ld.add_action(robot_news_station_jander)
    ld.add_action(robot_news_station_c3po)
    ld.add_action(smartphone)
    
    return ld