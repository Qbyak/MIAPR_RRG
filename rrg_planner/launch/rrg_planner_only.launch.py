from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    params_file = "/home/kacper/Dokumenty/Magisterka/ros2_ws/rrg_planner_only_params.yaml"
    map_file = "/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.yaml"

    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_base_link_tf",
            arguments=[
                "0", "0", "0",
                "0", "0", "0",
                "map", "base_link"
            ],
            output="screen"
        ),

        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[params_file]
        ),

        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[params_file]
        ),

        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_planner_only",
            output="screen",
            parameters=[params_file]
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen"
        ),

        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "service",
                        "call",
                        "/map_server/load_map",
                        "nav2_msgs/srv/LoadMap",
                        "{map_url: '" + map_file + "'}"
                    ],
                    output="screen"
                )
            ]
        ),

        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package="rrg_planner",
                    executable="rrg_auto_request",
                    name="rrg_auto_request",
                    output="screen"
                )
            ]
        ),
    ])