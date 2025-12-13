from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    rosbridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        parameters=[{
            "port": 9091,
            "delay_between_messages": 0.0
        }]
    )

    file_manager = Node(
        package="ros2_lidar_georeference",
        executable="file_manager",   # <-- your node executable name
        name="file_manager",
        output="screen"
    )

    return LaunchDescription([
        rosbridge,
        file_manager
    ])