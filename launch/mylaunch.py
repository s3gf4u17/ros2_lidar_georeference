from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node

def generate_launch_description():
    setup = ExecuteProcess(
        cmd=["sudo","ls"],
        shell=False,
        output="screen",
    )

    hello_world_node = Node(
        package="ros2_lidar_georeference",
        executable="hello_world",
        name="hello_world",
        output="screen",
    )

    cleanup = ExecuteProcess(
        cmd=["sudo","ls"],
        shell=False,
        output="screen",
    )

    on_shutdown = RegisterEventHandler(
        OnShutdown(on_shutdown=[cleanup])
    )

    return LaunchDescription([setup,hello_world_node,on_shutdown])