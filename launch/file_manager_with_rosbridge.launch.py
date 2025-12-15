from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    AnyLaunchDescriptionSource,
)
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory


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

    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("velodyne"),
                "launch",
                "velodyne-all-nodes-VLP16-launch.py"
            )
        )
    )

    fixposition_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("fixposition_driver_ros2"),
                "launch",
                "node.launch"
            )
        )
    )

    measurement_collector = Node(
        package="ros2_lidar_georeference",
        executable="measurement_collector",   # <-- your node executable name
        name="measurement_collector",
        output="screen",
        parameters=[{
            "skip": 5,
            "workers": 6
        }]
    )

    measurement_processor = Node(
        package="ros2_lidar_georeference",
        executable="measurement_processor",   # <-- your node executable name
        name="measurement_processor",
        output="screen"
    )

    measurement_manager = Node(
        package="ros2_lidar_georeference",
        executable="measurement_manager",   # <-- your node executable name
        name="measurement_manager",
        output="screen"
    )

    return LaunchDescription([
        rosbridge,
        file_manager,
        velodyne_launch,
        fixposition_launch,
        measurement_collector,
        measurement_processor,
        measurement_manager
    ])