from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnShutdown, OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

setup_script="""
pkg_share=$(ros2 pkg prefix --share ros2_lidar_georeference)
dest=/opt/leo_ui

sudo cp "$pkg_share/web/ros2_lidar_georeference.html" "$dest/"
sudo cp "$pkg_share/web/css/ros2_lidar_georeference.css" "$dest/css/"
sudo cp "$pkg_share/web/js/ros2_lidar_georeference.js" "$dest/js/"
"""

cleanup_script="""
unset ROS2_LIDAR_GEOREFERENCE_DATA

dest=/opt/leo_ui

sudo rm "$dest/ros2_lidar_georeference.html"
sudo rm "$dest/css/ros2_lidar_georeference.css"
sudo rm "$dest/js/ros2_lidar_georeference.js"
"""

def generate_launch_description():
    pkg_share=get_package_share_directory("ros2_lidar_georeference")
    setup_script=os.path.join(pkg_share,"script","setup.sh")

    ros2_data = "/home/pi/ros2_lidar_georeference_data/"
    os.makedirs(os.path.join(ros2_data, "config"), exist_ok=True)
    os.makedirs(os.path.join(ros2_data, "raw"), exist_ok=True)
    os.makedirs(os.path.join(ros2_data, "las"), exist_ok=True)

    setup = ExecuteProcess(
        cmd=["bash",setup_script],
        shell=False,
        output="screen",
    )

    # web_server = ExecuteProcess(
    #     cmd=["python3","-m","http.server","9000"],
    #     shell=False,
    #     output="screen",
    # )

    hello_world_node = Node(
        package="ros2_lidar_georeference",
        executable="hello_world",
        name="hello_world",
        output="screen",
    )

    cleanup = ExecuteProcess(
        cmd=["echo","cleaningup"],
        shell=False,
        output="screen",
    )

    after_setup = RegisterEventHandler(
        OnProcessExit(
            target_action=setup,
            on_exit=[hello_world_node]
        )
    )

    on_shutdown = RegisterEventHandler(
        OnShutdown(on_shutdown=[cleanup])
    )

    return LaunchDescription([setup,after_setup,on_shutdown])