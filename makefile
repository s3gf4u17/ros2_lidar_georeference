default2:
	ros2 launch leo_bringup leo_bringup.launch.xml # in /opt/ros/jazzy/share

run:
	docker run --rm -it -p 9000:9000 -e DISPLAY=host.docker.internal:0 -e QT_X11_NO_MITSHM=1 ros2image

build:
	docker build -t ros2image .

build-pkg:
	colcon build --packages-select ros2_lidar_georeference

run-pkg:
	# Edit sudoers with visudo and add a line granting the ROS user NOPASSWD for the specific script(s):
	# rosuser ALL=(root) NOPASSWD: /usr/local/bin/setup_hardware.sh, /usr/local/bin/cleanup_hardware.sh
	ros2 launch ros2_lidar_georeference mylaunch.py

scp:
	scp -r ros2_lidar_georeference/* pi@leo-lis:/home/pi/src/ros2_lidar_georeference/