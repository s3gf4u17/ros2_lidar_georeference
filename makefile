default2:
	ros2 launch leo_bringup leo_bringup.launch.xml # in /opt/ros/jazzy/share

run:
	docker run --rm -it -p 9000:9000 -e DISPLAY=host.docker.internal:0 -e QT_X11_NO_MITSHM=1 ros2image

build:
	docker build -t ros2image .

build-pkg:
	colcon build --packages-select my_package