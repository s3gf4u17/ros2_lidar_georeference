DRIVERS := velodyne

build-all: clean-install $(DRIVERS:%=build-driver-%)

push-all: $(DRIVERS:%=push-driver-%)

clean-install:
	sudo rm -rf $(DRIVERS:%=./driver/%/install)

build-driver-%:
	sudo docker build --platform linux/arm64 --tag driver-$*:builder driver/$*
	container_id=$$(sudo docker create driver-$*:builder) && \
	sudo docker cp $$container_id:/ws/install/ ./driver/$*/install && \
	sudo docker rm $$container_id

push-driver-%:
	scp -r ./driver/$*/install pi@10.0.0.1:/home/pi/driver-$*

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