# ros2_lidar_georeference

This package is developed for the [Leo Rover](https://www.leorover.tech/) platform, but can be adapted to other systems running ROS 2 Jazzy, Rosbridge, and Nginx.

On user request, the package records [Velodyne](https://github.com/ros-drivers/velodyne) LiDAR point clouds together with ECEF positioning data published by the [Fixposition](https://github.com/fixposition/fixposition_driver) driver, storing the data in a temporary workspace. When the measurement session is stopped, the collected data is fused offline into a single .las file ([LAS version 1.2](https://liblas.org/_static/files/specifications/asprs_las_format_v12.pdf)), which is then made available for user download.

| Leo Rover with installed Velodyne VLP16 and Fixpostion Vision-RTK 2 sensors |
|----------|
| ![Rover Image](image/IMG_1818-min.jpg) |







1. Build ROS2 drivers for `velodyne`, `fixposition` and the `ros2_lidar_georeference` package:

```bash
[RUN ON BUILD PC]
make build-all
```

2. Connect the Velodyne sensor to `eth0`. Configure the network:

```bash
[RUN ON ROVER]
sudo ip addr add 192.168.1.100/24 dev br0
sudo ip link set br0 up
```

3. if the dhcp is enabled, you can access the velodyne web ui directly from the browser. if the dhcp is disabled, you will need to set up ssh tunnelling via 10.0.0.1

4. check if sensor is sending data

```bash
sudo tcpdump -i eth0 -w test.pcap
du -h test.pcap
```

5. if the dhcp is enabled, the sensor will be assigned an IP in the 10.0.0.1/24 domain, velodyne config change is required (device_ip in velodyne_driver/config/VLP16-velodyne_driver_node-params.yaml)

6. launch velodyne node

```bash
source driver-velodyne/setup.bash
ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py
```

7. check if the node is reading the tcp data correctly and publishing it on the /velodyne_points topic

```bash
ros2 topic echo /velodyne_points
```



package is designed for leo rover, but can be ported to other machines that run rosbridge and nginx. just update 'dest' nginx directory in script/setup.sh