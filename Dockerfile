FROM arm64v8/ros:jazzy-ros-base

WORKDIR /ws/src

RUN apt update && apt install -y \
    git \
    ros-jazzy-velodyne \
    ros-jazzy-ros-environment \
    libyaml-cpp-dev \
    libboost-all-dev \
    zlib1g-dev \
    libeigen3-dev \
    linux-libc-dev \
    nlohmann-json3-dev

RUN git clone --recurse-submodules -j8 https://github.com/ros-drivers/velodyne
RUN git clone --recurse-submodules -j8 https://github.com/fixposition/fixposition_driver
COPY . ros2_lidar_georeference/

WORKDIR /ws/src/fixposition_driver
RUN ./setup_ros_ws.sh

WORKDIR /ws

RUN . /opt/ros/jazzy/setup.sh && colcon build --cmake-args -DBUILD_TESTING=OFF