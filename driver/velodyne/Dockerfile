FROM arm64v8/ros:jazzy-ros-base

WORKDIR /ws/src

RUN apt update && apt install -y \
    git \
    ros-jazzy-velodyne \
    ros-jazzy-ros-environment

RUN git clone --recurse-submodules -j8 https://github.com/ros-drivers/velodyne

WORKDIR /ws

RUN . /opt/ros/jazzy/setup.sh && colcon build --cmake-args -DBUILD_TESTING=OFF