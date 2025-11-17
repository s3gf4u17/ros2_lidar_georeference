FROM ubuntu:24.04

ENV LANG=C.UTF-8

RUN apt update && apt install -y \
    software-properties-common && \
    add-apt-repository universe && \
    apt update && apt install -y \
    curl

ENV ROS_APT_SOURCE_VERSION=1.1.0

RUN curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"

RUN dpkg -i /tmp/ros2-apt-source.deb

RUN apt update && apt upgrade && apt install -y ros-jazzy-desktop

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

RUN apt install python3-colcon-common-extensions -y

RUN mkdir -p /root/ros2ws/src

RUN echo "cd /root/ros2ws" >> /root/.bashrc

RUN apt install git -y

ARG CACHE_BUST=5
RUN git clone https://github.com/s3gf4u17/ros2_lidar_georeference /root/ros2ws/src/ros2_lidar_georeference -b master