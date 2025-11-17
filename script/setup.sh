#!/bin/bash

# create folder for package data
mkdir -p /home/pi/ros2_lidar_georeference/{config,raw,las}

# inject web files into existing nginx setup
pkg_share=$(ros2 pkg prefix --share ros2_lidar_georeference)
dest=/opt/leo_ui

sudo cp "$pkg_share/web/ros2_lidar_georeference.html" "$dest/"
sudo cp "$pkg_share/web/css/ros2_lidar_georeference.css" "$dest/css/"
sudo cp "$pkg_share/web/js/ros2_lidar_georeference.js" "$dest/js/"