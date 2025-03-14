#!/bin/bash

# 依次启动Realsense、VINS、Elastic-Tracker
gnome-terminal \
--window -e 'bash -c "source devel/setup.bash; roslaunch realsense2_camera rs_camera.launch ; exec bash" ' \
--tab -e 'bash -c "source devel/setup.bash; sleep 5 && roslaunch vins Drone_250.launch; exec bash" ' \
--tab -e 'bash -c "source devel/setup.bash; sleep 10 && roslaunch planning run_in_exp.launch; exec bash" '
