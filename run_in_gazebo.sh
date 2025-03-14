#!/bin/bash

# 依次启动Gazebo仿真、VINS、Elastic-Tracker
gnome-terminal \
--window -e 'bash -c "roslaunch px4 mavros_posix_sitl.launch; exec bash" ' \
--tab -e 'bash -c "source devel/setup.bash; sleep 15 && roslaunch vins Drone_gazebo.launch; exec bash" ' \
--tab -e 'bash -c "source devel/setup.bash; sleep 20 && roslaunch planning run_in_gazebo.launch; exec bash" '
