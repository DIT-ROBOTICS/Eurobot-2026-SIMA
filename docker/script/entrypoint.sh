#!/bin/bash
sudo chmod 666 /dev/ttyAMA0
cd /home/user/SIMA-ws
source install/setup.bash
ros2 launch sima-main real_launch.py