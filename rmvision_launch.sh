#!/bin/bash
cd /home/wdr/hero_autoaim
source install/setup.bash
ros2 launch rm_vision_bringup vision_bringup.launch.py
