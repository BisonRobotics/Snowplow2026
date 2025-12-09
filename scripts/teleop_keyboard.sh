#!/bin/bash
# Script to run teleop_twist_keyboard with remapped topic for keyboard teleop

source /opt/ros/jazzy/setup.bash
# Assuming the workspace is sourced elsewhere, or add source /path/to/workspace/install/setup.bash if needed

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/keyboard_cmd_vel