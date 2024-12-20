#!/bin/bash

SCRIPT_PATH=~/x3cator_robot/linux

echo "##################### ROS2 x3cator settings (Abdullah) #############" >> ~/.bashrc

echo "" >> ~/.bashrc
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
echo "" >> ~/.bashrc

echo "source $SCRIPT_PATH/ros_sc.sh" >> ~/.bashrc
echo "" >> ~/.bashrc

echo "source $SCRIPT_PATH/ros_ws.sh" >> ~/.bashrc
echo "" >> ~/.bashrc

source ~/.bashrc
echo "Added ROS Foxy setup, ros_sc.sh, ros_ws.sh, and ROS workspace config to .bashrc and sourced"