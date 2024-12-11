#!/bin/bash

for srv in $(ros2 interface list | grep x3cator_msgs/srv/); do
    echo "Definition of $srv:"
    ros2 interface show $srv
    echo "--------------------------"
done
