#!/bin/bash

for msg in $(ros2 interface list | grep x3cator_msgs/msg/); do
    echo "Definition of $msg:"
    ros2 interface show $msg
    echo "--------------------------"
done
