#!/bin/bash

pkill gz
pkill gzclient
pkill gzserver
pkill gazebo
pkill roslaunch
pkill roscore
pkill -f "xterm -T mavros"
pkill -f "xterm -T gazebo"
pkill -f "xterm -T gz"
pkill -f "../rcS"
pkill px4
pkill -f "px4-"
#docker stop $(docker ps -q -a --filter "ancestor=my_docker:tag")
