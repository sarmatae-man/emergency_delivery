#!/bin/bash

# Setup environment
PX4_DIR="/home/$USER/PX4-Autopilot"
MODEL_FILES="/home/$USER/catkin_ws/src/emergency_delivery/uav_robot_model"
MODEL="uav_ex_dev"

roslaunch $PX4_DIR/launch/mavros_posix_sitl.launch \
          vehicle:=$MODEL \
          sdf:=$MODEL_FILES/$MODEL/$MODEL.sdf \
          world:=$MODEL_FILESR/worlds/empty.world
