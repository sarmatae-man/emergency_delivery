#!/bin/bash

source /opt/ros/noetic/setup.bash
source /home/$USER/catkin_ws/devel/setup.bash
source /home/$USER/PX4-Autopilot/Tools/setup_gazebo.bash /home/$USER/PX4-Autopilot /home/$USER/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=/opt/ros/noetic/share:/home/$USER/your_path/PX4-Autopilot:/home/$USER/your_path/PX4-Autopilot/Tools/sitl_gazebo
export GAZEBO_MODEL_PATH=/home/$USER/catkin_ws/src/emergency_delivery/uav_robot_model
