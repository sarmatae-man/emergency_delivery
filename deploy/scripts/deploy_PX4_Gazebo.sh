#!/bin/bash

cd /home/$USER/
git clone https://github.com/PX4/PX4-Autopilot.git && cd PX4-Autopilot
git checkout v1.13.3
sudo chmod +x Tools/setup/ubuntu.sh
./Tools/setup/ubuntu.sh
DONT_RUN=1 make px4_sitl_default gazebo -j$(nproc)
sudo cd  /home/$USER/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes
sudo cp 10016_iris 10016_uav_ex_dev
sudo echo "param set COM_RCL_EXCEPT 4" >> 10016_uav_ex_dev
cd /home/$USER/catkin_ws/src
git clone https://github.com/sarmatae-man/emergency_delivery.git
