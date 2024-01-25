# Используем образ ROS Noetic как базовый
FROM osrf/ros:noetic-desktop-full

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# user 
ENV USER="aviator"
# Добавляем пользователя USER в sudo
RUN useradd -ms /bin/bash $USER && echo "$USER:1" | chpasswd && adduser $USER sudo
RUN usermod -aG sudo $USER
RUN echo "${USER} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# OpenGL
RUN apt-get update && apt-get install -y \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    libgl1-mesa-dev

# Устанавливем git, редактор nano, wget, ssh
RUN apt-get update
RUN apt-get upgrade -y
RUN apt-get install -y git nano wget tmux openssh-server
# Устанавливаем catkin
RUN apt-get install python3-catkin-tools -y

# Устанавливаем mavros
RUN apt-get install ros-noetic-mavros -y
RUN apt-get install ros-noetic-mavros-extras -y
# Провереям в установке, возможны ошибки при отказе из-за IP адреса
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
RUN bash ./install_geographiclib_datasets.sh -y
RUN rm install_geographiclib_datasets.sh

USER $USER
WORKDIR /home/$USER
# Устанавливаем PX4-Autopilot с Gazebo
RUN git clone https://github.com/PX4/PX4-Autopilot.git
WORKDIR /home/$USER/PX4-Autopilot
RUN git checkout v1.13.3
RUN sudo /bin/bash Tools/setup/ubuntu.sh
RUN sudo DONT_RUN=1 make px4_sitl_default gazebo -j$(nproc)

# копируем базовую модель для собираемой uav_ex_dev
WORKDIR /home/$USER/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes
RUN sudo cp 10016_iris 10016_uav_ex_dev
RUN sudo chmod 777 10016_uav_ex_dev
RUN echo "param set COM_RCL_EXCEPT 4" >> 10016_uav_ex_dev

RUN echo "source /opt/ros/noetic/setup.bash" >> /home/$USER/.bashrc
RUN echo "source /home/$USER/PX4-Autopilot/Tools/setup_gazebo.bash /home/$USER/PX4-Autopilot /home/$USER/PX4-Autopilot/build/px4_sitl_default" >> /home/$USER/.bashrc
RUN echo "export ROS_PACKAGE_PATH=/opt/ros/noetic/share:/home/$USER/PX4-Autopilot:/home/$USER/PX4-Autopilot/Tools/sitl_gazebo" >> /home/$USER/.bashrc
RUN echo "export GAZEBO_MODEL_PATH=/home/$USER/catkin_ws/src/emergency_delivery/uav_robot_model" >> /home/$USER/.bashrc

# Инициализируем рабочее пространство
RUN mkdir -p /home/$USER/catkin_ws/src
WORKDIR /home/$USER/catkin_ws/src

# Клонируем репозитарий с моделью и оборудованием, даём права и собираем пакет
RUN git clone https://github.com/sarmatae-man/emergency_delivery.git
RUN sudo chmod -R +x /home/$USER/catkin_ws/src/emergency_delivery
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_init_workspace"
WORKDIR /home/$USER/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin build"
RUN echo "source /home/$USER/catkin_ws/devel/setup.bash" >> /home/$USER/.bashr
# Копируем скрипты для запуска, для удобства в корень user-a
RUN cp /home/$USER/catkin_ws/src/emergency_delivery/scripts/start_PX4_Gazebo.sh /home/$USER/catkin_ws
RUN cp /home/$USER/catkin_ws/src/emergency_delivery/scripts/start_monitoring.sh /home/$USER/catkin_ws

ENTRYPOINT sudo service ssh start && top -b
