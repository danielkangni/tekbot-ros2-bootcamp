# ROS 2 Humble + Gazebo Classic, version minimale (pas de desktop complet)
FROM docker.io/library/ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-xacro \
        ros-humble-joint-state-publisher \
        ros-humble-rviz2 \
        ros-humble-rqt-gui \
        ros-humble-rqt-gui-py \
        ros-humble-joy \
        ros-humble-teleop-twist-joy \
        ros-humble-teleop-twist-keyboard \
        ros-humble-robot-localization \
        ros-humble-slam-toolbox \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        python3-colcon-common-extensions \
        python3-numpy \
        libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/* /usr/share/doc /usr/share/man

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
