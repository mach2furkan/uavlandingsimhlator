FROM ros:humble


# Install Gazebo Garden (or specific version compatible with ros_gz)
# Note: ROS 2 Humble usually pairs with Gazebo Fortress or Garden.
# We will use the standard ros_gz_sim packages available in apt.

RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-ros-gz \
    ros-humble-cv-bridge \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Python deps
RUN pip3 install setuptools==58.2.0

# Setup workspace
WORKDIR /ws
COPY . /ws/src/uav_landing_sim

# Build
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Source entrypoint
COPY resource/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

CMD ["ros2", "launch", "uav_landing_sim", "simulation.launch.py"]
