FROM osrf/ros:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive

WORKDIR /workspace

# Fix SGML issues
RUN mkdir -p /usr/share/sgml /usr/share/xml /etc/sgml

# Install ROS & tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    git build-essential \
    python3-colcon-common-extensions \
    ros-humble-xacro \
    ros-humble-nav2-bringup \
    ros-humble-tf2-tools \
    ros-humble-vision-opencv \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-robot-localization \
    ros-humble-velodyne \
    ros-humble-velodyne-gazebo-plugins \
    ros-humble-velodyne-description \
    curl unzip tmux && \
    rm -rf /var/lib/apt/lists/*

# Setup ROS environment
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

CMD ["bash"]
