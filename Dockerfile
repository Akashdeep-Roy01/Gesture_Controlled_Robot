# Use ROS2 Humble base image
FROM ros:humble-ros-core-jammy

# Define arguments for non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME && usermod -aG video $USERNAME && rm -rf /var/lib/apt/lists/*

# Install common tools and ROS2 dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    wget \
    bash-completion \
    build-essential \
    cmake \
    gdb \
    git \
    openssh-client \
    python3-argcomplete \
    python3-pip \
    python3-vcstool \
    python3-rosdep \
    python3-colcon-common-extensions \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-robot-state-publisher \
    ros-humble-moveit \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    numpy \
    opencv-python \
    mediapipe

# Configure ROS2 environment
ENV ROS_DISTRO=humble
ENV AMENT_PREFIX_PATH=/opt/ros/humble \
    COLCON_PREFIX_PATH=/opt/ros/humble \
    LD_LIBRARY_PATH=/opt/ros/humble/lib \
    PATH=/opt/ros/humble/bin:$PATH \
    PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages \
    ROS_PYTHON_VERSION=3 \
    ROS_VERSION=2

# Set up .bashrc for non-root user
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc \
    && chown $USERNAME:$USERNAME /home/$USERNAME/.bashrc

# Create and set permissions for workspace
RUN mkdir -p /docker_ws/src && chown -R $USERNAME:$USERNAME /docker_ws

# Switch to non-root user
USER $USERNAME
WORKDIR /docker_ws

# Default command
CMD ["bash"]