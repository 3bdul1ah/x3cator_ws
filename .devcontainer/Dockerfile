FROM osrf/ros:humble-desktop-full

# Environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    WORKSPACE_DIR=/workspace \
    USERNAME=ubuntu \
    USER_UID=1000 \
    USER_GID=1000 \
    ROS_DOMAIN_ID=0 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    CAN_BITRATE=1000000 \
    CAN_INTERFACE=can0 \
    PYTHON_VERSION=3.10 \
    QT_X11_NO_MITSHM=1

# Create user
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-vcstool \
    python3-colcon-common-extensions \
    vim \
    nano \
    tmux \
    can-utils \
    usbutils \
    network-manager \
    net-tools \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 packages
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-msgs \
    ros-${ROS_DISTRO}-nav2-planner \
    ros-${ROS_DISTRO}-nav2-controller \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-gazebo-ros \
    ros-${ROS_DISTRO}-gazebo-ros2-control \
    ros-${ROS_DISTRO}-gazebo-plugins \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-position-controllers \
    ros-${ROS_DISTRO}-velocity-controllers \
    ros-${ROS_DISTRO}-effort-controllers \
    ros-${ROS_DISTRO}-diff-drive-controller \
    ros-${ROS_DISTRO}-imu-tools \
    ros-${ROS_DISTRO}-diagnostic-updater \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install \
    transforms3d \
    pyserial \
    python-can \
    canopen \
    numpy \
    scipy \
    matplotlib \
    opencv-python

# Setup CAN interface
RUN echo "allow-hotplug ${CAN_INTERFACE}" > /etc/network/interfaces.d/${CAN_INTERFACE} && \
    echo "iface ${CAN_INTERFACE} inet manual" >> /etc/network/interfaces.d/${CAN_INTERFACE} && \
    echo "    pre-up /sbin/ip link set \$IFACE type can bitrate ${CAN_BITRATE}" >> /etc/network/interfaces.d/${CAN_INTERFACE} && \
    echo "    up /sbin/ifconfig \$IFACE up" >> /etc/network/interfaces.d/${CAN_INTERFACE} && \
    echo "    down /sbin/ifconfig \$IFACE down" >> /etc/network/interfaces.d/${CAN_INTERFACE}

RUN usermod --append --groups dialout,plugdev,sudo,video $USERNAME

USER $USERNAME
WORKDIR $WORKSPACE_DIR

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc && \
    echo "source $WORKSPACE_DIR/install/setup.bash 2>/dev/null" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=$ROS_DOMAIN_ID" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION" >> ~/.bashrc && \
    echo 'setup_can() {' >> ~/.bashrc && \
    echo "    sudo ip link set ${CAN_INTERFACE} type can bitrate ${CAN_BITRATE}" >> ~/.bashrc && \
    echo "    sudo ip link set up ${CAN_INTERFACE}" >> ~/.bashrc && \
    echo '}' >> ~/.bashrc && \
    echo 'alias cb="cd ${WORKSPACE_DIR} && colcon build --symlink-install"' >> ~/.bashrc

CMD ["/bin/bash"]