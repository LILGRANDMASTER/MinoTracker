FROM ros:jazzy-ros-base

# install working tools
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    nano \
    && rm -rf /var/lib/apt/lists/*


# Install ros packages
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

# Install WiringPi
RUN git clone https://github.com/WiringPi/WiringPi.git && \
    cd /WiringPi/wiringPi && \
    make all && make install

WORKDIR /ros_ws/src

RUN git clone https://github.com/LILGRANDMASTER/MinoTracker.git .



