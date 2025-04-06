FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    && rm -rf /var/lib/apt/lists/*


RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros_ws/src

RUN git clone https://github.com/LILGRANDMASTER/MinoTracker.git .



