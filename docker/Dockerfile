FROM nvidia/cuda:12.6.0-devel-ubuntu24.04 AS base

ENV DEBIAN_FRONTEND=noninteractive

# ROS2 Jazzy
RUN apt-get update && apt-get install -y curl gnupg && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu noble main" \
        > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y \
        ros-jazzy-ros-base \
        ros-jazzy-rosbag2-cpp \
        ros-jazzy-sensor-msgs \
        ros-jazzy-tf2 \
        ros-jazzy-tf2-msgs \
        ros-jazzy-pcl-conversions \
        ros-jazzy-pcl-ros \
        ros-jazzy-cv-bridge \
        ros-jazzy-image-transport \
        ros-jazzy-nav-msgs \
        ros-jazzy-std-msgs \
        ros-jazzy-rosbag2-transport \
        ros-jazzy-rosbag2-storage \
        ros-jazzy-rcl-interfaces \
        python3-vcstool \
        python3-colcon-common-extensions

# system deps
RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    libopencv-dev \
    libsuitesparse-dev \
    libglfw3-dev \
    libgl-dev \
    freeglut3-dev \
    libx11-dev \
    libxrandr-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxi-dev \
    liblz4-dev \
    libzstd-dev \
    libqglviewer-dev-qt5 \
    libgl1-mesa-dri \
    mesa-utils \
    cmake \
    build-essential \
    git \
    time \
    gnuplot \
    emacs

# -------------------------------------------------------------------
FROM base AS srrg

COPY docker/repos.yml /tmp/repos.yml
RUN mkdir -p /ws/src && cd /ws/src && vcs import < /tmp/repos.yml
WORKDIR /ws
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --cmake-args -DEIGEN3_INCLUDE_DIR=/usr/include/eigen3 && \
    rm -rf /ws/src /ws/build /tmp/repos.yml

ARG BLAST_CACHE=1
# -------------------------------------------------------------------
FROM srrg AS cpu

RUN mkdir -p /ws/src && cd /ws/src && \
    git clone https://github.com/rvp-group/kd_slam2.git
WORKDIR /ws
RUN . /opt/ros/jazzy/setup.sh && \
    . /ws/install/local_setup.sh && \
    colcon build \
        --cmake-args -DHAVE_CUDA=OFF -DEIGEN3_INCLUDE_DIR=/usr/include/eigen3
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc && \
    echo "source /ws/src/kd_slam2/scripts/kd_slam_setup.bash" >> /etc/bash.bashrc
WORKDIR /

# -------------------------------------------------------------------
FROM srrg AS cuda

ENV CPATH=/usr/local/cuda/include
ENV LIBRARY_PATH=/usr/local/cuda/lib64

RUN mkdir -p /ws/src && cd /ws/src && \
    git clone https://github.com/rvp-group/kd_slam2.git
WORKDIR /ws
RUN . /opt/ros/jazzy/setup.sh && \
    . /ws/install/local_setup.sh && \
    colcon build \
        --cmake-args -DHAVE_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=all-major \
            -DEIGEN3_INCLUDE_DIR=/usr/include/eigen3
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc && \
    echo "source /ws/src/kd_slam2/scripts/kd_slam_setup.bash" >> /etc/bash.bashrc
WORKDIR /
