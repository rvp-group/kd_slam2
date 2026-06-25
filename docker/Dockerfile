FROM ubuntu:24.04

ARG CUDA=0

ENV DEBIAN_FRONTEND=noninteractive

# CUDA toolkit (only when CUDA=1)
RUN if [ "$CUDA" = "1" ]; then \
    apt-get update && apt-get install -y curl gnupg && \
    curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/3bf863cc.pub \
        | gpg --dearmor -o /usr/share/keyrings/cuda-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/cuda-archive-keyring.gpg] \
        https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/ /" \
        > /etc/apt/sources.list.d/cuda.list && \
    apt-get update && apt-get install -y cuda-toolkit-12-0; \
fi

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
    cmake \
    build-essential \
    git

# clone srrg public deps
WORKDIR /ws/src
COPY docker/repos.yml /ws/repos.yml
RUN vcs import < /ws/repos.yml

# build srrg deps first (cached layer -- independent of kd_slam2 source)
WORKDIR /ws
ARG CUDA
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build \
        --cmake-args -DHAVE_CUDA=${CUDA} -DEIGEN3_INCLUDE_DIR=/usr/include/eigen3 \
        --packages-select \
            srrg_cmake_modules \
            srrg2_core_msgs \
            srrg2_core \
            srrg2_core_ros \
            srrg2_qgl_viewport \
            srrg2_solver \
            srrg2_solver_extras

# clone kd_slam2 from public repo
RUN git clone https://github.com/rvp-group/kd_slam2.git /tmp/kd_slam && \
    mv /tmp/kd_slam/kd_slam2 /ws/src/kd_slam2 && \
    mv /tmp/kd_slam/kd_slam2_msgs /ws/src/kd_slam2_msgs && \
    rm -rf /tmp/kd_slam

# build kd_slam2
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build \
        --cmake-args -DHAVE_CUDA=${CUDA} -DEIGEN3_INCLUDE_DIR=/usr/include/eigen3 \
        --packages-select \
            kd_slam2_msgs \
            kd_slam2

RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc && \
    echo "source /ws/install/setup.bash" >> /etc/bash.bashrc

RUN apt-get update && apt-get install -y libgl1-mesa-dri mesa-utils

WORKDIR /
