FROM nvidia/cuda:13.0.2-cudnn-runtime-ubuntu22.04
COPY --from=osrf/ros:humble-desktop-full-jammy / /
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO humble
WORKDIR /workspace

# Install ALL system and ROS dependencies first (Cacheable Layer)
# Combined the initial build tools  with the ROS packages from the end of the file 
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential gcc g++ gfortran git patch wget pkg-config liblapack-dev libmetis-dev \
    cmake curl apt-utils python3 python3-pip gnupg2 vim libsm6 libxext6 libxrender-dev \
    nano screen tmux htop openssh-client unzip \
    ros-humble-moveit \
    ros-humble-moveit-ros-perception \
    ros-humble-depth-image-proc \
    ros-humble-rtabmap-ros \
    ros-humble-octomap-server \
    ros-humble-octomap-ros \
    ros-humble-octomap-rviz-plugins \
    ros-humble-controller-manager \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    libhdf5-dev \
    && rm -rf /var/lib/apt/lists/* \
    && ln -s /usr/bin/python3 /usr/bin/python
RUN wget http://snapshots.ros.org/humble/2025-06-18/ubuntu/pool/main/r/ros-humble-rviz-common/ros-humble-rviz-common_11.2.17-1jammy.20250617.234657_amd64.deb && \
    apt-get update && \
    apt-get install -y --allow-downgrades ./ros-humble-rviz-common_11.2.17-1jammy.20250617.234657_amd64.deb && \
    rm ros-humble-rviz-common_11.2.17-1jammy.20250617.234657_amd64.deb && \
    rm -rf /var/lib/apt/lists/*

# Build Micro XRCE-DDS Agent (Stable Layer)
# Moved up so it doesn't rebuild when PX4 code changes
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig /usr/local/lib/

# Clone and Setup PX4 (Volatile Layer-Keep at the end)
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive
RUN bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
# Swap GZ versions
RUN apt-get update && \
    apt purge -y ros-humble-ros-gz-sim && \
    apt install -y ros-humble-ros-gzharmonic ros-humble-ros-gzharmonic-sim ros-humble-ros-gzharmonic-bridge && \
    rm -rf /var/lib/apt/lists/*
WORKDIR /workspace/PX4-Autopilot

COPY drone_ws/src/gazebo_worlds/worlds/warehouse.sdf \
    /workspace/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf
RUN make px4_sitl_default

WORKDIR /workspace/drone_ws/src
RUN git clone https://github.com/PX4/px4_msgs.git 
COPY drone_ws/src /workspace/drone_ws/src
WORKDIR /workspace/drone_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --parallel-workers $(nproc)"

COPY scripts/startup.sh /workspace/scripts/startup.sh

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
