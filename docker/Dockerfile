FROM nvidia/cuda:13.0.2-cudnn-devel-ubuntu22.04
COPY --from=osrf/ros:humble-desktop-full-jammy / /
ARG CUROBO_VERSION=v0.8.0
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=humble
ENV CUDA_HOME=/usr/local/cuda
ENV PATH=/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH}
ENV TORCH_CUDA_ARCH_LIST=8.9
WORKDIR /workspace

# Install ALL system and ROS dependencies first (Cacheable Layer)
# Combined the initial build tools  with the ROS packages from the end of the file 
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential gcc g++ gfortran git git-lfs patch wget pkg-config liblapack-dev libmetis-dev \
    cmake ninja-build curl apt-utils python3 python3-dev python3-pip python3-venv gnupg2 vim libsm6 libxext6 libxrender-dev \
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
WORKDIR /workspace
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git

WORKDIR /workspace/Micro-XRCE-DDS-Agent
RUN cmake -S . -B build && \
    cmake --build build --parallel $(nproc) && \
    cmake --install build && \
    ldconfig /usr/local/lib/

# Clone, set up, and build PX4.
WORKDIR /workspace
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive

WORKDIR /workspace/PX4-Autopilot
RUN bash Tools/setup/ubuntu.sh
# Swap GZ versions
RUN apt-get update && \
    apt purge -y ros-humble-ros-gz-sim && \
    apt install -y ros-humble-ros-gzharmonic ros-humble-ros-gzharmonic-sim ros-humble-ros-gzharmonic-bridge && \
    rm -rf /var/lib/apt/lists/*

COPY drone_ws/src/gazebo_worlds/worlds/warehouse.sdf \
    /workspace/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf
# Compile the PX4 SITL firmware without starting a simulator during the image
# build. startup.sh launches it at runtime with `make px4_sitl gz_x500_depth`.
RUN make px4_sitl

# Install cuRobo only after Micro XRCE-DDS and PX4 have been built.
WORKDIR /opt
RUN python3 -m pip install --no-cache-dir \
      wheel ninja h5py uv

RUN git lfs install && \
    git clone --branch "${CUROBO_VERSION}" --depth 1 https://github.com/NVlabs/curobo.git

WORKDIR /opt/curobo
RUN git lfs pull && \
    uv pip install --system --no-cache -e ".[cu13-torch]" && \
    python3 -c "import curobo, torch; print(f'cuRobo version: {curobo.__version__}'); print(f'PyTorch version: {torch.__version__}; CUDA version: {torch.version.cuda}')"

WORKDIR /workspace/drone_ws/src
RUN git clone https://github.com/PX4/px4_msgs.git 
COPY drone_ws/src /workspace/drone_ws/src
WORKDIR /workspace/drone_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --parallel-workers $(nproc)"

COPY scripts/startup.sh /workspace/scripts/startup.sh

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
