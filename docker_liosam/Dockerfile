# FROM nvidia/cuda:12.1.1-devel-ubuntu22.04
FROM nvidia/cuda:12.4.0-devel-ubuntu22.04

############ Install ROS2 (first half from ros:humble-ros-core-jammy) ############

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN set -eux; \
       key='C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654'; \
       export GNUPGHOME="$(mktemp -d)"; \
       gpg --batch --keyserver keyserver.ubuntu.com --recv-keys "$key"; \
       mkdir -p /usr/share/keyrings; \
       gpg --batch --export "$key" > /usr/share/keyrings/ros2-latest-archive-keyring.gpg; \
       gpgconf --kill all; \
       rm -rf "$GNUPGHOME"

# setup sources.list
RUN echo "deb [ signed-by=/usr/share/keyrings/ros2-latest-archive-keyring.gpg ] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list

# setup environment
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=humble

## NEW ##
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    build-essential \
    git \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# setup rosdep
RUN rosdep init && rosdep update --rosdistro $ROS_DISTRO

RUN pip install colcon-mixin colcon-metadata

# install colcon mixins
RUN colcon mixin add default \
    https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
    https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# install extras
RUN apt-get update && apt-get install -y \
    ros-humble-gtsam \
    ros-humble-perception-pcl \
    ros-humble-pcl-msgs \
    ros-humble-vision-opencv \
    ros-humble-xacro \
    ros-humble-microstrain-inertial-driver \
    ros-humble-microstrain-inertial-rqt \
    ros-humble-velodyne \
    vim nano tmux locales \
    && rm -rf /var/lib/apt/lists/*

# fix locale
RUN locale-gen en_US.UTF-8 && update-locale LANG=en_US.UTF-8

# setup entrypoint (copied from host)
COPY docker/ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

############ CUDA + Torch Setup ##############
ENV CUDA_HOME=/usr/local/cuda
ENV TORCH_CUDA_ARCH_LIST="6.0 6.1 6.2 7.0 7.2 7.5 8.0 8.6 8.9"

############ Livox SDK ################
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd Livox-SDK2 && mkdir build && cd build && \
    cmake .. && make -j$(nproc) && make install && cd ../..

############ Livox ROS2 Driver ################
RUN git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2 && \
    . /opt/ros/humble/setup.sh && \
    cd ws_livox/src/livox_ros_driver2 && ./build.sh humble && \
    echo "source /ws_livox/install/setup.bash" >> ~/.bashrc

############ Ouster SDK + Python Dependencies ############
RUN pip install numpy matplotlib opencv-python open3d ouster-sdk

ENV WS="/root/ros2_ws"
WORKDIR $WS/src

############ LIO-SAM Setup ############
RUN git clone -b ros2 --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git && \
    git clone https://github.com/TixiaoShan/LIO-SAM.git && \
    cd LIO-SAM && git checkout ros2 && git pull

# Add custom configs and launch files
ADD docker_liosam/liosam_ousterimu_params.yaml  LIO-SAM/config/ousterimu_params.yaml
ADD docker_liosam/liosam_sbgimu_params.yaml     LIO-SAM/config/sbgimu_params.yaml 
ADD docker_liosam/rviz2.rviz                    LIO-SAM/config/rviz2.rviz
ADD docker_liosam/run_ouster_imu.launch.py      LIO-SAM/launch/run_ouster_imu.launch.py
ADD docker_liosam/run_sbg_imu.launch.py         LIO-SAM/launch/run_sbg_imu.launch.py

############ LIORF Setup ############
# RUN apt update && apt-get install -y \
#     libgeographic-dev \
#     && rm -rf /var/lib/apt/lists/*

# Use bash shell to allow sourcing setup
SHELL ["/bin/bash", "-c"]

WORKDIR ${WS}
# Build workspace
RUN source /opt/ros/humble/setup.bash && \
colcon build --executor sequential --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
COPY ./liorf ./src/liorf

# Source workspace
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source ${WS}/install/setup.bash" >> ~/.bashrc