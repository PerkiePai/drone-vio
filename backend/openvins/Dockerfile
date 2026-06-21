# OpenVINS (ROS1 Noetic) built into a catkin workspace.
# CPU-only visual-inertial odometry. Build context = backend/ (expects ./open_vins).
#   docker build -t openvins:noetic -f backend/Dockerfile backend
# Run with the dataset + config mounted (see backend/run_openvins.sh).
FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# Build tooling + OpenVINS deps (Ceres from apt, Eigen, catkin tools).
RUN apt-get update && apt-get install -y --no-install-recommends \
        git nano \
        python3-catkin-tools python3-osrf-pycommon \
        cmake build-essential \
        libeigen3-dev \
        libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev libceres-dev \
        python3-dev python3-matplotlib python3-numpy python3-psutil python3-tk \
        ros-noetic-image-transport ros-noetic-compressed-image-transport ros-noetic-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

# Copy OpenVINS source into a catkin workspace and build it.
WORKDIR /catkin_ws
COPY open_vins/ /catkin_ws/src/open_vins/
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
        catkin init && \
        catkin config --extend /opt/ros/noetic && \
        catkin build ov_msckf ov_eval -j$(nproc) --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_ROS=ON"

# ffmpeg: ensures OpenCV VideoWriter can encode mp4 (+ CLI transcode fallback).
# Placed after the build so the catkin compile layer stays cached.
RUN apt-get update && apt-get install -y --no-install-recommends ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# Source ROS + workspace on every interactive shell.
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

WORKDIR /catkin_ws
CMD ["/bin/bash"]
