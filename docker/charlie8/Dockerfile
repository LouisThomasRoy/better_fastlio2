# ---------------------------------------------------------------------------
# better_fastlio2 + GNSS -- build/run environment for the Charlie8 dataset.
#
# This image is an ENVIRONMENT ONLY. It deliberately does not contain the SLAM
# source code: you clone that yourself onto the host and bind-mount it at
# /catkin_ws, so you can edit it with your normal editor and rebuild inside the
# container. That split is the whole point -- the image is the toolchain, your
# workspace is your work.
#
#   Build (from the repository root):
#     docker build -t bfl2-gnss:noetic \
#         --build-arg UID=$(id -u) --build-arg GID=$(id -g) \
#         docker/charlie8
#
# The GTSAM step compiles from source and takes 15-30 minutes on a laptop. It
# is cached: you pay it once.
#
# Upstream's own docker/Dockerfile (an nvidia/cuda base) is left untouched next
# to this one. We use plain Ubuntu 20.04 because nothing in the mapping path
# needs CUDA, and this runs on any laptop.
# ---------------------------------------------------------------------------
FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# GeographicLib and python3-dev are easy to miss: CMakeLists.txt calls
# find_package(GeographicLib REQUIRED) and find_package(PythonLibs REQUIRED)
# (the latter for the vendored matplotlibcpp.h), but neither appears in
# package.xml, so rosdep will not pull them in.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git wget curl \
        libeigen3-dev libboost-all-dev libomp-dev \
        libpcl-dev libgoogle-glog-dev libgflags-dev \
        libtbb-dev libmetis-dev libgeographic-dev \
        python3-dev python3-pip python3-catkin-tools \
        ros-noetic-pcl-ros ros-noetic-tf ros-noetic-tf2-ros \
        ros-noetic-cv-bridge ros-noetic-image-transport \
        ros-noetic-eigen-conversions \
    && rm -rf /var/lib/apt/lists/*

# --- GTSAM 4.0.3: the version better_fastlio2's backend is written against.
# Newer 4.1+ changes the iSAM2 API in ways the repo does not track.
RUN git clone --depth 1 --branch 4.0.3 https://github.com/borglab/gtsam.git /tmp/gtsam && \
    cmake -S /tmp/gtsam -B /tmp/gtsam/build \
        -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
        -DCMAKE_BUILD_TYPE=Release && \
    cmake --build /tmp/gtsam/build -j"$(nproc)" --target install && \
    ldconfig && rm -rf /tmp/gtsam

# NOTE: Sophus is NOT installed here. better_fastlio2 vendors a header-only copy
# in include/sophus and never calls find_package(Sophus). Installing upstream
# Sophus fails anyway -- current HEAD requires CMake >= 3.24 and Ubuntu 20.04
# ships 3.16.

# --- Underlay workspace: the two message packages fast_lio_sam needs but does
# not ship. Built into the image so your own workspace contains exactly one
# package -- the SLAM pipeline -- and nothing else.
#
# livox_ros_driver is an unconditional include in preprocess.h even when the
# Livox path is unused. darknet_ros_msgs is listed as a catkin component in
# CMakeLists.txt (for an optional YOLO path that camera_en gates off) but is
# absent from package.xml; only the message package is needed, and cloning
# without --recursive skips the CUDA-heavy darknet submodule.
RUN mkdir -p /opt/deps_ws/src && cd /opt/deps_ws/src && \
    git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver.git && \
    git clone --depth 1 https://github.com/leggedrobotics/darknet_ros.git /tmp/darknet_ros && \
    cp -r /tmp/darknet_ros/darknet_ros_msgs . && \
    rm -rf /tmp/darknet_ros && \
    source /opt/ros/noetic/setup.bash && \
    cd /opt/deps_ws && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build -j"$(nproc)" && \
    chmod -R a+rX /opt/deps_ws

# numpy for tools/anchor/keyposes_to_tum.py (transformations.pcd -> TUM) and
# tools/eval/validate.py. Nothing here needs pip.
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-numpy sudo less nano tmux && \
    rm -rf /var/lib/apt/lists/*

# --- A non-root user whose UID matches yours on the host. Without this, every
# file catkin writes into the bind-mounted workspace is owned by root and you
# need sudo to delete your own build directory.
ARG UID=1000
ARG GID=1000
ARG USERNAME=slam
RUN if ! getent group "$GID" >/dev/null; then groupadd -g "$GID" "$USERNAME"; fi && \
    if ! getent passwd "$UID" >/dev/null; then \
        useradd -m -u "$UID" -g "$GID" -s /bin/bash "$USERNAME"; \
    fi && \
    USER_NAME="$(getent passwd "$UID" | cut -d: -f1)" && \
    echo "$USER_NAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/99-slam && \
    chmod 0440 /etc/sudoers.d/99-slam && \
    mkdir -p /catkin_ws/src /output && \
    chown -R "$UID:$GID" /catkin_ws /output

# ROS needs to advertise under a name the container can resolve. localhost is
# always right here: roscore, the mapping node and rosbag play all live in this
# one container.
ENV ROS_MASTER_URI=http://localhost:11311 \
    ROS_HOSTNAME=localhost \
    QT_X11_NO_MITSHM=1

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER $UID:$GID
WORKDIR /catkin_ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
