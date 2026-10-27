# Onboard stack. UNVERIFIED skeleton: pin every ARG during M0.
ARG L4T_BASE=nvcr.io/nvidia/l4t-jetpack:r36.4.0   # TODO M0: match flashed JetPack
FROM ${L4T_BASE}

ARG ROS_DISTRO=humble
ENV ROS_DISTRO=${ROS_DISTRO} DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8

# TODO M0: install ROS 2 (apt from packages.ros.org), python3-vcstool, python3-colcon-common-extensions, rosdep
# TODO M0: install pinned ZED SDK for this L4T release (silent installer, skip drivers)

WORKDIR /ws
COPY . /ws
# RUN . /opt/ros/${ROS_DISTRO}/setup.sh && scripts/bootstrap.sh
