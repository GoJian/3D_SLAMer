# Onboard stack. UNVERIFIED skeleton: pin every ARG during M0.
ARG L4T_BASE=TODO   # must match L4T R39.2.0 / JetPack 7.2 (decision 0001); verify an NGC base exists
FROM ${L4T_BASE}

ARG ROS_DISTRO=jazzy
ENV ROS_DISTRO=${ROS_DISTRO} DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8

# TODO M0: install ROS 2 (apt from packages.ros.org), python3-vcstool, python3-colcon-common-extensions, rosdep
# TODO M0: install pinned ZED SDK for this L4T release (silent installer, skip drivers)

WORKDIR /ws
COPY . /ws
# RUN . /opt/ros/${ROS_DISTRO}/setup.sh && scripts/bootstrap.sh
