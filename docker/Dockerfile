# ROS 2 Jazzy for the laptop: RViz and tools, plus what's needed to build and test r5.
# See doc/development.md.
FROM osrf/ros:jazzy-desktop
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    dnsutils \
    iperf3 \
    iproute2 \
    iputils-ping \
    liburdfdom-tools \
    netcat-openbsd \
    openssh-client \
    tcpdump \
    traceroute \
    ros-jazzy-ament-cmake-pytest \
    ros-jazzy-ament-cmake-ros \
    ros-jazzy-backward-ros \
    ros-jazzy-controller-manager \
    ros-jazzy-diff-drive-controller \
    ros-jazzy-hardware-interface \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-launch-testing-ament-cmake \
    ros-jazzy-nav2-bt-navigator \
    ros-jazzy-nav2-controller \
    ros-jazzy-nav2-lifecycle-manager \
    ros-jazzy-nav2-navfn-planner \
    ros-jazzy-nav2-planner \
    ros-jazzy-nav2-regulated-pure-pursuit-controller \
    ros-jazzy-nav2-simple-commander \
    ros-jazzy-nav2-velocity-smoother \
    ros-jazzy-nav2-waypoint-follower \
    ros-jazzy-ros2-control-cmake \
    ros-jazzy-ros2controlcli \
    ros-jazzy-robot-localization \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc
WORKDIR /home/jdutra/git/r5
