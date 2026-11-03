# ROS 2 Jazzy with what the dashboard server needs, for running and testing it on the laptop.
# See README.md.
FROM osrf/ros:jazzy-desktop
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get upgrade -y \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    python3-aiohttp \
    python3-cryptography \
    python3-pytest \
    ros-jazzy-rosbridge-server \
    && rm -rf /var/lib/apt/lists/*
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc
WORKDIR /home/jdutra/r5_ws/r5_dashboard
