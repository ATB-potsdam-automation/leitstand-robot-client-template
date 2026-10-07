# Build from the directory that holds this checkout and leitstand-robot-contract side by side.
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}
ARG ROS_DISTRO

RUN apt-get update \
    && apt-get install -y --no-install-recommends python3-venv ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# A venv, because pip may not modify Ubuntu's Python packages, with system site packages for rclpy.
RUN python3 -m venv --system-site-packages /opt/venv
ENV PATH=/opt/venv/bin:$PATH

WORKDIR /ws
COPY leitstand-robot-contract /contract
COPY leitstand-robot-client-template/leitstand_client src/leitstand_client
COPY leitstand-robot-client-template/leitstand_client_ros2 src/leitstand_client_ros2

RUN apt-get update \
    && rosdep update --include-eol-distros \
    && rosdep install --from-paths src --ignore-src -y \
    && rm -rf /var/lib/apt/lists/*
# ubuntu 22.04 pip is too old to build the contract's package metadata.
RUN pip install --no-cache-dir --upgrade pip \
    && pip install --no-cache-dir /contract -r src/leitstand_client/requirements.txt
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && python -m colcon build --symlink-install

COPY leitstand-robot-client-template/docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
# ros2 launch shuts its nodes down on SIGINT, not on SIGTERM, which docker stop sends.
STOPSIGNAL SIGINT
CMD ["ros2", "launch", "leitstand_client_ros2", "client.launch.py", "config:=/config/robot.yaml"]
