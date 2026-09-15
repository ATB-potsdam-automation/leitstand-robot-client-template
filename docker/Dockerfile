# Build context is a workspace src/ directory holding BOTH checkouts side by side:
#   src/leitstand-robot-contract/            (at the tag this client speaks)
#   src/leitstand-robot-client-template/
# from that directory:
#   docker build -f leitstand-robot-client-template/docker/Dockerfile -t leitstand-client .
# The image's ROS distro must match the robot's, because message definitions must agree.
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}
ARG ROS_DISTRO

# Both RMWs are installed because the container must speak the same DDS vendor as the robot's
# Nav2; pick one at run time with RMW_IMPLEMENTATION.
RUN apt-get update \
 && apt-get install -y --no-install-recommends python3-venv python3-pip ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
 && rm -rf /var/lib/apt/lists/*

# A virtual environment that sees the ROS packages: the contract and the client's dependencies
# go there, Ubuntu's own Python stays untouched, and its pip is current on every base image.
# RUN python3 -m venv --system-site-packages /opt/venv
# ENV PATH=/opt/venv/bin:$PATH

WORKDIR /ws
COPY leitstand-robot-contract /contract
COPY leitstand-robot-client-template/leitstand_client src/leitstand_client
COPY leitstand-robot-client-template/leitstand_client_ros2 src/leitstand_client_ros2

# End-of-life distributions (iron since 2024-12) are skipped by rosdep unless asked for.
RUN apt-get update \
 && rosdep update --include-eol-distros \
 && rosdep install --from-paths src --ignore-src -y \
 && rm -rf /var/lib/apt/lists/*
# The venv's pip is upgraded first: the one Ubuntu 22.04 ships builds the contract's pyproject
# metadata as UNKNOWN with no modules.
RUN pip install --break-system-packages --no-cache-dir --upgrade pip \
 && pip install --break-system-packages --no-cache-dir /contract -r src/leitstand_client/requirements.txt
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --symlink-install

COPY leitstand-robot-client-template/docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
# Mount your robot.yaml at /config/robot.yaml; run with --network host so DDS reaches the robot,
# and set RMW_IMPLEMENTATION (plus CYCLONEDDS_URI for a mounted Cyclone config) to match it.
CMD ["ros2", "launch", "leitstand_client_ros2", "client.launch.py", "config:=/config/robot.yaml"]
