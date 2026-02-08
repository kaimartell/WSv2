FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg2 \
    lsb-release \
    locales \
    python3 \
    python3-pip \
    python3-yaml \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo ${UBUNTU_CODENAME}) main" \
    > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-vcstool \
    ros-humble-ros-base \
    ros-humble-rqt \
    ros-humble-rqt-graph \
    && (apt-get install -y --no-install-recommends python3-rosdep || apt-get install -y --no-install-recommends python3-rosdep2) \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true
RUN rosdep update

WORKDIR /ros2_ws
COPY ros2_ws/src /ros2_ws/src
COPY patterns /patterns

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && rosdep install --from-paths src --ignore-src -r -y \
    && colcon build --symlink-install

RUN printf '\n# Auto-source ROS for interactive shells.\nsource /opt/ros/humble/setup.bash\nif [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi\n' >> /etc/bash.bashrc \
    && printf '\n# Auto-source ROS for root interactive shells.\nsource /opt/ros/humble/setup.bash\nif [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi\n' >> /root/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
