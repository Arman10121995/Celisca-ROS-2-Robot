# Clean-host reproduction of the Robot Lab workspace (R9.1).
#
# Built from a `git archive` of one revision by
# scripts/clean_host_reproduction.sh: the official ROS 2 Humble base image,
# dependencies resolved only by rosdep from the packages' manifests plus the
# pinned Python packages in requirements.txt, then a full colcon build.
# Nothing is copied from the reference host's install tree or caches.
FROM ros:humble-ros-base-jammy
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip python3-colcon-common-extensions python3-rosdep git xvfb xauth \
      libgl1-mesa-dri libglx-mesa0 mesa-utils \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /ws
COPY ws/src /ws/src
# ORB_SLAM3 needs Pangolin, which is not packaged for Ubuntu 22.04; no
# integrated algorithm uses it (feature_detection is the lab's own).
RUN touch /ws/src/ORB_SLAM3/COLCON_IGNORE
RUN apt-get update && rosdep update --rosdistro humble \
    && (rosdep install --from-paths src --ignore-src -y -r --rosdistro humble 2>&1 | tee /rosdep-install.log) \
    && rm -rf /var/lib/apt/lists/*
COPY requirements.txt /requirements.txt
RUN pip3 install --no-cache-dir -r /requirements.txt 2>&1 | tee /pip-install.log && pip3 freeze > /pip-freeze.txt
COPY ws/ /ws/
RUN touch /ws/src/ORB_SLAM3/COLCON_IGNORE \
    && source /opt/ros/humble/setup.bash \
    && (colcon build --event-handlers console_cohesion+ 2>&1 | tee /colcon-build.log; exit ${PIPESTATUS[0]})
