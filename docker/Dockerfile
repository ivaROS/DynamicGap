# Imitation learning (manual gap selection) workspace: arena-rosnav + DynamicGap on ROS Noetic,
# built, with the IL dataset and trained models in /root/il_data.
# Build with docker/build_image.sh (it stages the data into the build context).
FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive \
    TZ=America/Chicago \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all

# System packages: build tools, the libraries install2.sh installs, the ROS packages the
# Arena-Rosnav Docker guide found missing, and display/recording tools.
RUN apt-get update && apt-get install -y --no-install-recommends \
        git nano less wget curl sudo gdb tzdata \
        python3-pip python-is-python3 python3-vcstool python3-catkin-tools python3-rosdep python3-opencv \
        libopencv-dev liblua5.2-dev libarmadillo-dev liblcm-dev ros-noetic-nlopt \
        ros-noetic-tf2-sensor-msgs ros-noetic-navigation ros-noetic-costmap-converter ros-noetic-teb-local-planner \
        x11-apps x11-utils mesa-utils libgl1-mesa-dri libglvnd0 libgl1 libglx0 libegl1 xvfb ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# Python packages, at the versions of the lab PC. torch is the CPU build of the lab PC's 1.12.1.
RUN pip3 install --no-cache-dir \
        sismic==1.6.6 watchdog==4.0.2 filelock==3.16.1 numpy==1.24.4 scipy==1.10.1 \
        pandas==2.0.3 matplotlib==3.7.5 seaborn==0.13.2 lxml netifaces defusedxml \
    && pip3 install --no-cache-dir torch==1.12.1+cpu --extra-index-url https://download.pytorch.org/whl/cpu

# Sources
WORKDIR /root/arena_ws
COPY il.repos /tmp/il.repos
RUN mkdir src && vcs import --retry 3 src < /tmp/il.repos

# The four IL repos, branch imitation_learning. origin is always ivaROS (where students push);
# IL_SOURCE can point the clone at a staging copy of the branches until they are on ivaROS.
# IL_HEADS (the branches' commits, set by build_image.sh) makes this step re-run when one moves.
ARG IL_SOURCE=https://github.com/ivaROS
ARG IL_HEADS=unknown
RUN echo "IL branches from ${IL_SOURCE}: ${IL_HEADS}" \
    && for r in planners/dynamic_gap:DynamicGap arena/arena-rosnav:arena-rosnav \
                arena/evaluation:arena-evaluation arena/simulation-setup:arena-simulation-setup; do \
           dir=src/${r%%:*}; name=${r#*:}; \
           git clone --branch imitation_learning "${IL_SOURCE}/${name}.git" "$dir" \
           && git -C "$dir" remote set-url origin "https://github.com/ivaROS/${name}.git" || exit 1; \
       done

# Everything else the sources declare
RUN apt-get update \
    && (rosdep init || true) && rosdep update --rosdistro noetic \
    && rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

# Build: the provider packages first (undeclared dependencies), then the rest.
# No build type, as on the lab PC: with Release (-O3) pedsim_simulator aborts ("free(): invalid
# pointer") when the task generator spawns pedestrians.
RUN source /opt/ros/noetic/setup.bash \
    && catkin config --extend /opt/ros/noetic \
    && catkin build --no-status --summarize costmap_2d mbf_msgs mbf_costmap_core \
    && catkin build --no-status --summarize

# Data: the IL dataset and trained models. /home/abdel/il_data is linked too, so the lab PC's paths
# in docs, configs and run.yaml files keep working.
COPY il_data/ /root/il_data/
RUN mkdir -p /home/abdel && ln -s /root/il_data /home/abdel/il_data \
    && mkdir -p /root/il_data/il_gap_selection

# rclone for uploads to the shared Google Drive folder (Ubuntu 20.04's own rclone is from 2019)
ARG RCLONE_VERSION=v1.68.2
RUN curl -fsSL -o /tmp/rclone.deb "https://downloads.rclone.org/${RCLONE_VERSION}/rclone-${RCLONE_VERSION}-linux-amd64.deb" \
    && dpkg -i /tmp/rclone.deb && rm /tmp/rclone.deb

COPY bashrc_il /root/.bashrc_il
COPY il_check_install.sh /usr/local/bin/il_check_install
# a link into the repo, so `git pull` updates the collection script too
RUN ln -s /root/arena_ws/src/planners/dynamic_gap/docker/run_il_dataset.sh /root/run_il_dataset.sh \
    && chmod +x /usr/local/bin/il_check_install && echo 'source /root/.bashrc_il' >> /root/.bashrc

LABEL org.opencontainers.image.source=https://github.com/ivaROS/DynamicGap
LABEL org.opencontainers.image.description="Imitation-learning workspace (Arena-Rosnav + DynamicGap, ROS Noetic) with the IL dataset and models"
CMD ["bash"]
