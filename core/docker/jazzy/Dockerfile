# Use ROS 2 Jazzy (Ubuntu 24.04)
FROM ros:jazzy-ros-base

ENV USERNAME="admin"
RUN useradd -ms /bin/bash ${USERNAME}

ENV HOME="/home/${USERNAME}" \
    ROS_WS="/home/${USERNAME}/ros_ws" \
    AMBF_WS="/home/${USERNAME}/ros_ws/src/ambf"

# Basic utilities
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        apt-utils wget gdb git python3-pip \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Create workspace folders
RUN mkdir -p ${ROS_WS}/src
COPY . ${AMBF_WS}

WORKDIR ${ROS_WS}

# # Install apt dependencies for AMBF
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        $(cat ${AMBF_WS}/core/install/apt-requirements.txt) \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install pip requirements
RUN python3 -m pip install -U pip && \
    cat ${AMBF_WS}/core/install/pip-requirements.txt | xargs -n 1 -L 1 python3 -m pip install -U

# Build AMBF (ROS 2 uses colcon)
RUN . /opt/ros/jazzy/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source ROS 2 + AMBF environment automatically
RUN echo "source /opt/ros/jazzy/setup.bash" >> ${HOME}/.bashrc && \
    echo "source ${ROS_WS}/install/setup.bash" >> ${HOME}/.bashrc
