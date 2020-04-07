# Import base ros melodic image
FROM ros:melodic-ros-base-bionic

ENV HOME="/root" \
    AMBF_WS="/root/ambf"

# Install git
RUN apt-get update && \
    apt-get -y -qq -o Dpkg::Use-Pty=0 install --no-install-recommends \
    --fix-missing apt-utils git vim && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Clone AMBF into $HOME
WORKDIR ${HOME}
RUN git clone https://github.com/DhruvKoolRajamani/ambf.git && \
    cd ${AMBF_WS} && \
    git submodule update --init --recursive && \
    git checkout docker-image

# Install apt and pip packages listed in (*-requirements.txt)
WORKDIR ${AMBF_WS}
COPY install/*-requirements.txt install/
RUN apt-get update && \
    apt-get -y -qq -o Dpkg::Use-Pty=0 install --no-install-recommends \
    --fix-missing $(cat install/apt-requirements.txt) && \
    cat install/pip-requirements.txt | xargs -n 1 -L 1 pip install -U && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Source ROS_PATH
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash"

# Build AMBF
RUN . /opt/ros/melodic/setup.sh && \
    mkdir -p ${AMBF_WS}/build && \
    cd ${AMBF_WS}/build && \
    cmake ../ && \
    make -j$(nproc)

# Entrypoint for the image (command that will execute when invoking docker run)
ENTRYPOINT ["${AMBF_WS}/docker_entrypoint.sh"]
CMD ["bash"]