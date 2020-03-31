## Base image
FROM ros:melodic-ros-base

## Set work directory
RUN mkdir /usr/src/ambf
WORKDIR /usr/src/ambf

## Install required packages
# RUN apt-get install -y git cmake
RUN apt-get update && \
    apt-get install -y ros-melodic-desktop-full \
    libasound2-dev libgl1-mesa-dev xorg-dev libyaml-cpp-dev && \
    rm -r /var/lib/apt/lists

## Install AMBF
# Copy files into docker 
COPY . .
# Update the submodules required by ambf
RUN git submodule update --init --recursive
# Create a build subdirectory, cd into it and build
RUN . /opt/ros/melodic/setup.sh && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j 2

## Entrypoint for the image (command that will execute when invoking docker run)
ENTRYPOINT ["/usr/src/ambf/docker_entrypoint.sh"]
CMD ["bash"]