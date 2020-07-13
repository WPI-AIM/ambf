FROM tensorflow/tensorflow:1.14.0-gpu-py3

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && apt-get install -q -y tzdata && rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO melodic
# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

RUN apt-get update && apt-get install -y \
    ros-melodic-ros-base=1.4.1-0* apt-utils git \
    && rm -rf /var/lib/apt/lists/*

ENV USERNAME="admin"

RUN useradd -ms /bin/bash ${USERNAME}

ENV HOME="/tmp" \
  AMBF_WS="/tmp/ambf"

WORKDIR ${HOME}
# Make Directory AMBF_WS
RUN git clone --single-branch --branch feat-rl https://github.com/WPI-AIM/ambf.git
WORKDIR ${AMBF_WS}
RUN cd ${AMBF_WS} && \
  git submodule update --init --recursive

# Install apt and pip packages listed in (*-requirements.txt)
WORKDIR ${AMBF_WS}
RUN apt-get update && \
  apt-get -y -qq -o Dpkg::Use-Pty=0 install --no-install-recommends \
  --fix-missing $(cat install/apt-requirements.txt) && \
  cat install/pip-requirements.txt | xargs -n 1 -L 1 pip install -U && \
  apt-get clean && \
  rm -rf /var/lib/apt/lists/*

# Build AMBF
RUN . /opt/ros/melodic/setup.sh && \
  mkdir -p ${AMBF_WS}/build && \
  cd ${AMBF_WS}/build && \
  cmake ../ && \
  make -j$(nproc)

RUN apt-get update && \
  cat install/training-pip-requirements.txt | xargs -n 1 -L 1 pip3 install -U && \
  apt-get clean && \
  rm -rf /var/lib/apt/lists/*

RUN touch ${HOME}/.bashrc && \
  echo "source /opt/ros/melodic/setup.bash" >> ${HOME}/.bashrc && \
  echo "source /tmp/ambf/build/devel/setup.bash" >> ${HOME}/.bashrc
  
WORKDIR ${AMBF_WS}/training_scripts