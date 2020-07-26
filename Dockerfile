# Import base ros melodic image
FROM ros:melodic-ros-base-bionic

ENV HOME="/root" \
  AMBF_WS="/root/ambf"

# Install git
RUN apt-get update && \
  apt-get -y -qq -o Dpkg::Use-Pty=0 install --no-install-recommends \
  --fix-missing apt-utils git && \
  apt-get clean && \
  rm -rf /var/lib/apt/lists/* && \
  mkdir -p ${AMBF_WS}

# Clone AMBF into $HOME
COPY . ${AMBF_WS}
WORKDIR ${AMBF_WS}
RUN cd ${AMBF_WS} && \
  git submodule update --init --recursive

# Install apt and pip packages listed in (*-requirements.txt)
WORKDIR ${AMBF_WS}
COPY install/*-requirements.txt install/
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

RUN touch /root/.bashrc && \
  echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc && \
  echo "source /root/ambf/build/devel/setup.bash" >> /root/.bashrc

WORKDIR ${HOME}