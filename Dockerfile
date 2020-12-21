# Import base ros melodic image
FROM ros:melodic-ros-base-bionic

ENV USERNAME="admin"
RUN useradd -ms /bin/bash ${USERNAME}
RUN usermod -aG sudo ${USERNAME}

ENV HOME="/home/${USERNAME}" \
  AMBF_WS="/home/${USERNAME}/ambf"

# Add apt-utils
RUN apt clean && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get update && \
    apt-get install apt-utils -q -y \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
  apt-get -y -qq install wget gdb

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

RUN touch ${HOME}/.bashrc && \
  echo "source /opt/ros/melodic/setup.bash" >> ${HOME}/.bashrc && \
  echo "source $AMBF_WS/build/devel/setup.bash" >> ${HOME}/.bashrc

RUN . ${HOME}/.bashrc
  
WORKDIR ${AMBF_WS}/training_scripts