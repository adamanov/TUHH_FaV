 FROM px4io/px4-dev-ros-melodic
#FROM px4io/px4-dev-nuttx
# Avoid warnings by switching to noninteractive
ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID



RUN apt-get update \
    && apt-get -y install --no-install-recommends apt-utils dialog 2>&1 \
    && apt-get install -y ros-melodic-desktop-full \
    && apt-get install -y ros-melodic-rqt-multiplot \
    && apt-get install -y vim \
    && apt-get install -y wget \
    && apt-get install -y libgl1-mesa-glx \
    && apt-get install -y libgl1-mesa-dri \
    && apt-get install -y python-pip \
    && apt-get install -y pylint \
    && apt-get install -y xmlstarlet \
    && pip install yapf \
    && pip install rope \
    && apt-get install -y python-catkin-tools \
    && groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

RUN echo 'source /opt/ros/melodic/setup.bash' >> /home/$USERNAME/.bashrc

# Switch back to dialog for any ad-hoc use of apt-get
ENV DEBIAN_FRONTEND=dialog
