ARG ROS_DISTRO=jazzy
FROM ros:$ROS_DISTRO

ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y

# Install memory heavy packages
RUN apt-get install -y \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-ros-gz

# Install essential packages
RUN apt-get install -y \
    python3-pip \
    git \
    bash-completion \
    pre-commit

# Remove docker-cache to enable apt-get install
RUN rm -rf /etc/apt/apt.conf.d/docker-clean && \
    apt-get update -y

ENV SHELL /bin/bash

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]
