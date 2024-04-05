ARG ROS_DISTRO="humble"
FROM ros:${ROS_DISTRO} 
# as basic

LABEL author = "weison_pan"
LABEL email = "weisonweileen@gmail.com"

USER root

#change the default source to aliyun and make everything up to date
RUN sed -i s@/archive.ubuntu.com/@/mirrors.aliyun.com/@g /etc/apt/sources.list && \
    sed -i s@/security.ubuntu.com/@/mirrors.aliyun.com/@g /etc/apt/sources.list && \
    apt-get clean && \
    apt-get update && \
    apt-get upgrade -y 

#install necessary tools
RUN apt-get update && \
    apt-get install -y python3-pip \
    vim  \
    cmake \
    g++ \
    wget \
    unzip \
    build-essential \
    cmake \
    gfortran \
    git \
    libatlas-base-dev \
    libavcodec-dev  \
    libavformat-dev \
    libswscale-dev \
    libgtk2.0-dev \
    libjpeg-dev \
    libopencv-dev \
    libpng-dev \
    libtiff-dev \
    pkg-config \
    python3-dev \
    python3-numpy \
    python3-opencv \
    python3-pycurl \
    webp \
    zlib1g-dev \
    gdb \
    v4l-utils 

#install openCV
RUN mkdir -p ~/opencv && \
    cd ~/opencv && \
    wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip && \
    unzip opencv.zip && \
    rm opencv.zip && \
    cd opencv-4.x && \
    mkdir build && \ 
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_C_EXAMPLES=ON \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D BUILD_EXAMPLES=ON .. && \
    make -j4 && \
    make install && \ 
    ldconfig

RUN apt-get update && \
    apt-get install -y ros-${ROS_DISTRO}-serial-driver \
    # ros-${ROS_DISTRO}-v4l2-camera \ #git clone and colcon build is better 
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-common-plugins \
    ros-${ROS_DISTRO}-rqt-image-view \
    ros-${ROS_DISTRO}-camera-calibration \
    ros-${ROS_DISTRO}-info-manager \
    ros-${ROS_DISTRO}-rviz2 \
    python3-rosdep
    
#install for yolov8 object detection and tracking (installed in '/home/jetson/.local/bin' )
RUN pip install opencv-python==4.8.1.78 \ 
    typing-extensions>=4.4.0 \
    ultralytics==8.1.29     \
    lap                   && \
    pip install --upgrade scipy
    #upgrade package

# install all dependencies of the workspace
# RUN rosdep install --from-paths src --ignore-src -r -y 
# it seems that there is no need because postCreateCommand had do the same thing




ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user, let the user password-free
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME 
    # && sudo usermod -aG dialout $USER         
    # let the user be able to deal with /dev/ttyACM0

## change bashrc and ADD path 
USER $USERNAME
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash'  >> ~/.bashrc && \
    echo 'source /home/ws/install/setup.bash' >> ~/.bashrc  && \
    echo 'export PATH=${PATH}:/home/jetson/.local/bin' >> ~/.bashrc

#为opencv 配置pkg-config，但是profile.d文件夹中默认是没有pkgconfig.sh文件 目前是失败的
# RUN touch /etc/profile.d/pkgconfig.sh && \
#     echo "export PKG_CONFIG_PATH=/usr/lib/x86_64-linux-gnu/pkgconfig/:$PKG_CONFIG_PATH" >> /etc/profile.d/pkgconfig.sh




ENV SHELL /bin/bash
# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************


# [Optional] Set the default user. Omit if you want to keep the default as root.
CMD ["/bin/bash"]