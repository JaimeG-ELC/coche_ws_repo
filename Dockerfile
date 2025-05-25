# Use NVIDIA CUDA base image with cuDNN (Ubuntu 20.04)
FROM nvidia/cuda:11.4.2-cudnn8-devel-ubuntu20.04

# Set environment variables for CUDA
ENV CUDA_HOME=/usr/local/cuda
ENV PATH=$CUDA_HOME/bin:$PATH
ENV LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Install ROS2 Foxy dependencies
RUN apt-get update && apt-get install -y \
    locales \
    lsb-release \
    gnupg2 \
    curl \
    wget \
    sudo \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

# Add ROS2 apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list

# Install ROS2 Foxy
RUN apt-get update && apt-get install -y \
    ros-foxy-ros-base \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Source ROS2 setup
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc

# Initialize rosdep
RUN rosdep init && rosdep update

# Set environment variables
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Update the package list and install necessary packages for building ROS2
RUN apt-get update && apt-get install -y \
    cmake \
    wget \
    curl \
    nano \
    sudo \
    libbullet-dev \
    tmux \
    python3-pip \
    python3-dev \
    python3-numpy \
    cython3 \
    libhidapi-dev \
    libusb-1.0-0-dev \
    dbus \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/coche_ws

# Set source file to install asio dependency later on
RUN wget https://github.com/chriskohlhoff/asio/archive/asio-1-12-2.tar.gz && \
    tar -xvzf asio-1-12-2.tar.gz && \
    cd asio-asio-1-12-2 && \
    cp -r asio/include/asio /usr/include/ && \
    apt-get update

# Create a workspace directory
WORKDIR /root/coche_ws/src

# Install VESC Drivers
RUN git clone https://github.com/ros-drivers/transport_drivers.git && \
    git clone -b foxy https://github.com/f1tenth/vesc.git
   
# Install Teleop Drivers
RUN git clone -b foxy-devel https://github.com/ros-teleop/teleop_tools.git

# Install Ackermann mux
RUN git clone  -b foxy-devel https://github.com/f1tenth/ackermann_mux.git

#Install Particle Filter repo
RUN git clone -b foxy-devel https://github.com/f1tenth/particle_filter.git

WORKDIR /root/coche_ws/src/particle_filter
RUN pip3 install cython && \
    git clone -b foxy-devel https://github.com/f1tenth/range_libc.git

WORKDIR /root/coche_ws/src/particle_filter/range_libc/pywrapper
RUN chmod +x compile_with_cuda.sh && ./compile_with_cuda.sh

WORKDIR /root/coche_ws

# Install the associated dependencies
RUN rosdep update --include-eol-distros && \
    rosdep install --from-paths src --ignore-src --rosdistro foxy -y

# Install Joy Ros package
RUN apt-get update && apt-get install -y ros-foxy-joy 

# Install other required dependencies
RUN apt install -y ros-foxy-diagnostics

#Install SLAM toolbox
RUN apt install -y ros-foxy-slam-toolbox

#Install RVIZ2
RUN apt install -y ros-foxy-rviz2

RUN source /opt/ros/foxy/setup.bash && colcon build

# Set the entrypoint to run the container in a bash shell
CMD ["/bin/bash"]