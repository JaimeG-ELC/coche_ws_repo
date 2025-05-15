# Use the ROS2 development base image (osrf/ros2:devel as requested)
FROM ros:foxy-ros-base-focal

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
    libhidapi-dev \
    libusb-1.0-0-dev \
    dbus \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/f1tenth_ws

# Create a workspace directory
WORKDIR /root/coche_ws/src

# Set source file to install asio dependency later on
RUN wget https://github.com/chriskohlhoff/asio/archive/asio-1-12-2.tar.gz && \
    tar -xvzf asio-1-12-2.tar.gz && \
    cd asio-asio-1-12-2 && \
    cp -r asio/include/asio /usr/include/ && \
    apt-get update

# Install VESC Drivers
RUN git clone https://github.com/ros-drivers/transport_drivers.git && \
    git clone -b foxy https://github.com/f1tenth/vesc.git
   
# Install Teleop Drivers
RUN git clone -b foxy-devel https://github.com/ros-teleop/teleop_tools.git

# Install Ackermann mux
RUN git clone  -b foxy_devel https://github.com/f1tenth/ackermann_mux.git

#Install Particle Filter repo
RUN git clone https://github.com/f1tenth/particle_filter.git

#For map server
RUN rosdep install -r --from-paths src --ignore-src --rosdistro kinetic -y && \

#For RangeLibc
RUN sudo pip install cython && \
    cd particle_filter && \
    git clone -b foxy_devel -https://github.com/f1tenth/range_libc.git && \
    cd range_libc/pywrappers && \
    ./compile_with_cuda.sh && \
    cd /root/f1tenth_ws

WORKDIR /root/f1tenth_ws

# Install the associated dependencies
RUN rosdep update --include-eol-distros && rosdep install --from-path src --ignore-src -y

# Install Joy Ros package
RUN apt-get update && apt-get install -y ros-foxy-joy 

# Install other required dependencies
RUN apt install -y ros-foxy-diagnostics

#Install SLAM toolbox
RUN apt install -y ros-foxy-slam-toolbox

#Install RVIZ2
RUN apt install -y ros-foxy-rviz2

SHELL ["/bin/bash", "-c"]

RUN source /opt/ros/foxy/setup.bash && colcon build

# Set the entrypoint to run the container in a bash shell
CMD ["/bin/bash"]