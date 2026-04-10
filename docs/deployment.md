# Docker Deployment and Build Guide

## Overview

The RoboRacer Stack is deployed via Docker containers to ensure consistent environments across development machines and the physical F1TENTH car. Two container variants are available:

| Variant | Base Image | Use Case |
|---------|-----------|----------|
| **CPU** | `ros:foxy-ros-base-focal` | Development, x86 systems without GPU |
| **GPU** | `f1tenth/focal-l4t-foxy:f1tenth-stack` | Jetson (ARM), CUDA-accelerated particle filter |

## Prerequisites

### Hardware Requirements

| Component | CPU Variant | GPU Variant |
|-----------|-------------|-------------|
| Architecture | x86_64 | ARM64 (Jetson) or x86_64 |
| RAM | 4 GB minimum | 4 GB minimum |
| GPU | Not required | NVIDIA GPU with CUDA support |
| Storage | 5 GB for image | 8 GB for image |

### Software Requirements

- Docker Engine 20.10+
- (GPU only) NVIDIA drivers + [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

## Quick Start

### 1. Clone the Repository

```bash
git clone https://github.com/JaimeG-ELC/coche_ws_repo.git
cd coche_ws_repo
```

### 2. Install Docker

```bash
# Automated script:
./scripts/install_docker.sh

# Or follow: https://docs.docker.com/get-docker/
```

### 3. Build the Container

```bash
# CPU variant:
./build_container.sh

# GPU variant (Jetson / CUDA):
./build_container_gpu.sh
```

### 4. Launch the Container

```bash
# CPU variant:
./launch_container.sh

# GPU variant:
./launch_container_gpu.sh
```

### 5. Build the ROS 2 Workspace (inside container)

```bash
cd /root/coche_ws
colcon build
source install/setup.bash
```

## Dockerfile Details

### CPU Dockerfile

The CPU Dockerfile builds on `ros:foxy-ros-base-focal` and installs:

1. **Build tools**: cmake, build-essential, python3-pip
2. **ASIO library**: Async I/O for transport_drivers (serial communication)
3. **Transport drivers**: Cloned from GitHub for VESC serial communication
4. **ROS 2 packages**:
   - `ros-foxy-joy` - Joystick driver
   - `ros-foxy-urg-node` - Hokuyo LiDAR driver
   - `ros-foxy-diagnostics` - Diagnostics framework
   - `ros-foxy-slam-toolbox` - SLAM
   - `ros-foxy-rviz2` - Visualization
   - `ros-foxy-ackermann-msgs` - Ackermann messages
   - `ros-foxy-nav2-lifecycle-manager` - Navigation lifecycle
   - `ros-foxy-navigation2` - Navigation stack
   - `ros-foxy-tf-transformations` - TF utilities
   - `ros-foxy-geographic-msgs` - Geographic message types
   - `ros-foxy-rqt-common-plugins` - RQT tools
   - `ros-foxy-rqt-tf-tree` - TF tree viewer
5. **Python packages**: `transforms3d`
6. **System libraries**: `libhidapi-dev`, `libusb-1.0-0-dev`, `libgeographic-dev`

### GPU Dockerfile

The GPU Dockerfile builds on `f1tenth/focal-l4t-foxy:f1tenth-stack` (L4T = Linux for Tegra, Jetson-optimized). Key differences from CPU:

- Base image already includes CUDA and ROS 2 Foxy
- Some packages commented out (reduced footprint for embedded deployment):
  - No `slam-toolbox` (maps are pre-built)
  - No `navigation2` (not used on car)
  - No `geographic-msgs` / `libgeographic-dev`
  - No `rqt` tools (no GUI on Jetson)
  - No `diagnostics` (reduced overhead)

### Particle Filter Compilation

The particle filter's RangeLibc library requires separate compilation depending on GPU availability:

```bash
# CPU-only (inside particle_filter/range_libc/pywrapper/):
./compile.sh

# With CUDA support:
./compile_with_cuda.sh
```

These lines are commented out in the Dockerfiles because the particle filter source code is included in the workspace rather than cloned at build time.

## Container Architecture

```
Host System
├── /dev/sensors/vesc          # VESC serial device
├── /dev/sensors/hokuyo        # LiDAR serial device (if serial mode)
├── /dev/input/js0             # Joystick
└── Network: 192.168.0.x       # LiDAR Ethernet (if Ethernet mode)
    │
    ▼
Docker Container (--privileged --network=host)
├── /root/coche_ws/            # ROS 2 workspace
│   ├── src/                   # Source packages (mounted from host)
│   ├── build/                 # Build artifacts
│   ├── install/               # Installed packages
│   └── log/                   # Build/run logs
├── /opt/ros/foxy/             # ROS 2 Foxy installation
└── /usr/local/cuda/           # CUDA toolkit (GPU variant only)
```

### Container Launch Options

The launch scripts typically include:

```bash
docker run -it \
    --privileged \                    # Full device access
    --network=host \                  # Share host network (for LiDAR Ethernet)
    --gpus all \                      # GPU passthrough (GPU variant)
    -v $(pwd)/src:/root/coche_ws/src \ # Mount source for live editing
    roboracer_stack:latest
```

## Device Setup

### Setting Up udev Rules

Run the device setup script on the host system to create persistent device symlinks:

```bash
sudo ./scripts/setup_devices.sh
```

This creates udev rules that map USB devices to consistent paths regardless of enumeration order:
- `/dev/sensors/vesc` - VESC motor controller
- `/dev/sensors/hokuyo` - Hokuyo LiDAR (serial mode)

### Network Configuration for LiDAR

When using the Hokuyo LiDAR over Ethernet:

1. Configure the host network interface to `192.168.0.x` (same subnet as LiDAR)
2. The LiDAR is at `192.168.0.10:10940`
3. Verify connectivity: `ping 192.168.0.10`
4. The container uses `--network=host` to access the LiDAR directly

## Common Operations

### Rebuilding a Single Package

```bash
cd /root/coche_ws
colcon build --packages-select <package_name>
source install/setup.bash
```

### Rebuilding All Packages

```bash
cd /root/coche_ws
colcon build
source install/setup.bash
```

### Running the Full Stack

```bash
# Terminal 1: Hardware bringup
ros2 launch f1tenth_stack bringup.launch.py

# Terminal 2: Localization (in tmux or new terminal)
ros2 launch f1tenth_stack all.launch.py

# Terminal 3: Autonomous driving
ros2 launch f1tenth_stack pp_reactive.launch.py
```

### Recording Data (ROS Bags)

```bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /scan /odom /drive /sensors/core
```

### Playing Back Data

```bash
ros2 bag play <bag_directory>
```

## Troubleshooting

### NVIDIA GPU Not Detected in Container

1. Verify NVIDIA driver: `nvidia-smi` on host
2. Install NVIDIA Container Toolkit
3. Ensure `--gpus all` flag in launch script
4. Test: `docker run --gpus all nvidia/cuda:11.0-base nvidia-smi`

### Build Errors Inside Container

1. Ensure you are inside the container (`docker exec -it <container_id> bash`)
2. Source ROS: `source /opt/ros/foxy/setup.bash`
3. Check dependencies: `rosdep install --from-path src --ignore-src -y`
4. Clean build: `rm -rf build/ install/ log/ && colcon build`

### VESC Not Connecting

1. Check device exists: `ls -la /dev/sensors/vesc`
2. Check permissions: container must run with `--privileged`
3. Check udev rules: `sudo udevadm control --reload-rules && sudo udevadm trigger`

### LiDAR Not Working

- **Ethernet mode**: Verify `ping 192.168.0.10` from inside container
- **Serial mode**: Check `/dev/sensors/hokuyo` exists
- Verify `--network=host` flag in container launch

### Joystick Not Detected

1. Check `/dev/input/js0` exists on host
2. Test with `jstest /dev/input/js0`
3. Ensure `--privileged` flag in container launch
