# RoboRacer Stack

A complete ROS 2 autonomous racing platform for the [ROBORACER](https://f1tenth.org/) competition (formerly F1TENTH), featuring path tracking, reactive obstacle avoidance, multi-modal localization, and full hardware integration.

## Collaborators

- **Jaime Garcia Arrojo**: [GitHub Profile](https://github.com/JaimeG-ELC)

---

## Table of Contents

- [Features](#features)
- [System Architecture](#system-architecture)
- [Quick Start](#quick-start)
- [Package Overview](#package-overview)
- [Documentation](#documentation)
- [Operating Modes](#operating-modes)
- [Vehicle Parameters](#vehicle-parameters)
- [Troubleshooting](#troubleshooting)

---

## Features

- **Pure Pursuit Path Tracking** with speed-adaptive lookahead and lateral acceleration limiting
- **Reactive Obstacle Avoidance** using LiDAR gap-based safety validation
- **Hybrid Control** combining planned paths with real-time obstacle avoidance
- **Dual Localization** via Particle Filter (MCL) and Extended Kalman Filter (EKF)
- **SLAM Support** for building new maps with SLAM Toolbox
- **Joystick Teleoperation** with dynamic throttle gain adjustment
- **Waypoint Recording** and visualization tools
- **Docker Deployment** with CPU and GPU (CUDA) variants
- **34+ Pre-built Racelines** with multiple speed profiles per track
- **12+ Pre-built Maps** for various environments

---

## System Architecture

```
┌───────────────────────────────────────────────────────────────────┐
│                        RoboRacer Stack                            │
│                                                                   │
│  PERCEPTION         LOCALIZATION         PLANNING                 │
│  ┌──────────┐      ┌──────────────┐     ┌───────────────────┐    │
│  │ LiDAR    │─────▶│ Particle     │────▶│ Pure Pursuit      │    │
│  │ VESC IMU │─────▶│ Filter (MCL) │     │ (Path Tracking)   │    │
│  │ Odometry │─────▶│ EKF / UKF    │     └─────────┬─────────┘    │
│  └──────────┘      └──────────────┘               │              │
│                                                    ▼              │
│                                          ┌───────────────────┐    │
│  TELEOPERATION                           │ Reactive Follower │    │
│  ┌──────────┐                            │ (Obstacle Avoid.) │    │
│  │ Joystick │───────────────────────────▶│         │         │    │
│  └──────────┘                            │         ▼         │    │
│                                          │ VESC Motor Driver │    │
│                                          └───────────────────┘    │
└───────────────────────────────────────────────────────────────────┘
```

For detailed architecture documentation, see [docs/architecture.md](docs/architecture.md).

---

## Quick Start

### Prerequisites

- **OS**: Ubuntu 20.04 (recommended) or compatible Linux / Windows with WSL2
- **Docker**: 20.10+
- **GPU (optional)**: NVIDIA GPU with drivers + [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

### 1. Clone the Repository

```bash
git clone https://github.com/JaimeG-ELC/coche_ws_repo.git
cd coche_ws_repo
```

### 2. Install Docker

```bash
./scripts/install_docker.sh
# Or follow: https://docs.docker.com/get-docker/
```

### 3. Build and Launch the Container

```bash
# CPU variant:
./build_container.sh
./launch_container.sh

# GPU variant (Jetson / CUDA):
./build_container_gpu.sh
./launch_container_gpu.sh
```

### 4. Build the Workspace (inside the container)

```bash
cd /root/coche_ws
colcon build
source install/setup.bash
```

### 5. Run

```bash
# Manual driving (joystick required):
ros2 launch f1tenth_stack bringup.launch.py

# Full autonomous stack:
ros2 launch f1tenth_stack all.launch.py

# Autonomous driving (Pure Pursuit + Reactive):
ros2 launch f1tenth_stack pp_reactive.launch.py
```

---

## Package Overview

| Package | Description | Language |
|---------|-------------|----------|
| [`f1tenth_stack`](src/f1tenth_stack/) | Core bringup, TF nodes, configs, maps, launch files | C++ / Python |
| [`pure_pursuit_pkg`](src/pure_pursuit_pkg/) | Pure Pursuit path tracking with CSV racelines | C++ |
| [`reactive_follower_pkg`](src/reactive_follower_pkg/) | LiDAR gap-based reactive obstacle avoidance | C++ |
| [`waypoint_generator_pkg`](src/waypoint_generator_pkg/) | Waypoint recording and RViz visualization | C++ |
| [`manual_control_pkg`](src/manual_control_pkg/) | Joystick teleoperation with dynamic throttle | C++ |
| [`interfaces_pkg`](src/interfaces_pkg/) | Custom ROS message definitions (GoalPoint, Odom, VescImu) | ROS IDL |
| [`vesc`](src/vesc/) | VESC motor controller driver, Ackermann conversion, odometry | C++ |
| [`state_estimation_pkg`](src/state_estimation_pkg/) | EKF/UKF sensor fusion (robot_localization) | C++ |
| [`particle_filter`](src/particle_filter/) | Monte Carlo Localization with RangeLibc ray casting | Python |

---

## Documentation

Comprehensive documentation is available in the [`docs/`](docs/) directory:

| Document | Description |
|----------|-------------|
| [Architecture](docs/architecture.md) | System overview, ROS topic graph, TF tree, data pipeline |
| [Pure Pursuit](docs/subsystems/pure_pursuit.md) | Path tracking algorithm, parameters, raceline format, tuning guide |
| [Reactive Follower](docs/subsystems/reactive_follower.md) | Gap-based obstacle avoidance, safety validation, integration |
| [Localization](docs/subsystems/localization.md) | Particle Filter, EKF/UKF, maps, TF transform nodes |
| [VESC Hardware](docs/subsystems/vesc_hardware.md) | Motor controller, Ackermann conversion, sensors, messages |
| [Utilities](docs/subsystems/utilities.md) | Joystick control, waypoint tools, custom messages, TF nodes |
| [Deployment](docs/deployment.md) | Docker setup, build instructions, device configuration |

---

## Operating Modes

| Mode | Launch Command | Description |
|------|---------------|-------------|
| **Manual Teleop** | `ros2 launch f1tenth_stack bringup.launch.py` | Hardware drivers + joystick control |
| **Full Autonomous** | `ros2 launch f1tenth_stack all.launch.py` | Bringup + localization + state estimation |
| **SLAM Autonomous** | `ros2 launch f1tenth_stack all_slam.launch.py` | Full stack with SLAM Toolbox |
| **Pure Pursuit Direct** | `ros2 launch pure_pursuit_pkg pure_pursuit.launch.py` | Path tracking only (no obstacle avoidance) |
| **PP + Reactive** | `ros2 launch f1tenth_stack pp_reactive.launch.py` | Path tracking with reactive safety layer |
| **Mapping** | `ros2 launch f1tenth_stack mapping.launch.py` | Build maps via manual driving + SLAM |
| **Localize Only** | `ros2 launch f1tenth_stack localize.launch.py` | Particle filter localization only |

---

## Hardware

| Component | Model |
|-----------|-------|
| Chassis | Traxxas Rustler 4X4 (1:10 scale) |
| Motor | Castle 1415 (2400 KV) |
| ESC | VESC 6 MKV (integrated IMU) |
| LiDAR | Hokuyo UST-10LX (270 deg, 40 Hz, 30 m) |
| Computer | NVIDIA Jetson Xavier NX |
| Battery | 3S LiPO 5000 mAh |
| Controller | Logitech F710 Wireless |

## Vehicle Parameters

| Parameter | Value |
|-----------|-------|
| Wheelbase | 0.33 m |
| Max Steering Angle | +/-14.705 deg (+/-0.2567 rad) |
| Turning Diameter | ~1.30 m |
| ERPM Gain | 7528 (m/s to electrical RPM) |
| Servo Center | 0.482 |
| Servo Range | [0.1, 0.9] |
| LiDAR FOV | 270 deg (1180 samples) |
| LiDAR | Hokuyo URG via Ethernet (192.168.0.10) |
| base_link to laser | (0.27, 0.0, 0.11) m |

---

## Troubleshooting

### NVIDIA GPU not detected in container

- Ensure NVIDIA Container Toolkit is installed
- Launch with `--gpus all` flag
- If no GPU is available, use the CPU Dockerfile and compile the particle filter with `compile.sh` instead of `compile_with_cuda.sh`

### Build errors

- Verify you are running commands **inside** the container
- Source ROS: `source /opt/ros/foxy/setup.bash`
- Install missing dependencies: `rosdep install --from-path src --ignore-src -y`

### VESC not connecting

- Check device exists: `ls -la /dev/sensors/vesc`
- Run `setup_devices.sh` on the host to configure udev rules
- Container must be launched with `--privileged`

### LiDAR not working

- Ethernet mode: `ping 192.168.0.10` from inside the container
- Ensure `--network=host` in container launch

### Joystick not responding

- Check `/dev/input/js0` on host: `jstest /dev/input/js0`
- Ensure `--privileged` flag in container launch

---

## References

- [F1TENTH Platform](https://f1tenth.org/)
- [ROS 2 Foxy Documentation](https://docs.ros.org/en/foxy/index.html)
- [NVIDIA CUDA Documentation](https://docs.nvidia.com/cuda/)
- [robot_localization Wiki](http://docs.ros.org/en/noetic/api/robot_localization/html/)
- [RangeLibc](https://github.com/kctess5/range_libc)
