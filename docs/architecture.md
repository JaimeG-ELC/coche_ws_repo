# System Architecture

## Overview

The RoboRacer Stack is a ROS 2-based autonomous racing platform built for the [ROBORACER](https://f1tenth.org/) competition (formerly F1TENTH). It provides a modular baseline autonomy stack designed as a foundation for future research in autonomous motorsport.

The system runs on ROS 2 Foxy (Ubuntu 20.04) and is deployed via Docker containers on both x86 and ARM (NVIDIA Jetson Xavier NX) hardware with optional CUDA acceleration. It follows the classical **see-think-act** (sense-plan-act) pipeline architecture for clarity, adaptability, and reproducibility.

The architecture uses a modular, publish-subscribe design where independent ROS 2 nodes communicate through well-defined topics. Each subsystem (perception, localization, planning, control) is encapsulated in its own package, enabling independent development, testing, and deployment.

## High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          RoboRacer Stack                                    │
│                                                                             │
│  ┌──────────────┐   ┌──────────────────┐   ┌────────────────────────────┐  │
│  │  PERCEPTION   │   │   LOCALIZATION    │   │        PLANNING            │  │
│  │              │   │                  │   │                            │  │
│  │  URG LiDAR   │──▶│  Particle Filter │──▶│  Pure Pursuit              │  │
│  │  VESC IMU    │──▶│  EKF / UKF       │   │  (Path Tracking)           │  │
│  │  Odometry    │──▶│  TF Publishers   │   │                            │  │
│  └──────────────┘   └──────────────────┘   └──────────┬─────────────────┘  │
│                                                        │                    │
│                                                        ▼                    │
│                                            ┌────────────────────────────┐  │
│                                            │        CONTROL             │  │
│                                            │                            │  │
│                                            │  Reactive Follower         │  │
│                                            │  (Obstacle Avoidance)      │  │
│                                            │         │                  │  │
│                                            │         ▼                  │  │
│  ┌──────────────┐                          │  Ackermann-to-VESC        │  │
│  │  TELEOPERATION│─────────────────────────▶│  (Motor Commands)         │  │
│  │  Joystick    │                          │                            │  │
│  └──────────────┘                          └────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────────┘
```

## ROS 2 Package Structure

```
roboracer_stack/src/
├── f1tenth_stack/          # Core bringup, TF nodes, configs, maps, launch files
├── pure_pursuit_pkg/       # Pure Pursuit path tracking algorithm
├── reactive_follower_pkg/  # Gap-based reactive obstacle avoidance
├── waypoint_generator_pkg/ # Waypoint recording and visualization
├── manual_control_pkg/     # Joystick teleoperation interface
├── interfaces_pkg/         # Custom ROS message definitions
├── vesc/                   # VESC motor controller driver suite
│   ├── vesc_driver/        #   Low-level serial communication
│   ├── vesc_ackermann/     #   Ackermann ↔ VESC conversion
│   └── vesc_msgs/          #   VESC message definitions
├── state_estimation_pkg/   # EKF/UKF (robot_localization)
│   └── robot_localization/ #   Extended/Unscented Kalman Filter
└── particle_filter/        # Monte Carlo Localization (MCL)
    └── range_libc/         #   Accelerated ray-casting library
```

## ROS Topic Graph

### Sensor Data Flow

```
/joy                    ─── Joy msg ───▶  manual_control_node
                                              │
                                              ▼
/drive                  ◀── AckermannDriveStamped ── ackermann_to_vesc_node
                                              ▲
                                              │
                              ┌────────────────┤
                              │                │
                        pure_pursuit_node  reactive_follower_node
                              ▲                ▲
                              │                │
/odom ──────────────────────┘                │
/scan ──────────────────────────────────────┘
```

### Complete Topic Map

| Topic | Message Type | Publisher(s) | Subscriber(s) |
|-------|-------------|-------------|----------------|
| `/joy` | `sensor_msgs/Joy` | `joy_node` | `manual_control_node` |
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | `manual_control_node`, `pure_pursuit_node`, `reactive_follower_node` | `ackermann_to_vesc_node` |
| `/scan` | `sensor_msgs/LaserScan` | `urg_node` | `reactive_follower_node`, `particle_filter` |
| `/odom` | `nav_msgs/Odometry` | `vesc_to_odom_node`, `tf_odom_node` | `pure_pursuit_node`, `particle_filter` |
| `/goalpoint` | `interfaces_pkg/GoalPoint` | `pure_pursuit_node` (reactive mode) | `reactive_follower_node` |
| `/commands/motor/speed` | `std_msgs/Float64` | `ackermann_to_vesc_node` | `vesc_driver_node` |
| `/commands/servo/position` | `std_msgs/Float64` | `ackermann_to_vesc_node` | `vesc_driver_node` |
| `/sensors/core` | `vesc_msgs/VescStateStamped` | `vesc_driver_node` | `vesc_to_odom_node` |
| `/sensors/imu/raw` | `sensor_msgs/Imu` | `vesc_driver_node` | `tf_imu_node` |
| `/sensors/imu` | `sensor_msgs/Imu` | `tf_imu_node` | `robot_localization` |
| `/visualization_marker` | `visualization_msgs/Marker` | `pure_pursuit_node`, `waypoint_visualizer_node` | RViz |
| `/reactive/laser` | `sensor_msgs/LaserScan` | `reactive_follower_node` | RViz (debug) |
| `/enable_button_0` | `std_msgs/Bool` | `manual_control_node` | Safety multiplexer |
| `/enable_button_1` | `std_msgs/Bool` | `manual_control_node` | Safety multiplexer |

## TF Frame Tree

The transform tree defines spatial relationships between all coordinate frames:

```
map
 └── odom                          (published by: particle_filter / robot_localization)
      └── base_link                (published by: vesc_to_odom_node)
           ├── laser               (static: x=0.27, y=0.0, z=0.11)
           └── imu                 (static: x=0.09, y=-0.02, z=0.06, roll=π)
```

### Frame Descriptions

| Frame | Description |
|-------|-------------|
| `map` | Global fixed frame. Origin at the map's (0,0). Aligned with the occupancy grid. |
| `odom` | Odometry frame. Continuous but subject to drift. Set by the localization system. |
| `base_link` | Robot body frame. Origin at rear axle center, X forward, Y left, Z up. |
| `laser` | LiDAR sensor frame. Offset 27 cm forward and 11 cm above `base_link`. |
| `imu` | IMU sensor frame. Offset 9 cm forward, 2 cm right, 6 cm above `base_link`. Rotated 180 deg around X. |

## Operating Modes

The system supports several operating modes controlled through launch file composition:

### 1. Manual Teleoperation
```bash
ros2 launch f1tenth_stack bringup.launch.py
```
Brings up hardware drivers and joystick control only.

### 2. Autonomous with Particle Filter Localization
```bash
ros2 launch f1tenth_stack all.launch.py
```
Full stack: bringup + robot_localization + tf_odom + particle filter localization.

### 3. Autonomous with SLAM
```bash
ros2 launch f1tenth_stack all_slam.launch.py
```
Full stack with SLAM Toolbox for simultaneous mapping and localization.

### 4. Pure Pursuit Only (Direct Drive)
```bash
ros2 launch pure_pursuit_pkg pure_pursuit.launch.py
```
Path tracking without reactive obstacle avoidance.

### 5. Pure Pursuit + Reactive Follower (Hybrid)
```bash
ros2 launch f1tenth_stack pp_reactive.launch.py
```
Pure Pursuit publishes goal points; Reactive Follower validates safety and issues final drive commands.

### 6. Mapping
```bash
ros2 launch f1tenth_stack mapping.launch.py
```
Drives manually while building an occupancy grid map using SLAM Toolbox.

## Data Pipeline

The autonomous driving pipeline processes data in the following order:

```
1. SENSING          Hokuyo LiDAR → /scan
                    VESC IMU → /sensors/imu/raw → tf_imu_node → /sensors/imu
                    VESC Encoder → /sensors/core → vesc_to_odom → /odom

2. LOCALIZATION     Particle Filter: /scan + /odom + map → TF(map→odom)
                    EKF/UKF: /odom + /sensors/imu → fused /odometry/filtered

3. PLANNING         Pure Pursuit: TF(map→base_link) + raceline CSV → steering + speed
                    → publishes GoalPoint (reactive mode) or AckermannDrive (direct)

4. SAFETY           Reactive Follower: /scan + GoalPoint → gap analysis
                    → validates goal safety → publishes final AckermannDrive

5. ACTUATION        ackermann_to_vesc: AckermannDrive → ERPM + servo position
                    vesc_driver: sends commands via serial to VESC ESC
```

## Hardware Interface

### Bill of Materials

| Component | Model | Interface | Notes |
|-----------|-------|-----------|-------|
| Chassis | Traxxas Rustler 4X4 | - | 1:10 scale, 4WD, independent suspension |
| Motor | Castle 1415 (2400 KV) | Via VESC | Brushless DC, integrated encoder |
| Motor Controller | VESC 6 MKV | USB Serial (/dev/sensors/vesc) | ESC with integrated IMU, regenerative braking |
| LiDAR | Hokuyo UST-10LX | Ethernet (192.168.0.10:10940) | 270 deg FOV, 1180 samples, 40 Hz, 30 m range, 0.25 deg resolution |
| On-Board Computer | NVIDIA Jetson Xavier NX | - | 6-core ARM Carmel CPU, 384-core NVIDIA Volta GPU |
| Battery | 3S LiPO 5000 mAh | XT60 | Max 4S per ROBORACER rules |
| Power Board | JTK3024S12 DC-DC | - | 9-36V input, 12V regulated output |
| Joystick | Logitech F710 | USB wireless | Supported via ROS joy package |
| IMU | VESC Built-in | Via VESC serial | 9-DOF IMU integrated in VESC 6 MKV |
| WiFi | USB WiFi Dongle | USB | Remote monitoring and debugging |
| Platform Deck | Custom fabricated | - | Mounting surface for OBC, VESC, power board |

### Vehicle Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Wheelbase | 0.33 m | Distance between front and rear axles |
| Max Steering Angle | ±14.705 deg (±0.2567 rad) | Mechanical limit |
| Turning Diameter | ~1.30 m | At full lock |
| ERPM Gain | 7528 | Converts m/s to electrical RPM |
| Servo Center | 0.482 | Neutral steering position |
| Servo Range | [0.1, 0.9] | Min/max servo values |
| Speed Range | [-23250, 53000] ERPM | Reverse/forward limits |
| base_link→laser | (0.27, 0, 0.11) m | LiDAR mount offset |

## Dependencies

### ROS 2 Packages
- `rclcpp`, `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`
- `ackermann_msgs` - Ackermann steering messages
- `tf2_ros`, `tf2_geometry_msgs` - Transform framework
- `nav2_lifecycle_manager`, `nav2_map_server` - Navigation2 map serving
- `slam_toolbox` - SLAM for mapping
- `joy` - Joystick driver
- `urg_node` - Hokuyo LiDAR driver
- `diagnostics` - Diagnostic aggregator

### System Libraries
- `Eigen3` - Linear algebra (used in Pure Pursuit)
- `ASIO` - Async I/O (used in transport_drivers)
- `libhidapi`, `libusb-1.0` - USB device access
- `transforms3d` (Python) - 3D coordinate transformations
- `numpy`, `cython` (Python) - Numerical computing

### Optional
- NVIDIA CUDA - GPU-accelerated particle filter ray casting
- RangeLibc - High-performance 2D ray casting library

## Competition Context (ROBORACER)

The system is designed for the ROBORACER competition, which features three dynamic events:

| Event | Description |
|-------|-------------|
| **Practice Session** | Map the track and test algorithms (regulated + open sessions) |
| **Time Trial** | Qualifying: drive the track as fast and consistently as possible |
| **Head-to-Head** | Two cars race simultaneously, requiring robust localization and safe overtaking |

### Competition Rules (Key Constraints)
- Chassis: any 1:10 scale car (2WD or 4WD)
- LiDAR: max 30 m range, 40 Hz, 0.25 deg resolution
- Motor: brushless DC, max 3500 KV
- Battery: max one 4S powering the motor
- All computation must be onboard
- 12x12x20 cm rear-mounted box required for opponent LiDAR detection

### Trajectory Optimization

Racelines are generated using an adaptation of the [TUMFTM global trajectory optimizer](https://github.com/TUMFTM/global_racetrajectory_optimization) with multiple optimization objectives:

| Objective | Description | Use Case |
|-----------|-------------|----------|
| Shortest path | Minimizes distance | Not suitable for racing |
| **Minimum curvature** | Minimizes lateral acceleration | **Primary choice** - smooth, high-speed corners |
| Minimum time | Fastest lap considering dynamics | Requires more parameters, longer computation |
| Min time + powertrain | Includes motor/battery characteristics | Most realistic but complex |

The minimum curvature path is generated via quadratic programming (QP) formulation, producing both the path and a matching velocity profile.

## Validated Performance

Performance measured over 20 laps on an 18 m indoor track (CPU-only, 100 particles):

| Metric | Slow Pace (2 m/s) | Fast Pace (4 m/s) |
|--------|-------------------|-------------------|
| Lateral error (avg) | -0.027 m | 0.002 m |
| Lateral error (std) | 0.113 m | 0.095 m |
| Longitudinal error (avg) | -0.307 m/s | -0.031 m/s |
| Longitudinal error (std) | 0.643 m/s | 0.521 m/s |
| Average lap time | 7.28 s | 7.23 s |
| Best lap time | 6.93 s | 6.72 s |

CPU utilization during head-to-head racing (PP + FTG): peak < 23%. State estimation was the most resource-intensive module (~9.7%), while control consumed negligible CPU.

## Development Methodology

The project follows a three-phase iterative methodology:

1. **Simulation**: Initial algorithm development using the ROBORACER simulator and Gazebo
2. **Controlled Testing**: Low-speed physical validation, progressive speed increase, safety protocols, data logging
3. **Competition Validation**: Time trial and head-to-head racing under competition conditions
