# Localization and State Estimation

## Overview

The RoboRacer Stack provides two complementary localization approaches:

1. **Particle Filter (MCL)** - Monte Carlo Localization using LiDAR scan matching against a known map. Provides the global `map → odom` transform.
2. **EKF / UKF (robot_localization)** - Extended/Unscented Kalman Filter for sensor fusion of odometry and IMU data. Provides smooth, continuous state estimation.

Both systems can operate independently or be combined for robust localization.

---

## Particle Filter (Monte Carlo Localization)

### Overview

The particle filter package implements a standard MCL algorithm that localizes the robot within a known occupancy grid map by matching real-time LiDAR scans against expected scans simulated from candidate poses.

**Package**: `particle_filter`  
**Node**: `particle_filter` (Python)  
**Source**: `src/particle_filter/particle_filter/particle_filter.py`

### Algorithm

#### 1. Initialization

Particles are initialized uniformly across free space in the map, or around a specified initial pose:

```python
particles = np.array([x, y, theta] * MAX_PARTICLES)
weights = np.ones(MAX_PARTICLES) / MAX_PARTICLES
```

#### 2. Motion Model

When an odometry update arrives, each particle is displaced by the measured motion plus Gaussian noise:

```python
delta_x = odom_delta_x + noise_x
delta_y = odom_delta_y + noise_y
delta_theta = odom_delta_theta + noise_theta

# Noise parameters:
# motion_dispersion_x, motion_dispersion_y, motion_dispersion_theta
```

#### 3. Sensor Model

When a LiDAR scan arrives, each particle's weight is updated based on how well the expected scan (ray-cast from the particle's pose) matches the actual scan:

```python
for each particle:
    expected_ranges = ray_cast(particle.x, particle.y, particle.theta, map)
    weight = likelihood(actual_ranges, expected_ranges)
```

The likelihood function uses a mixture model with four components:
- `z_hit`: Gaussian noise around true range (sigma = `sigma_hit`)
- `z_short`: Exponential model for unexpected short readings
- `z_max`: Uniform probability for max-range readings
- `z_rand`: Uniform probability for random noise

#### 4. Resampling

Particles are resampled (with replacement) proportional to their weights, concentrating particles in high-likelihood regions.

#### 5. Pose Estimation

The estimated pose is the weighted mean of all particles, published as a TF transform (`map → odom`).

### Ray Casting with RangeLibc

The particle filter uses **RangeLibc**, a high-performance ray casting library that supports multiple backends:

| Method | Description | Speed |
|--------|-------------|-------|
| `BressenhamLib` | Classic Bresenham line algorithm | Baseline |
| `CDDTLib` | Compressed Directional Distance Transform | Fast |
| `RayMarchingLib` | Sphere tracing / ray marching | Fast |
| `GPURayMarchingLib` | CUDA-accelerated ray marching | Fastest |

The GPU variant requires CUDA and is compiled via `compile_with_cuda.sh`.

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `max_particles` | int | 4000 | Number of particles |
| `max_viz_particles` | int | 60 | Particles to visualize in RViz |
| `squash_factor` | float | 2.2 | Log-likelihood compression factor |
| `z_hit` | float | - | Weight for hit model |
| `z_short` | float | - | Weight for short model |
| `z_max` | float | - | Weight for max-range model |
| `z_rand` | float | - | Weight for random model |
| `sigma_hit` | float | - | Std dev for Gaussian hit model (m) |
| `motion_dispersion_x` | float | - | X motion noise (m) |
| `motion_dispersion_y` | float | - | Y motion noise (m) |
| `motion_dispersion_theta` | float | - | Angular motion noise (rad) |
| `theta_discretization` | int | - | Angular resolution for pre-computation |
| `angle_step` | int | - | LiDAR angle subsampling step |
| `range_method` | string | - | Ray casting backend (cddt, rmgpu, etc.) |

### Maps

The system includes pre-built occupancy grid maps stored as PGM + YAML pairs:

| Map | File | Description |
|-----|------|-------------|
| Casa | `casa.yaml` | Home/lab environment |
| Hallway | `hallway.yaml` | Corridor environment |
| Hall IEEE | `hall_ieee_v0.yaml` | IEEE competition venue |
| Hall UC3M | `hall_uc3m-11-07.yaml` | UC3M hallway (Nov 7) |
| Roma | `roma.yaml` | Roma competition track |
| UC3M Hall | `uc3m_hall.yaml` | UC3M main hall |
| Race Map | `race_map_23_07.yaml` | Race track (Jul 23) |
| Test Day | `test_day_v2.yaml`, `test_day_v4.yaml` | Testing environments |

Each map YAML file specifies:
```yaml
image: map_file.pgm
resolution: 0.05          # meters per pixel
origin: [-x, -y, 0.0]    # map origin in world coordinates
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

### Launch

```bash
ros2 launch f1tenth_stack localize.launch.py
```

---

## EKF / UKF State Estimation (robot_localization)

### Overview

The `robot_localization` package provides Extended Kalman Filter (EKF) and Unscented Kalman Filter (UKF) implementations for fusing multiple sensor streams into a single, smooth state estimate. It maintains a 15-dimensional state vector.

**Package**: `state_estimation_pkg` (contains `robot_localization`)  
**Nodes**: `ekf_node`, `ukf_node`  
**Source**: `src/state_estimation_pkg/robot_localization/src/`

### State Vector

The filter maintains a 15-dimensional state:

```
[x, y, z, roll, pitch, yaw, dx, dy, dz, droll, dpitch, dyaw, ddx, ddy, ddz]
 position    orientation      velocity       angular_vel     acceleration
```

### Sensor Configuration

Each sensor input can selectively contribute to specific state dimensions. The configuration matrix specifies which measurements to fuse:

```yaml
# Example: fuse x, y, yaw from odometry
odom0_config: [true, true, false,    # x, y, z
               false, false, true,   # roll, pitch, yaw
               false, false, false,  # dx, dy, dz
               false, false, false,  # droll, dpitch, dyaw
               false, false, false]  # ddx, ddy, ddz
```

### EKF vs UKF

| Feature | EKF | UKF |
|---------|-----|-----|
| Linearization | First-order Taylor | Sigma point sampling |
| Non-linearity handling | Approximate | Better for highly non-linear |
| Computational cost | Lower | Higher |
| Accuracy | Good for mild non-linearity | Better for strong non-linearity |

For the F1TENTH platform, EKF is typically sufficient given the planar motion model.

### Configuration Files

Multiple EKF configurations are available:

| File | Description |
|------|-------------|
| `ekf_1.yaml` | Single-sensor EKF (odometry only) |
| `ekf_2.yaml` | Dual-sensor EKF (odometry + IMU) |
| `ekf_fused.yaml` | Fused EKF configuration |
| `ekf_2_fused.yaml` | Dual-sensor fused EKF |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/set_pose` | `SetPose` | Reset filter to a given pose |
| `/toggle_filter_processing` | `ToggleFilterProcessing` | Pause/resume filtering |
| `/get_state` | `GetState` | Query current state estimate |

### Launch

```bash
ros2 launch f1tenth_stack robot_localization.launch.py
```

---

## TF Transform Nodes

Three custom TF nodes manage coordinate frame relationships:

### tf_publisher_node

**Source**: `src/f1tenth_stack/src/tf_publisher_node.cpp`

Integrates IMU acceleration to compute position and publishes the `odom → imu` transform. Uses double integration of IMU linear acceleration:

```cpp
velocity += acceleration * gravity * dt
position += velocity * dt + 0.5 * acceleration * gravity * dt^2
```

### tf_odom_node

**Source**: `src/f1tenth_stack/src/tf_odom_node.cpp`

Applies a fixed offset transform from the particle filter's laser-frame odometry to the virtual tf_odom frame:

```cpp
// laser → tf_odom offset: (-0.36, 0, -0.12)
map→tf_odom = (map→laser) * (laser→tf_odom)
```

Also republishes odometry on `/odom` with the `map` frame.

### tf_imu_node

**Source**: `src/f1tenth_stack/src/tf_imu_node.cpp`

Applies calibration offsets to raw IMU data:

```cpp
q_corrected = q_offset * q_raw
```

Subscribes to `/sensors/imu/raw` and publishes corrected IMU on `/sensors/imu`.

---

## Localization Launch Configurations

### Particle Filter + EKF (Recommended)
```bash
ros2 launch f1tenth_stack all.launch.py
```
Launches: bringup + robot_localization + tf_odom + particle filter

### Particle Filter + SLAM
```bash
ros2 launch f1tenth_stack all_slam.launch.py
```
Uses SLAM Toolbox instead of a pre-built map

### Particle Filter Only
```bash
ros2 launch f1tenth_stack localize.launch.py
```

### Pure SLAM (Mapping Mode)
```bash
ros2 launch f1tenth_stack mapping.launch.py
```
For building new maps via teleoperation

### Localize with SLAM (Existing Map)
```bash
ros2 launch f1tenth_stack localize_slam.launch.py
```

## Utility Functions

The `particle_filter/utils.py` module provides:

### CircularArray
Fixed-size circular buffer for smoothing:
```python
buf = CircularArray(10)
buf.append(value)
smoothed = buf.mean()
```

### Timer
Performance monitoring:
```python
timer = Timer(10)
timer.tick()
fps = timer.fps()
```

### Coordinate Conversions
- `map_to_world(x, y, map_info)` - Pixel coordinates to world meters
- `particle_to_pose(particle)` - [x, y, theta] to ROS Pose message
- `angle_to_quaternion(angle)` - Yaw angle to quaternion
