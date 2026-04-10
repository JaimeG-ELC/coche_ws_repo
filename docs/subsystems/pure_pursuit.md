# Pure Pursuit Path Tracking

## Overview

The Pure Pursuit package (`pure_pursuit_pkg`) implements a geometric path tracking algorithm that steers the vehicle toward a dynamically computed lookahead point on a pre-recorded raceline. It supports two operating modes:

- **Direct mode**: Publishes `AckermannDriveStamped` commands directly to `/drive`.
- **Reactive mode**: Publishes `GoalPoint` messages to `/goalpoint`, which are consumed by the Reactive Follower for safety-validated driving.

**Package**: `pure_pursuit_pkg`  
**Node**: `pure_pursuit_node`  
**Source**: `src/pure_pursuit_pkg/src/pure_pursuit_node.cpp`

## Algorithm

### 1. Lookahead Distance Computation

The lookahead distance adapts to the current vehicle speed:

```
lookahead_dist = clamp(max_lookahead_dist * current_speed / lookahead_ratio,
                       min_lookahead_dist,
                       max_lookahead_dist)
```

At low speeds, the lookahead shrinks for tighter tracking. At high speeds, it extends for smoother trajectories.

### 2. Closest Point Search

A windowed search centered on the last known closest index finds the nearest waypoint to the vehicle:

```
for n in [0, window_size):
    i = (start_index + n - window_size/2) mod n_pathpoints
    distance = euclidean_dist(pathpoints[i], current_pose)
    if distance < shortest:
        closest_pathpoint = i
```

This sliding window avoids O(n) full-path scans on every cycle.

### 3. Lookahead Point Selection

Starting from the closest point window, the algorithm searches forward for the first waypoint that satisfies:

1. Distance from the car >= `lookahead_dist`
2. Point is **in front** of the car (positive X in car frame)

Points are transformed from the map frame to the car frame using the cached TF transform. If no valid point is found, the algorithm falls back to the midpoint of the window.

### 4. Steering Angle Calculation

The steering angle is computed using the Pure Pursuit formula:

```
angle = -Kp * (2 * y_local) / (distance_local^2)
steering_angle = clamp(angle, -max_steering_angle, +max_steering_angle)
```

Where:
- `y_local` is the lateral offset of the lookahead point in the car frame
- `distance_local` is the Euclidean distance to the lookahead point in the car frame
- `Kp` is the proportional steering gain

### 5. Speed Calculation

The base speed comes from the raceline CSV. It is then limited by lateral acceleration:

```
if |steering_angle| > 0.1 rad (~5.7 deg):
    radius = wheelbase / tan(|steering_angle|)       # bicycle model turn radius
    curve_speed = sqrt(max_lateral_acc * radius)      # max speed for given radius
    target_speed = min(target_speed, curve_speed)

speed = clamp(target_speed, min_speed, max_speed)
```

This prevents the vehicle from exceeding a safe lateral acceleration in turns.

### 6. Coordinate Frame Transform

The node caches the `map → base_link` transform at the start of each odometry callback to avoid multiple TF lookups. Map-frame waypoints are converted to car-frame coordinates via:

```
p_car = R_map2car * (p_map - t_car_in_map)
```

Where `R_map2car` is the transpose of the quaternion-derived rotation matrix.

## ROS Interface

### Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `/odom` (configurable) | `nav_msgs/Odometry` | Vehicle odometry for speed and trigger |

### Publications

| Topic | Type | Condition | Description |
|-------|------|-----------|-------------|
| `/drive` (configurable) | `ackermann_msgs/AckermannDriveStamped` | `reactive: false` | Direct drive commands |
| `/goalpoint` (configurable) | `interfaces_pkg/GoalPoint` | `reactive: true` | Goal point for reactive layer |
| `/visualization_marker` | `visualization_msgs/Marker` | Always | RViz markers for closest + lookahead points |

### TF Dependencies

- Requires: `map → base_link` transform (provided by localization)

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `min_lookahead_dist` | double | 0.5 | Minimum lookahead distance (m) |
| `max_lookahead_dist` | double | 4.0 | Maximum lookahead distance (m) |
| `lookahead_ratio` | double | 8.0 | Speed-to-lookahead scaling factor |
| `max_speed` | double | 4.0 | Maximum allowed speed (m/s) |
| `min_speed` | double | 0.4 | Minimum speed floor (m/s) |
| `Kp` | double | 0.25 | Proportional steering gain |
| `max_steering_angle` | double | 14.705 | Maximum steering angle (degrees, converted to radians internally) |
| `window_size` | int | 25 | Sliding window size for waypoint search |
| `max_lateral_acc` | double | 5.0 | Maximum lateral acceleration (m/s^2) |
| `csv_path` | string | (see config) | Path to raceline CSV file |
| `map_frame` | string | `"map"` | Map coordinate frame name |
| `car_frame` | string | `"base_link"` | Car body frame name |
| `odom_topic` | string | `"/odom"` | Odometry input topic |
| `goalpoint_topic` | string | `"/goalpoint"` | Goal point output topic |
| `drive_topic` | string | `"/drive"` | Drive command output topic |
| `reactive` | bool | `true` | Enable reactive mode (GoalPoint output) |

## Raceline Format

Racelines are stored as CSV files in `pure_pursuit_pkg/racelines/`. Each line contains:

```
x, y, v
```

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `x` | float | meters | Global X coordinate (map frame) |
| `y` | float | meters | Global Y coordinate (map frame) |
| `v` | float | m/s | Target velocity at this waypoint |

If the velocity column is missing or unparseable, `min_speed` is used as the default.

### Available Racelines

The package includes 34+ pre-computed racelines organized by track and speed profile:

```
racelines/
├── 01-09_fast.csv          # Track variant, high-speed profile
├── 01-09_medium.csv        # Track variant, medium-speed profile
├── 01-09_slow.csv          # Track variant, low-speed profile
├── 01-09_super_slow.csv    # Track variant, very conservative
├── raceline_23_07_*.csv    # July 23 track variants
├── raceline_27_07_*.csv    # July 27 track variants
├── raceline_casa_*.csv     # Home track variants
├── raceline_roma_*.csv     # Roma track variants
├── reactive_*.csv          # Racelines optimized for reactive mode
└── ...
```

## Launch Files

### Direct Drive Mode
```bash
ros2 launch pure_pursuit_pkg pure_pursuit.launch.py
```
Loads `pure_pursuit_params.yaml` with `reactive: false`.

### Reactive Mode
```bash
ros2 launch pure_pursuit_pkg pure_pursuit_reactive.launch.py
```
Loads `pure_pursuit_params.yaml` with `reactive: true`.

### Combined with Reactive Follower
```bash
ros2 launch f1tenth_stack pp_reactive.launch.py
```
Launches both `pure_pursuit_reactive.launch.py` and `reactive_follower.launch.py`.

## Tuning Guide

### Increasing Track Speed
1. Increase `max_speed` and `max_lateral_acc` together
2. Use a faster raceline CSV with higher `v` values
3. Increase `lookahead_ratio` for smoother high-speed tracking
4. Increase `max_lookahead_dist` to look further ahead

### Improving Cornering
1. Decrease `Kp` for less aggressive steering (smoother but may cut corners)
2. Increase `Kp` for tighter corner following (more responsive but can oscillate)
3. Lower `max_lateral_acc` to force speed reduction in turns
4. Increase `window_size` if the car loses track of the path in tight turns

### Debugging
- Watch RViz markers: red sphere = closest point, green sphere = lookahead point
- If the green sphere jumps erratically, increase `window_size`
- If the car cuts corners, increase `Kp` or decrease `min_lookahead_dist`
- If the car oscillates, decrease `Kp` or increase `min_lookahead_dist`
