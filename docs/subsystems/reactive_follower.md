# Reactive Follower (Obstacle Avoidance)

## Overview

The Reactive Follower package (`reactive_follower_pkg`) implements a gap-based obstacle avoidance algorithm. It acts as a safety layer between the Pure Pursuit planner and the drive actuator: it receives goal points from Pure Pursuit, validates them against real-time LiDAR data, and either forwards the planned command or computes an alternative safe trajectory through the largest available gap.

**Package**: `reactive_follower_pkg`  
**Node**: `reactive_follower_node`  
**Source**: `src/reactive_follower_pkg/src/reactive_follower_node.cpp`

## Algorithm

### Pipeline Overview

```
LiDAR /scan ──▶ Cache latest scan
                    │
GoalPoint ─────────▶ goal_callback triggers pipeline
                    │
                    ▼
            ┌─ Crop to front FOV ─┐
            │                     │
            ▼                     │
     Preprocess LiDAR             │
     (NaN removal)                │
            │                     │
            ▼                     │
     Find closest obstacle        │
            │                     │
            ▼                     │
     Eliminate bubble             │
     (zero out around obstacle)   │
            │                     │
            ▼                     │
     Calculate safety distance    │
     & minimum gap size           │
            │                     │
            ▼                     │
     Convert goal point ──────────┘
     to LiDAR index
            │
            ▼
     Is goal point safe?
      ├── YES: Use Pure Pursuit commands
      └── NO:  Find biggest gap → alternative commands
            │
            ▼
     Publish AckermannDriveStamped
```

### 1. LiDAR Preprocessing

The raw LiDAR scan is first cropped to the front-facing angular window:

```
start_angle = lidar_angle_front_car - (processed_angle / 2)    # e.g., 135 - 67.5 = 67.5 deg
end_angle   = lidar_angle_front_car + (processed_angle / 2)    # e.g., 135 + 67.5 = 202.5 deg
```

Within the cropped window, NaN readings are replaced with the last valid measurement to eliminate sensor gaps.

### 2. Obstacle Bubble Elimination

The closest obstacle within 1.0 m is identified. A "bubble" of `bubble_radius` scan indices is zeroed out around it, creating a virtual exclusion zone:

```
ranges[closest_idx - bubble_radius : closest_idx + bubble_radius] = 0.0
```

This prevents the gap-finding algorithm from selecting paths that pass too close to nearby obstacles.

### 3. Safety Distance and Minimum Gap Size

The safety distance determines how wide a gap must be for the vehicle to pass through safely:

```
safety_distance = safety_distance_min    # currently fixed, speed-dependent logic prepared
```

The minimum number of contiguous LiDAR beams required for a safe gap is computed geometrically:

```
alpha = 2 * atan2(vehicle_width / 2, safety_distance)
min_gap_size = ceil(alpha * lidar_scans / lidar_angle_rad)
```

This ensures the gap is at least as wide as the vehicle plus safety margins at the given distance.

### 4. Goal Point Safety Validation

The Pure Pursuit goal point is converted from car-frame coordinates to a LiDAR scan index:

```
angle = steering_angle + (processed_angle_rad / 2)
index = round(angle * lidar_scans / lidar_angle_rad)
```

The goal is considered safe if all LiDAR readings within `min_gap_size / 2` indices on both sides of the goal index exceed the safety distance threshold. This verifies there is sufficient clearance for the vehicle to pass through.

### 5. Gap Finding

If the goal point is unsafe, the algorithm performs a linear scan to find the largest contiguous gap in the processed LiDAR data:

```
for each scan index:
    if range > 0.5:  mark as free
    else:            mark as blocked

Find largest contiguous free region >= min_gap_size
```

### 6. Alternative Command Generation

When using the biggest gap, the steering angle targets the gap center:

```
best_idx = gap_start + (gap_end - gap_start) / 2
steering = best_angle - (processed_angle_rad / 2)
steering = clamp(steering, -max_steering_angle_rad, +max_steering_angle_rad)
```

Speed is modulated by both distance to the target and the steering magnitude:

```
speed = min(|max_speed * (weight_speed * distance - weight_steering * |steering|)|, max_speed)
speed = clamp(speed, min_speed, max_speed)
```

The steering angle is negated before publishing because the alternative commands compute in the LiDAR frame convention (opposite to the drive convention).

## ROS Interface

### Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` (configurable) | `sensor_msgs/LaserScan` | Raw LiDAR scan data |
| `/goalpoint` (configurable) | `interfaces_pkg/GoalPoint` | Goal from Pure Pursuit (triggers pipeline) |

### Publications

| Topic | Type | Description |
|-------|------|-------------|
| `/drive` (configurable) | `ackermann_msgs/AckermannDriveStamped` | Final validated drive commands |
| `/reactive/laser` | `sensor_msgs/LaserScan` | Debug: cropped and processed LiDAR scan |

### TF Dependencies

- Requires: `laser → base_link` transform (for coordinate conversion)

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `lidarscan_topic` | string | `"/scan"` | LiDAR input topic |
| `drive_topic` | string | `"/drive"` | Drive output topic |
| `goalpoint_topic` | string | `"/goalpoint"` | Goal point input topic |
| `laser_frame` | string | `"laser"` | LiDAR coordinate frame |
| `car_frame` | string | `"base_link"` | Vehicle body frame |
| `lidar_angle` | double | 270.0 | Total LiDAR FOV (degrees) |
| `lidar_angle_front_car` | double | 135.0 | Angle offset to car front (degrees) |
| `lidar_scans` | int | 1180 | Total number of LiDAR samples |
| `max_speed` | double | 2.0 | Maximum output speed (m/s) |
| `min_speed` | double | 0.5 | Minimum output speed (m/s) |
| `max_steering_angle` | double | 14.705 | Max steering angle (degrees) |
| `bubble_radius` | int | 50 | Exclusion zone radius (scan indices) |
| `processed_angle` | double | 135.0 | Front angular window for processing (degrees) |
| `safety_distance_min` | double | 0.6 | Minimum clearance distance (m) |
| `safety_distance_threshold` | double | 2.0 | Speed threshold for dynamic safety (m/s) |
| `safety_distance_gain` | double | 0.2 | Speed-proportional safety gain |
| `vehicle_width` | double | 0.3 | Vehicle width for gap calculation (m) |
| `max_lidar_distance` | double | 20.0 | Maximum valid LiDAR range (m) |
| `weight_speed` | double | 0.5 | Distance weight for speed calculation |
| `weight_steering` | double | 0.5 | Steering weight for speed calculation |

## Integration with Pure Pursuit

The Reactive Follower is designed to work in tandem with Pure Pursuit in reactive mode:

```
Pure Pursuit (reactive: true) ──▶ /goalpoint ──▶ Reactive Follower ──▶ /drive
                                                       ▲
                                        /scan ─────────┘
```

When launched together via `pp_reactive.launch.py`:
1. Pure Pursuit computes the optimal path-following commands
2. Pure Pursuit publishes them as a `GoalPoint` (x, y, speed, steering)
3. Reactive Follower receives the goal and the latest LiDAR scan
4. If the goal direction is obstacle-free, Pure Pursuit's commands pass through
5. If obstacles block the goal, the biggest safe gap is chosen instead

This architecture separates path tracking concerns from obstacle avoidance, allowing each to be tuned independently.

## Tuning Guide

### More Aggressive Obstacle Avoidance
- Increase `bubble_radius` to create larger exclusion zones
- Increase `safety_distance_min` for wider safety margins
- Increase `vehicle_width` (conservative estimate)

### Smoother Alternative Trajectories
- Increase `weight_speed` to prioritize maintaining speed
- Decrease `weight_steering` to allow larger steering corrections

### Debugging
- Subscribe to `/reactive/laser` in RViz to see the processed LiDAR data
- Watch console output: "Pure Pursuit" means the goal passed safety, "Alternative commands" means gap-following activated
- If the car stops unnecessarily, decrease `bubble_radius` or `safety_distance_min`
- If the car clips obstacles, increase `safety_distance_min` or `vehicle_width`
