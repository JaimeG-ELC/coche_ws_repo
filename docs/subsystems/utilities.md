# Utility Packages

## Manual Control (Joystick Teleoperation)

### Overview

The manual control package converts joystick input into Ackermann drive commands, enabling manual teleoperation of the F1TENTH car. It supports dynamic throttle adjustment, constant-speed mode, forward/reverse driving, and enable buttons for safety multiplexing.

**Package**: `manual_control_pkg`  
**Node**: `manual_control_node`  
**Source**: `src/manual_control_pkg/src/manual_control_node.cpp`

### Joystick Mapping

The node expects a standard gamepad layout (e.g., Logitech F710, Xbox controller):

```
Controller Layout:

          [LB]  [RB]                    Buttons:
    [LT]              [RT]             0 = A (Test/Zero speed)
                                       1 = B (Enable button 1)
     ┌─────────────────────┐           5 = RB (Constant throttle)
     │  [↑]           [Y]  │
     │[←] [→]   [X]   [B]  │          Axes:
     │  [↓]           [A]  │           0 = Left stick horizontal (steering)
     │                      │           2 = LT (reverse throttle)
     │  [L]       [R]      │           5 = RT (forward throttle)
     └─────────────────────┘           7 = D-pad vertical (throttle gain adj.)
```

### Throttle Logic

The throttle calculation follows a priority chain:

```
1. Button A (idx 0) pressed?     → speed = 0.0 (test/emergency stop)
2. Button RB (idx 5) pressed?    → speed = constant_throttle
3. LT axis (idx 2) != 1.0?      → speed = -throttle_gain * (LT_mapped)  (reverse)
4. RT axis (idx 5) active?       → speed = throttle_gain * RT_mapped     (forward)
5. None active?                  → speed = 0.0

Axis mapping: raw [-1, 1] → mapped [0, 1] via: mapped = -(raw - 1) / 2
```

### Steering
```
steering = -(left_stick_horizontal * steering_gain) + steering_offset
```

### Dynamic Throttle Gain Adjustment

D-pad up/down adjusts the throttle multiplier in real-time:
- D-pad up: `throttle_gain += 0.05`
- D-pad down: `throttle_gain -= 0.05`
- Edge-triggered (only on button press, not hold)

When both LB and RB are pressed simultaneously, the `throttle_multiplier` parameter is applied to the throttle gain.

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `throttle_gain` | double | 2.0 | Throttle sensitivity multiplier |
| `throttle_multiplier` | double | 3.0 | Boost multiplier (LB+RB) |
| `steering_gain` | double | 0.2567 | Steering sensitivity (rad) |
| `steering_offset` | double | 0.0 | Steering center offset |
| `constant_throttle` | double | 1.0 | Fixed speed for RB button (m/s) |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | Drive commands |
| `/enable_button_0` | `std_msgs/Bool` | Button A state (safety) |
| `/enable_button_1` | `std_msgs/Bool` | Button B state (safety) |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joy` | `sensor_msgs/Joy` | Raw joystick input |

### Launch

```bash
# Included automatically in bringup.launch.py
# Or standalone:
ros2 launch manual_control_pkg manual_control.launch.py
```

---

## Waypoint Generator

### Overview

Records the vehicle's position at regular intervals to create waypoint files (CSV) that can be used as racelines for Pure Pursuit.

**Package**: `waypoint_generator_pkg`  
**Node**: `waypoint_generator_node`  
**Source**: `src/waypoint_generator_pkg/src/waypoint_generator_node.cpp`

### Algorithm

The node polls the `map → base_link` TF transform at 100 Hz (10 ms timer). A new waypoint is recorded only when the vehicle has moved at least `min_distance` from the last recorded point:

```cpp
if euclidean_dist(current_pos, last_recorded_pos) >= min_distance:
    write_to_csv(x, y, velocity)
    last_recorded_pos = current_pos
```

### Output Format

The output CSV matches the Pure Pursuit raceline format:

```csv
x, y, v
1.234, 5.678, 2.0
1.567, 5.890, 2.5
...
```

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `min_distance` | double | 0.1 | Minimum distance between recorded points (m) |
| `csv_path` | string | - | Output CSV file path |
| `map_frame` | string | `"map"` | Map frame for TF lookup |
| `car_frame` | string | `"base_link"` | Car frame for TF lookup |

### TF Dependencies

- Requires: `map → base_link` transform

### Launch

```bash
ros2 launch waypoint_generator_pkg waypoint.launch.py
```

### Workflow

1. Launch the full stack with localization (`all.launch.py`)
2. Launch the waypoint generator
3. Drive the car manually around the track
4. Stop the node - CSV file is written
5. Use the CSV as a raceline in Pure Pursuit

---

## Waypoint Visualizer

### Overview

Loads a waypoint CSV file and publishes RViz markers for visualization of racelines and recorded paths.

**Package**: `waypoint_generator_pkg`  
**Node**: `waypoint_visualizer_node`  
**Source**: `src/waypoint_generator_pkg/src/waypoint_visualizer_node.cpp`

### Visualization

Publishes two marker types:
- **SPHERE_LIST**: Individual waypoint spheres along the path
- **LINE_STRIP**: Continuous line connecting all waypoints

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `csv_file_path` | string | - | Path to waypoint CSV file |
| `frame_id` | string | `"map"` | Coordinate frame for markers |
| `marker_scale` | double | 0.05 | Sphere diameter (m) |
| `marker_color` | double[4] | [1.0, 0.0, 0.0, 1.0] | RGBA color (default: red) |

### Launch

```bash
ros2 launch waypoint_generator_pkg waypoint_visualizer.launch.py
```

---

## Custom Message Definitions

### interfaces_pkg

Custom ROS message types used across the stack:

#### GoalPoint.msg

Inter-node communication between Pure Pursuit and Reactive Follower:

```
float64 x        # Lookahead point X in car frame (m)
float64 y        # Lookahead point Y in car frame (m)
float64 v        # Desired velocity (m/s)
float64 s        # Computed steering angle (rad)
```

#### Odom.msg

Combined odometry and IMU message:

```
std_msgs/Header header
VescImu imu      # IMU data from VESC
float64 v        # Linear velocity (m/s)
float64 s        # Steering angle (rad)
```

#### VescImu.msg / VescImuStamped.msg

VESC built-in IMU data (duplicated in both `interfaces_pkg` and `vesc_msgs`):

```
geometry_msgs/Vector3 ypr                  # Yaw, Pitch, Roll
geometry_msgs/Vector3 linear_acceleration  # m/s^2
geometry_msgs/Vector3 angular_velocity     # rad/s
geometry_msgs/Vector3 compass              # Magnetometer
geometry_msgs/Quaternion orientation       # Quaternion
```

#### VescState.msg / VescStateStamped.msg

Motor controller state (see [VESC Hardware docs](./vesc_hardware.md) for full field list).

---

## TF Transform Nodes

### tf_publisher_node

**Purpose**: Integrates IMU acceleration to produce an `odom → imu` transform.

**Algorithm**:
```
1. Receive IMU message (orientation + linear_acceleration)
2. Rotate acceleration from body frame to world frame using IMU quaternion
3. Double-integrate: acceleration → velocity → position
4. Publish transform with integrated position and IMU orientation
```

**Use case**: Provides odometry from IMU-only when wheel encoders are unavailable.

### tf_odom_node

**Purpose**: Bridges the particle filter's laser-frame output to the standard odometry frame.

**Algorithm**:
```
1. Subscribe to particle filter odometry (map → laser frame)
2. Apply fixed transform: laser → tf_odom at offset (-0.36, 0, -0.12)
3. Publish composed transform: map → tf_odom
4. Republish as standard /odom message with "map" frame
```

### tf_imu_node

**Purpose**: Applies calibration offsets to raw IMU orientation.

**Algorithm**:
```
1. Subscribe to /sensors/imu/raw
2. Apply RPY offset quaternion: q_corrected = q_offset * q_raw
3. Broadcast corrected transform
4. Republish corrected IMU on /sensors/imu
```

**Parameters**:
- `roll_offset`, `pitch_offset`, `yaw_offset`: Calibration angles (rad)
- `x_offset`, `y_offset`, `z_offset`: Position offsets (m)
