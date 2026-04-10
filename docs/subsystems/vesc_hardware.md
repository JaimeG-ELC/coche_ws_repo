# VESC and Hardware Interface

## Overview

The VESC (Vedder Electronic Speed Controller) subsystem provides the interface between ROS 2 high-level commands and the physical hardware of the F1TENTH car. It handles:

- Serial communication with the VESC motor controller
- Conversion between Ackermann steering commands and VESC motor/servo signals
- Odometry computation from motor encoder feedback
- IMU data acquisition from the VESC's built-in IMU

## Package Structure

```
vesc/
├── vesc/                  # Metapackage
├── vesc_driver/           # Low-level serial communication with VESC
├── vesc_ackermann/        # Ackermann ↔ VESC command conversion + odometry
└── vesc_msgs/             # Custom VESC message definitions
```

---

## VESC Driver

**Package**: `vesc_driver`  
**Node**: `vesc_driver_node`  
**Source**: `src/vesc/vesc_driver/src/`

### Description

The VESC driver provides bidirectional serial communication with the VESC hardware:

- **Outbound**: Receives motor speed (ERPM) and servo position commands, sends them to the VESC via serial protocol
- **Inbound**: Reads VESC telemetry (motor state, IMU data) and publishes it as ROS messages

The VESC 6 MKV electronic speed controller provides advanced motor control features including regenerative braking, current limiting, and an integrated IMU for motion sensing.

### Serial Protocol

The driver uses a custom packet-based protocol over USB serial:
- CRC-16 checksums for data integrity
- Variable-length packets with start/end markers
- Automatic device discovery via UUID lookup

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/sensors/core` | `vesc_msgs/VescStateStamped` | Motor telemetry (speed, current, voltage, temperature) |
| `/sensors/imu/raw` | `sensor_msgs/Imu` | Raw IMU data (orientation, angular velocity, linear acceleration) |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/commands/motor/speed` | `std_msgs/Float64` | Motor speed command (ERPM) |
| `/commands/servo/position` | `std_msgs/Float64` | Servo position command (0.0 to 1.0) |

### Configuration

```yaml
# vesc.yaml (shared parameters)
port: /dev/sensors/vesc
duty_cycle_min: 0.0
duty_cycle_max: 0.0
current_min: 0.0
current_max: 100.0
brake_min: -20000.0
brake_max: 200000.0
speed_min: -23250.0         # ~3.1 m/s reverse
speed_max: 53000.0          # ~7.0 m/s forward
position_min: 0.0
position_max: 0.0
servo_min: 0.1              # Full left
servo_max: 0.9              # Full right
```

---

## Ackermann to VESC Conversion

**Package**: `vesc_ackermann`  
**Node**: `ackermann_to_vesc_node`  
**Source**: `src/vesc/vesc_ackermann/src/ackermann_to_vesc.cpp`

### Description

Converts high-level Ackermann steering commands (speed in m/s, steering angle in radians) to low-level VESC signals (ERPM, servo position).

### Conversion Formulas

#### Speed to ERPM
```
ERPM = speed_to_erpm_gain * speed_mps + speed_to_erpm_offset
     = 7528 * speed + 0
```

Example: 2.0 m/s = 15056 ERPM

#### Steering Angle to Servo Position

The servo conversion uses **asymmetric gains** to compensate for mechanical differences between left and right steering:

```
if steering_angle >= 0 (right turn):
    servo = right_gain * steering_angle + offset
           = 1.628 * angle + 0.482

if steering_angle < 0 (left turn):
    servo = left_gain * steering_angle + offset
           = 1.488 * angle + 0.482
```

The general `steering_angle_to_servo_gain` parameter (-1.0) is overridden by the asymmetric gains in the node implementation.

#### Servo Position Range

| Servo Value | Steering | Notes |
|-------------|----------|-------|
| 0.1 | Full left | -14.705 deg = -0.2567 rad |
| 0.482 | Center | Neutral position |
| 0.9 | Full right | +14.705 deg = +0.2567 rad |

### ROS Interface

| Direction | Topic | Type |
|-----------|-------|------|
| Subscribe | `/drive` | `ackermann_msgs/AckermannDriveStamped` |
| Publish | `/commands/motor/speed` | `std_msgs/Float64` |
| Publish | `/commands/servo/position` | `std_msgs/Float64` |

---

## VESC to Odometry

**Package**: `vesc_ackermann`  
**Node**: `vesc_to_odom_node`  
**Source**: `src/vesc/vesc_ackermann/src/vesc_to_odom.cpp`

### Description

Computes dead-reckoning odometry from VESC motor state feedback and servo commands. Uses a bicycle kinematic model for yaw rate estimation.

### Odometry Integration

```cpp
// 1. Convert ERPM to linear speed
current_speed = -(state.speed - speed_to_erpm_offset) / speed_to_erpm_gain

// 2. Calculate angular velocity from steering angle (bicycle model)
if use_servo_cmd:
    steering_angle = (servo_cmd - offset) / gain
    angular_velocity = current_speed * tan(steering_angle) / wheelbase

// 3. Dead-reckoning integration
x   += current_speed * cos(yaw) * dt
y   += current_speed * sin(yaw) * dt
yaw += angular_velocity * dt
```

### Covariance

Fixed diagonal covariance values:
- Position (x, y): 0.2
- Orientation (yaw): 0.4

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `odom_frame` | string | `"odom"` | Odometry frame ID |
| `base_frame` | string | `"base_link"` | Robot base frame ID |
| `publish_tf` | bool | `false` | Publish odom→base_link TF |
| `use_servo_cmd_to_calc_angular_velocity` | bool | `true` | Use servo position for yaw rate |
| `wheelbase` | double | 0.33 | Vehicle wheelbase (m) |

### ROS Interface

| Direction | Topic | Type |
|-----------|-------|------|
| Subscribe | `/sensors/core` | `vesc_msgs/VescStateStamped` |
| Subscribe | `/commands/servo/position` | `std_msgs/Float64` |
| Publish | `/odom` | `nav_msgs/Odometry` |

---

## VESC Message Definitions

### VescState.msg

Complete motor controller telemetry:

| Field | Type | Description |
|-------|------|-------------|
| `temp_fet` | float64 | FET temperature (C) |
| `temp_motor` | float64 | Motor temperature (C) |
| `current_motor` | float64 | Motor current (A) |
| `current_input` | float64 | Input current (A) |
| `avg_id` | float64 | Average direct-axis current |
| `avg_iq` | float64 | Average quadrature-axis current |
| `duty_cycle` | float64 | PWM duty cycle (0-1) |
| `speed` | float64 | Motor speed (ERPM) |
| `voltage_input` | float64 | Battery voltage (V) |
| `charge_drawn` | float64 | Total charge drawn (Ah) |
| `charge_regen` | float64 | Total charge regenerated (Ah) |
| `energy_drawn` | float64 | Total energy drawn (Wh) |
| `energy_regen` | float64 | Total energy regenerated (Wh) |
| `displacement` | float64 | Tachometer displacement (counts) |
| `distance_traveled` | float64 | Absolute distance (counts) |
| `fault_code` | int32 | Error code (0 = no fault) |
| `pid_pos_now` | float64 | Current PID position |

### VescImu.msg

Built-in IMU data from VESC:

| Field | Type | Description |
|-------|------|-------------|
| `ypr` | Vector3 | Yaw, pitch, roll (degrees) |
| `linear_acceleration` | Vector3 | Linear acceleration (m/s^2) |
| `angular_velocity` | Vector3 | Angular velocity (rad/s) |
| `compass` | Vector3 | Magnetometer readings |
| `orientation` | Quaternion | Orientation quaternion |

---

## Sensor Configuration

### Hokuyo UST-10LX LiDAR

```yaml
# sensors.yaml
urg_node:
  ros__parameters:
    angle_max: 2.355              # +135 degrees
    angle_min: -2.355             # -135 degrees
    ip_address: "192.168.0.10"    # LiDAR IP (Ethernet)
    ip_port: 10940                # LiDAR port
    laser_frame_id: "laser"       # TF frame
    calibrate_time: false
    diagnostics_tolerance: 0.05
    cluster: 1
    skip: 0
```

| Specification | Value |
|---------------|-------|
| Model | Hokuyo UST-10LX |
| Interface | Ethernet |
| IP Address | 192.168.0.10:10940 |
| FOV | 270 degrees (±135 deg) |
| Samples | 1180 per scan |
| Angular Resolution | ~0.229 deg |
| Frame | `laser` |
| Mount Offset | 27 cm forward, 11 cm above base_link |

---

## Throttle Interpolation

The VESC configuration includes a throttle interpolator for smooth acceleration:

```yaml
throttle_interpolator:
  ros__parameters:
    max_acceleration: 2.5          # m/s^2 (limits speed ramp)
    throttle_smoother_rate: 75.0   # Hz (smoothing rate)
    max_servo_speed: 3.2           # rad/s (limits steering rate)
    servo_smoother_rate: 75.0      # Hz (smoothing rate)
```

This prevents sudden speed or steering changes that could cause wheel slip or mechanical stress.

---

## Device Setup

The `setup_devices.sh` script configures udev rules for consistent device naming:

```bash
# Creates symlinks:
# /dev/sensors/vesc    → VESC motor controller
# /dev/sensors/hokuyo  → Hokuyo LiDAR (if serial)
# /dev/input/js0       → Joystick
```

This ensures device paths remain stable across reboots regardless of USB enumeration order.
