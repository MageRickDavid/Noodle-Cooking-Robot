# Noodle Cooking Robot Launch File

This repository provides a ROS 2 launch configuration for initializing and running multiple actuator control nodes in a coordinated system.  
The launch file starts servo controllers, rotating motors, grippers, pushers, camera motors, a conveyor, a pump, and a high-level sequencing algorithm.
[![Video Title](https://img.youtube.com/vi/zZVHVhHTDNM/0.jpg)](https://www.youtube.com/watch?v=zZVHVhHTDNM)

---

## Overview

The launch file initializes all low-level and high-level control nodes required for the system to operate as a whole.  
Each node is responsible for controlling a specific mechanical or functional component.

The system is designed to be modular: each actuator runs in its own ROS 2 node, allowing independent debugging, testing, and replacement.

---

## Launched Nodes

### Servo Controllers
| Node Name | Package | Executable | Description |
|----------|--------|------------|-------------|
| `servo_controller_x` | `servo_controller` | `servo_controller_x` | Controls the X-axis servo motor |
| `servo_controller_y` | `servo_controller` | `servo_controller_y` | Controls the Y-axis servo motor |

---

### Rotating Servo Motors
| Node Name | Package | Executable | Description |
|----------|--------|------------|-------------|
| `rotating_servo_Z` | `rotating_servo` | `rotating_servoZ` | Controls rotation around the Z-axis |
| `rotating_servo_X` | `rotating_servo` | `rotating_servoX` | Controls rotation around the X-axis |

---

### End-Effector Controllers
| Node Name | Package | Executable | Description |
|----------|--------|------------|-------------|
| `gripper_controller` | `gripper_controller` | `gripper_controller` | Controls the gripper mechanism |
| `pusher_controllers` | `pushers_controller` | `pushers_controller_services` | Controls pneumatic or mechanical pushers via ROS services |

---

### Camera / Z-Axis Control
| Node Name | Package | Executable | Description |
|----------|--------|------------|-------------|
| `cscam_control` | `cscam_control` | `cscam_control` | Controls camera positioning |
| `control_z_motor` | `cscam_control` | `cscamZ_control` | Controls Z-axis motor for camera system |

---

### Conveyor & Pump
| Node Name | Package | Executable | Description |
|----------|--------|------------|-------------|
| `conveyor_controller` | `conveyor_controller` | `conveyor_control` | Controls conveyor belt motion |
| `pump_control` | `pump_control` | `pump_control` | Controls fluid or vacuum pump |

---

### High-Level Control
| Node Name | Package | Executable | Description |
|----------|--------|------------|-------------|
| `sequence_algorithm` | `sequence` | `sequence_algorithm` | Executes the high-level operation sequence and coordination logic |

---

## Launch File Structure

All nodes are launched using the standard ROS 2 `Node` action:

- Output is printed to the screen
- `emulate_tty=True` is enabled for proper logging and colored output
- Nodes are launched concurrently

---

## How to Run

After building your workspace:

```bash
source install/setup.bash
ros2 launch sequence full_system_launch.launch.py
