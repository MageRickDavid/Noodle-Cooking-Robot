# Package Documentation: `servocontroller`

## Overview
This package holds the code for both, servo_controller_X and servo_controller_Y

## Node
**Node Name**: `servo_controller_Y`  
**Purpose**: Creates services, and topics to manage input and output data from and to the servo motors  
**Dependencies**: `rclpy`, `serial`, `gripper_interfaces`
**Command**: `ros2 run servo_controller servo_controller_Y`

## Topics
### `/servo_controllerY/position`
**Topic Type**: `gripper_interfaces/msg/ServoPosition`
**Description**: Publishes the current absolute position of the servo motor
**Command**: `ros2 topic echo /servo_controllerY/position` 

## Services
### `/servo_controllerY/moveabsNice`

**Service Type**: `/gripper_interfaces/srv/ServoAbsNice`  
**Description**: After inputting a position and a time, the service automatically computes the best combination of acceleration, deceleration and velocities for performing the desired movement.
**WARNING**: Never request a time lower than 0.5 seconds or an excessive distance since it might create a collision
**Command**: `ros2 service call /servo_controllerY/moveabsNice /gripper_interfaces/srv/ServoAbsNice "{position: , time: }"`

#### Request

- `position`: float64 [mm]
- `time`: float64 [s]

#### Response

- `success`: bool
- `message`: string

## Examples using services
### `/rotating_servoZ/rotatingTimeBased`
Begin by turning on the node controlling the rotating servo motor:
```
ros2 run rotating_servo rotating_servoz
```
If the node is successfully on, the service should be available, to move to the absolute 270 degree in 1 second:
```
ros2 service call /rotating_servoZ/rotatingTimeBased /gripper_interfaces/srv/RotatingServoTimeBased "{angle: 270.0, time: 1.0}"
```
Be sure to use floats, if the response is success True, the service worked fine.

## Node
**Node Name**: `servo_controller_X`  
**Purpose**: Creates services, and topics to manage input and output data from and to the servo motors  
**Dependencies**: `rclpy`, `serial`, `gripper_interfaces`
**Command**: `ros2 run servo_controller servo_controller_X`

## Topics
### `/servo_controllerX/position`
**Topic Type**: `gripper_interfaces/msg/ServoPosition`
**Description**: Publishes the current absolute position of the servo motor
**Command**: `ros2 topic echo /servo_controllerX/position` 

## Services
### `/servo_controllerX/moveabsNice`

**Service Type**: `/gripper_interfaces/srv/ServoAbsNice`  
**Description**: After inputting a position and a time, the service automatically computes the best combination of acceleration, deceleration and velocities for performing the desired movement.
**WARNING**: Never request a time lower than 0.5 seconds or an excessive distance since it might create a collision
**Command**: `ros2 service call /servo_controllerX/moveabsNice /gripper_interfaces/srv/ServoAbsNice "{position: , time: }"`

#### Request

- `position`: float64 [mm]
- `time`: float64 [s]

#### Response

- `success`: bool
- `message`: string