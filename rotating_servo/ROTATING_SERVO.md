# Package Documentation: `rotating_servo`

## Overview
This package holds the code for both, rotating_servoZ (beta) and rotating_servoX (alfa)

## Node
**Node Name**: `rotating_servoZ`  
**Purpose**: Creates services, and topics to manage input and output data from and to the rotating servo motors  
**Dependencies**: `rclpy`, `serial`, `gripper_interfaces`

**Command**: `ros2 run rotating_servo rotating_servoz`

## Topics
### `/rotating_servoZ/position`

**Topic Type**: `gripper_interfaces/msg/RotatingServoPosition`

**Description**: Publishes the current absolute position of the rotating servo motor

**Command**: `ros2 topic echo /rotating_servoZ/position` 

## Services
### `/rotating_servoZ/rotatingTimeBased`

**Service Type**: `/gripper_interfaces/srv/RotatingServoTimeBased` 

**Description**: After inputting a position and a time, the service automatically computes the service rotates the servo motor with the proper acceleration so that the movement gets completed during the requested time.
**WARNING**: Never request a position near 0 degrees or near 360 degrees since the resulting movement might be unpredictable.

**Command**: `ros2 service call /rotating_servoZ/rotatingTimeBased /gripper_interfaces/srv/RotatingServoTimeBased "{angle: , time: }"`

#### Request

- `angle`: float64 [degree]
- `time`: float64 [s]

#### Response

- `success`: bool
- `message`: string

### `/rotating_servoZ/command`

**Service Type**: `/gripper_interfaces/srv/RotatingServo`  

**Description**: There are some movements relative to the current position available for using (CW = clockwise; CCW = counter-clockwise):
- "CW_90"  
- "CCW_90" 
- "CCW_180"
- "CW_180" 
- "CW_360"

The first part of the string says the direction, the second the degrees to rotate.

**WARNING**: The case "CCW_360" has not been coded yet.

**Command**: `ros2 service call /rotating_servoZ/command /gripper_interfaces/srv/RotatingServo "{direction: '', angle: }"`

#### Request

- `direction`: string ['CCW' or 'CW']
- `angle`: int64 [degree]

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

### `/rotating_servoZ/command`
Begin by turning on the node controlling the rotating servo motor:
```
ros2 run rotating_servo rotating_servoz
```
If the node is successfully on, the service should be available, to rotating the servo driver 90 degrees counter-clockwise:
```
ros2 service call /rotating_servoZ/command /gripper_interfaces/srv/RotatingServo "{command: 'CCW', angle: 90}"
```
Be sure to use an integer, if the response is success True, the service worked fine.

## Node
**Node Name**: `rotating_servoX`  
**Purpose**: Creates services, and topics to manage input and output data from and to the rotating servo motors  
**Dependencies**: `rclpy`, `serial`, `gripper_interfaces`

**Command**: `ros2 run rotating_servo rotating_servox`

## Topics
### `/rotating_servoX/position`
**Topic Type**: `gripper_interfaces/msg/RotatingServoPosition`

**Description**: Publishes the current absolute position of the rotating servo motor

**Command**: `ros2 topic echo /rotating_servoX/position` 

## Services
### `/rotating_servoX/rotatingTimeBased`

**Service Type**: `/gripper_interfaces/srv/RotatingServoTimeBased` 

**Description**: After inputting a position and a time, the service automatically computes the service rotates the servo motor with the proper acceleration so that the movement gets completed during the requested time.

**WARNING**: Never request a position near 0 degrees or near 360 degrees since the resulting movement might be unpredictable.

**Command**: `ros2 service call /rotating_servoX/rotatingTimeBased /gripper_interfaces/srv/RotatingServoTimeBased"{angle: , time: }"`

#### Request

- `angle`: float64 [degree]
- `time`: float64 [s]

#### Response

- `success`: bool
- `message`: string

## Examples using services
### `/rotating_servoX/rotatingTimeBased`
Begin by turning on the node controlling the rotating servo motor:
```
ros2 run rotating_servo rotating_servox
```
If the node is successfully on, the service should be available, to move to the absolute 270 degree in 1 second:
```
ros2 service call /rotating_servoX/rotatingTimeBased /gripper_interfaces/srv/RotatingServoTimeBased "{angle: 270.0, time: 1.0}"
```
Be sure to use floats, if the response is success True, the service worked fine.


### `/rotating_servoX/command`
Begin by turning on the node controlling the rotating servo motor:
```
ros2 run rotating_servo rotating_servox
```
If the node is successfully on, the service should be available, to rotating the servo driver 90 degrees counter-clockwise:
```
ros2 service call /rotating_servoX/command /gripper_interfaces/srv/RotatingServo "{command: 'CCW', angle: 90}"
```
Be sure to use an integer, if the response is success True, the service worked fine.




