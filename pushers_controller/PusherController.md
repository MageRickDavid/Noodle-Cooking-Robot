# Package Documentation: `pushers_controller`

## Overview
This package holds the code for controlling the actuators for the dispensers.

## Node
**Node Name**: `pushers_controller_services` 

**Purpose**: Creates services, to communicate with the boards controlling the actuators attached to the dispensers.

**Dependencies**: `rclpy`, `serial`, `gripper_interfaces`

**Command**: `ros2 run puhsers_controller pusher_controller_services`

## Services
### `"/pusher/open_close"`

**Service Type**: `/gripper_interfaces/srv/PusherTimeMaster`

**Description**: This service accepts open or close command to control all 4 dispensers. It is also possible to control the time this action lasts. 

**WARNING**: If the time inserted is too high, the actuators might cross the allowable limit, hence misaligning the actuators from the dispenser. If this happens, try closing the dispensers using a short time (recommended: 100ms) 

**Command**: `ros2 service call /pusher/open_close /gripper_interfaces/srv/PusherTimeMaster "{id: ,command:  ,time: }"`

#### Request

- `id`: int64 [1,2,3 or 4]
- `command`: string ["open" or "close"]
- `time`: int64 [ms]

#### Response

- `success`: bool
- `message`: string

## Examples using services
### `/pusher/open`
Begin by turning on the node controlling the linear actuators:
```
ros2 run pushers_controller pushers_controller_services
```
If the node is successfully on, the service should be available, to open and close dispenser1, dispenser2, dispenser3, dispenser4 in 1500 ms each and more than one at a time:
- `Opening and closing dispenser 1: `
```
ros2 service call /pusher/open_close /gripper_interfaces/srv/PusherTimeMaster "{id: 1,command: "open", time: 1500}"

ros2 service call /pusher/open_close /gripper_interfaces/srv/PusherTimeMaster "{id: 1,command: "close", time: 1500}"
```

- `Opening and closing dispenser 2: `
```
ros2 service call /pusher/open_close /gripper_interfaces/srv/PusherTimeMaster "{id: 2,command: "open", time: 1500}"

ros2 service call /pusher/open_close /gripper_interfaces/srv/PusherTimeMaster "{id: 2,command: "close", time: 1500}"
```

- `Opening and closing dispenser 3: `
```
ros2 service call /pusher/open_close /gripper_interfaces/srv/PusherTimeMaster "{id: 3,command: "open", time: 1500}"

ros2 service call /pusher/open_close /gripper_interfaces/srv/PusherTimeMaster "{id: 3,command: "close", time: 1500}"
```

- `Opening and closing dispenser 4: `
```
ros2 service call /pusher/open_close /gripper_interfaces/srv/PusherTimeMaster "{id: 4,command: "open", time: 1500}"

ros2 service call /pusher/open_close /gripper_interfaces/srv/PusherTimeMaster "{id: 4,command: "close", time: 1500}"
```

- `Opening and closing dispenser 3 an dispenser 2: `
```
ros2 service call /pusher/open_close /gripper_interfaces/srv/PusherTimeMaster "{id: 32,command: "open", time: 1500}"

ros2 service call /pusher/open_close /gripper_interfaces/srv/PusherTimeMaster "{id: 23,command: "close", time: 1500}"
```
Please notice that when selecting more than one dispenser, the order of the units does not matter. 32 and 23 are understood as wanting to control dispenser 3 and dispenser 2.

- `Opening and closing dispenser 1 ,2 3,4: `
```
ros2 service call /pusher/open_close /gripper_interfaces/srv/PusherTimeMaster "{id: 3214,command: "open", time: 1500}"

ros2 service call /pusher/open_close /gripper_interfaces/srv/PusherTimeMaster "{id: 1243,command: "close", time: 1500}"
```

Be sure to use integers for the id and the time for the action. If by mistake the user inputs a negative value or 0, the actuators will use the default time. If the response is success True, the service worked fine.



