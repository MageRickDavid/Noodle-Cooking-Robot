import serial
import rclpy
import rclpy.duration
import rclpy.executors
import rclpy.logging
import numpy as np
import time
from rclpy.node import Node
import rclpy.time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger

from gripper_interfaces.srv import ServoAbs
from gripper_interfaces.msg import ServoPosition
from gripper_interfaces.srv import GripperCom

COMMANDS ={"read_position": "PFB\r", "read_velocity":"MVEL\r", "disable_command":"K\r", "software_enable_command":"en\r", "move_servo_absolute_value":"moveabs", "get_stop_signal":"stopped\r"}

class ServoMotorNode(Node):
    def __init__(self):
        super().__init__('servo_motor_controller')

        self.ttyGantryY = "/dev/GantryUSBY"
        self.ttyGantryX = "/dev/GantryUSBX"
        
        self.read_write_feedback_Y = True
        self.read_write_feedback_X = True
        
        self.current_position_Y  = None
        self.current_position_X  = None

        
        self.connect_serial()
       

        #TODO: get feedback from limit sensors 
        self.call_back_groups_setup()
        self.timer_setup()
        self.service_setup()
        self.publisher_setup()
        self.send_feedback_to_master()
        self.get_logger().info("Servo motor fully setup!!\n")

    def connect_serial(self):
        try: 
            self.ser_Y = serial.Serial(port=self.ttyGantryY, baudrate= 115200, timeout = 1)
            if self.ser_Y.is_open:
                self.get_logger().info(f"The {self.ttyGantryY} Port is Opened Now!")
                self.ser_Y.write(COMMANDS["software_enable_command"].encode())
                
            else:
                self.get_logger().info(f"The {self.ttyGantryY} Port is Opened by another program")

            self.ser_X = serial.Serial(port=self.ttyGantryX, baudrate= 115200, timeout = 1)
            if self.ser_X.is_open:
                self.get_logger().info(f"The {self.ttyGantryX} Port is Opened Now!")
                self.ser_X.write(COMMANDS["software_enable_command"].encode())
                
            else:
                self.get_logger().info(f"The {self.ttyGantryX} Port is Opened by another program")
        except Exception as e:
            self.get_logger().error(e)
            return False
    
    def check_connectionY(self):
        try:
            if self.ser_Y.is_open:
                return True
            else:
                return False
        except:
            return False
    
    def check_connectionX(self):
       try:
           if self.ser_X.is_open:
               return True
           else:
               return False
       except:
           return False
    
    def publisher_setup(self):
        try:
            self.servoPositionPublisherY = self.create_publisher(ServoPosition, "/servo_controller/positionY",10,callback_group=self.servoPositionPublisherY_group)
        except Exception as e:
            self.get_logger().error(str(e))
        try:
            self.servoPositionPublisherX = self.create_publisher(ServoPosition, "/servo_controller/positionX",10,callback_group=self.servoPositionPublisherX_group)
        except Exception as e:
            self.get_logger().error(str(e))

    def call_back_groups_setup(self):
        self.servoPositionPublisherY_group = MutuallyExclusiveCallbackGroup()
        self.servoPositionPublisherX_group = MutuallyExclusiveCallbackGroup()

        self.reading_servoY_group = MutuallyExclusiveCallbackGroup()
        self.reading_servoX_group = MutuallyExclusiveCallbackGroup()

        self.servoControllerMoveabs_groupY = MutuallyExclusiveCallbackGroup()
        self.servoControllerMoveabs_groupX = MutuallyExclusiveCallbackGroup()
 
    def service_setup(self):
        self.move_servoY = self.create_service(ServoAbs,"/servo_controller/moveabsY",self.moveabs_servoY,callback_group=self.servoControllerMoveabs_groupY)
        self.move_servoX = self.create_service(ServoAbs,"/servo_controller/moveabsX",self.moveabs_servoX,callback_group=self.servoControllerMoveabs_groupX)
        
    def timer_setup(self):
        self.reading_servo_Y = self.create_timer(timer_period_sec=0.1,callback=self.read_positionY, callback_group=self.reading_servoY_group)
        self.reading_servo_X = self.create_timer(timer_period_sec=0.1,callback=self.read_positionX, callback_group=self.reading_servoX_group)

    
    def moveabs_servoY(self, request, response):
        try:
            #Wait to turn of response from feedback
            self.read_write_feedback_Y = False
            time.sleep(0.1)

            # Send command to move servo
            position = request.position             #mm
            velocity = request.velocity             #mm/s
            command = f"{COMMANDS['move_servo_absolute_value']} {position} {velocity}\r"
            if self.check_connectionY():
                self.ser_Y.write(command.encode())
                time.sleep(0.1)
                self.read_write_feedback_Y = True
                time.sleep(0.1)
                while (abs(self.current_position_Y - position)/100) > 0.5:
                    self.get_logger().info("Servo is still moving....please wait......")
                response.success = True
                response.message = "Command to move was sent successfully"
            else:
                response.success = False
                response.message = "It was not possible to connect to the servo"
            return response
        except Exception as e:
            self.get_logger().error(str(e))

    def moveabs_servoX(self, request, response):
        try:
            #Wait to turn of response from feedback
            self.read_write_feedback_X = False
            time.sleep(0.1)

            # Send command to move servo
            position = request.position             #mm
            velocity = request.velocity             #mm/s
            command = f"{COMMANDS['move_servo_absolute_value']} {position} {velocity}\r"
            if self.check_connectionX():
                self.ser_X.write(command.encode())
                time.sleep(0.1)
                self.read_write_feedback_X = True
                time.sleep(0.1)
                while (abs(self.current_position_X - position)/100) > 0.5:
                    self.get_logger().info("Servo is still moving....please wait......")
                response.success = True
                response.message = "Command to move was sent successfully"
            else:
                response.success = False
                response.message = "It was not possible to connect to the servo"
            return response
        except Exception as e:
            self.get_logger().error(str(e))


    def read_positionY(self):
        servo_position = ServoPosition()
        if self.read_write_feedback_Y:
            if self.check_connectionY():
                command = COMMANDS["read_position"]
                self.ser_Y.write(command.encode())
                time.sleep(0.1)
                response = self.ser_Y.read(self.ser_Y.in_waiting or 1).decode().strip().split("\r\n")[1].split(" ")[0]
                
                try:
                    response_float = float(response)
                    self.current_position_Y = response_float

                    #self.get_logger().info(f"current position: \n{self.current_position_Y}\n")
                    # self.get_logger().info(f"total of {len(response)} responses \n")
                except:
                    pass
            else:
                self.get_logger().info("Communication with the servo got lost")
        if self.current_position_Y is not None:
            servo_position.position = self.current_position_Y
            self.servoPositionPublisherY.publish(servo_position)
    
    def read_positionX(self):
        servo_position = ServoPosition()
        if self.read_write_feedback_X:
            if self.check_connectionX():
                command = COMMANDS["read_position"]
                self.ser_X.write(command.encode())
                time.sleep(0.1)
                response = self.ser_X.read(self.ser_X.in_waiting or 1).decode().strip().split("\r\n")[1].split(" ")[0]
                
                try:
                    response_float = float(response)
                    self.current_position_X = response_float

                    #self.get_logger().info(f"current position: \n{self.current_position_X}\n")
                    # self.get_logger().info(f"total of {len(response)} responses \n")
                except:
                    pass
            else:
                self.get_logger().info("Communication with the servo got lost")
        if self.current_position_X is not None:
            servo_position.position = self.current_position_X
            self.servoPositionPublisherX.publish(servo_position)



    def get_limit_sensor_dataY(self):
        command = COMMANDS["get_stop_signal"]
        self.ser_Y.write(command.encode())
        time.sleep(0.1)
        response = self.ser_Y.read(self.ser_Y.in_waiting or 1).decode().strip().split("\r\n")[1].split(" ")[0]
        return response

    def get_limit_sensor_dataX(self):
        command = COMMANDS["get_stop_signal"]
        self.ser_X.write(command.encode())
        time.sleep(0.1)
        response = self.ser_X.read(self.ser_X.in_waiting or 1).decode().strip().split("\r\n")[1].split(" ")[0]
        return response



    

    

  
   

def main():
    rclpy.init()
    servoMotor = ServoMotorNode()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(servoMotor, executor=executor)
    except KeyboardInterrupt:
        print('Shutting down')
        servoMotor.ser.write(COMMANDS["disable_command"].encode())
        #servoMotor.ser2.write("off\n".encode())
        #servoMotor.ser2.close()
        servoMotor.destroy_node()
       

if __name__ == '__main__':
    main()        
