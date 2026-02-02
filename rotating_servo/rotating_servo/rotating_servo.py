from . import rotating_servo_api as rservo
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

from gripper_interfaces.srv import RotatingServo
from gripper_interfaces.msg import RotatingServoPosition

COMMANDS = {
"reset_device": [0xFF, 0xFE, 0x00, 0x02, 0xF1, 0x0C],
"CW_90"      : [0xFF, 0xFE, 0x00, 0x07, 0x47, 0x01, 0x01, 0x23, 0x28, 0x00, 0x64],
"CCW_90"       : [0xFF, 0xFE, 0x00, 0x07, 0x48, 0x01, 0x00, 0x23, 0x28, 0x00, 0x64],
"CCW_180"     : [0xFF, 0xFE, 0x00, 0x07, 0xFD, 0x01, 0x00, 0x46, 0x50, 0x00, 0x64],
"CW_180"      : [0xFF, 0xFE, 0x00, 0x07, 0xFC, 0x01, 0x01, 0x46, 0x50, 0x00, 0x64],
"CW_360"      : [ 0xFF, 0xFE, 0x00, 0x06, 0x98, 0x02, 0x01, 0x8C, 0xA0, 0x32],
}


class RotatingServoNode(Node):
    def __init__(self):
        super().__init__("rotating_servoZ_node")
        self.name = "Z"
        self.tty = "/dev/TorqueUSB" + self.name
        self.read_write_feedback = True
        self.current_position = None
        self.connect_serial()
        self.call_back_groups_setup()
        self.service_setup()
        self.publisher_setup()
        self.timer_setup()
        self.get_logger().info(f"Rotating servo {self.name} fully setup!!")

    
    def call_back_groups_setup(self):
        self.rotatingServo_group = MutuallyExclusiveCallbackGroup()
        self.reading_servo_group = MutuallyExclusiveCallbackGroup()
        self.servoPositionPublisher_group = MutuallyExclusiveCallbackGroup()

    def publisher_setup(self):
        try:
            self.rotatingServoPositionPublisher = self.create_publisher(RotatingServoPosition, f"/rotating_servo{self.name}/position",10, callback_group=self.servoPositionPublisher_group)
        except Exception as e:
            self.get_logger().error(str(e))

    def timer_setup(self):
        self.reading_servo = self.create_timer(timer_period_sec=0.1,callback=self.read_position, callback_group=self.reading_servo_group)


    
    def service_setup(self):
        self.rotate_servo = self.create_service(RotatingServo,f"/rotating_servo{self.name}/command",self.rotate_servo, callback_group=self.rotatingServo_group)

    def connect_serial(self):
        try: 
            self.ser = serial.Serial(port=self.tty, 
                                     baudrate= 115200, 
                                     bytesize=serial.EIGHTBITS,
                                     parity=serial.PARITY_NONE,
                                     stopbits=serial.STOPBITS_ONE,
                                     timeout=1)
            if self.ser.is_open:
                self.get_logger().info(f"The {self.tty} Port is Opened Now!")
                
            else:
                self.get_logger().info(f"The {self.tty} Port is Opened by another program")
        except Exception as e:
            self.get_logger().error(e)
            return False

    def check_connection(self):
        try:
            if self.ser.is_open:
                return True
            else:
                return False
        except:
            return False
    

    def read_position(self):
        rotating_position_msg = RotatingServoPosition()
        if self.read_write_feedback:
            if self.check_connection():
                position = rservo.request_abs_position(self.ser)
                self.current_position = position
            else:
                self.get_logger().info(f"Communication with rotating servo {self.name} got lost")
        if self.current_position is not None:
            rotating_position_msg.position = self.current_position
            self.rotatingServoPositionPublisher.publish(rotating_position_msg)
    
    def rotate_servo(self, request, response):
        command = request.direction + "_" + str(request.angle)
        if not command in COMMANDS.keys():
            response.success = False
            response.message = 'The command entered is not part of the available functions'
            return response
        
        if self.check_connection():
            rservo.send_packet(self.ser, COMMANDS[command])
            responseFromServo = rservo.receive_response(serial_port=self.ser)
            #self.get_logger().info(f"Response from rotating servo {self.name} to command {command}: {responseFromServo}")
        else:
            response.success = False
            response.message = f"Command {command} was not sent to rotating servo {self.name}"
            return response
        time.sleep(5.0)
        
        if self.check_connection():
            rservo.reset_device(self.ser)
            responseFromServoReset = rservo.receive_response(serial_port=self.ser)
            #self.get_logger().info(f"Response from rotating servo {self.name} to command {command}: {responseFromServoReset}")
        else:
            response.success = False
            response.message = f"Command {command} was not sent to rotating servo {self.name}"
            return response

        response.success = True
        response.message = "Motor reseted successfully"
        return response

def main():
    rclpy.init()
    rotatinServo = RotatingServoNode()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(rotatinServo, executor=executor)
    except KeyboardInterrupt:
        print('Shutting down')
        rotatinServo.ser.close()
       

if __name__ == '__main__':
    main()    

        

             


