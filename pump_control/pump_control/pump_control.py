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
from gripper_interfaces.srv import PumpControl


#* These commands are for just remembering 
#* the dictionary is not yet being used in this script
COMMANDS = {
    "read_status": b'\x55\x56\x00\x00\x00\x00\x00\x00\xAB',
    "relay_on": {
        1: b'\x55\x56\x00\x00\x00\x01\x01\xAD',
        2: b'\x55\x56\x00\x00\x00\x02\x01\xAE',
        3: b'\x55\x56\x00\x00\x00\x03\x01\xAF',
        4: b'\x55\x56\x00\x00\x00\x04\x01\xB0',
    },
    "relay_off": {
        1: b'\x55\x56\x00\x00\x00\x01\x02\xAE',
        2: b'\x55\x56\x00\x00\x00\x02\x02\xAF',
        3: b'\x55\x56\x00\x00\x00\x03\x02\xB0',
        4: b'\x55\x56\x00\x00\x00\x04\x02\xB1',
    },
    
}
###########

def send_command(serial_connection, command):
    """Send a command to the relay module."""
    serial_connection.write(command)
    time.sleep(0.1)  # Wait for response
    response = serial_connection.read_all()
    return response



class PumpControlNode(Node):
    def __init__(self):
        super().__init__('pump_controller_node')
        self.tty = '/dev/Pumps'
        self.baudrate = 9600
        self.call_back_groups_setup()
        self.service_setup()
        self.connect_serial()

        


    def connect_serial(self):
        try: 
           self.ser = serial.Serial(port=self.tty, baudrate=self.baudrate, timeout = 1)
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
      
    def call_back_groups_setup(self):
        self.pump_controller_callback_group = MutuallyExclusiveCallbackGroup()

    
    def service_setup(self):
        self.pump_controller_srv = self.create_service(PumpControl, "/pump_control", self.turn_on_off, callback_group=self.pump_controller_callback_group)


    def turn_on_off(self, request, response):
        try:
            pump_id = request.pumpid                # 1,2,3,4
            command_on_off = request.command        # on or
            if self.check_connection():
                try:
                    pump_response = send_command(self.ser, COMMANDS[f"relay_{command_on_off}"][pump_id])
                except Exception as e:
                    response.success = False
                    response.message =  "Impossible to send command to pumps: "+ str(e)
                    return response
                response.success = True
                response.message = f'Message sent: {pump_response}'
            else:
                response.success = False
                response.message = "The port to relays could not be opened"
            return response
        except Exception as e:
            response.message = str(e)
            response.success = False
        return response



def main():
    rclpy.init()
    pusherMasterNode = PumpControlNode()
    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(pusherMasterNode, executor=executor)
    except KeyboardInterrupt:
        print('Shutting down')
        pusherMasterNode.ser.close()
        #pusherMasterNode.destroy_node()
        pusherMasterNode.get_logger().error("ErrorPump")

       

if __name__ == '__main__':
    main()        



        
        