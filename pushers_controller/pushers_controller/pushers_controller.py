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

from gripper_interfaces.srv import PusherMaster

COMMANDS = {"open_cilinder":"open", "close_cilinder":"close"}

class PushersControlNode(Node):
    def __init__(self):
        super().__init__('pusher_controller_node')
        self.tty = "/dev/ArduinoUNO"
        
        self.baudrate = 9600

        self.connect_serial()
        self.call_back_groups_setup()
        self.service_setup()
        
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
        self.pusher_callback_group = MutuallyExclusiveCallbackGroup()

    def service_setup(self):
        self.pusher_master_service = self.create_service(PusherMaster,"/pusher/open_close", self.open_close, callback_group=self.pusher_callback_group)
   
    def convert_pusherMasterRequest_to_string(self, PusherMasterRequest :PusherMaster.Request):
        command = f"pusher{PusherMasterRequest.pusher_id}_{PusherMasterRequest.command}\n"
        return command
    
    def open_close(self, request, response):
        try:
            command = self.convert_pusherMasterRequest_to_string(request)
            if self.check_connection():
                self.ser.write(command.encode())
                response.success = True
            else:
                response.success = False
                self.get_logger().info("The port to arduino could not be opened")
            return response

        except Exception as e:
            self.get_logger().error(e)
            response.success = False
        return response

def main():
    rclpy.init()
    pusherMasterNode = PushersControlNode()
    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(pusherMasterNode, executor=executor)
    except KeyboardInterrupt:
        print('Shutting down')
        pusherMasterNode.ser.close()
        pusherMasterNode.destroy_node()
       

if __name__ == '__main__':
    main()        





        
        



        