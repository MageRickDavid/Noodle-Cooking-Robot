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
        self.tty1 = "/dev/pusher12"
        self.tty2 = "/dev/pusher34"
        #self.tty3 = "/dev/pusher3"
        #self.tty4 = "/dev/pusher4"
        #self.tty4 = "/dev/pusher1"

        self.baudrate = 9600

        #self.connect_serial1()
        self.connect_serial2()
        #self.connect_serial3()
        #self.connect_serial4()
        
        
        
        self.call_back_groups_setup()
        self.service_setup()
    

    def connect_serial1(self):
        try: 
           self.ser1 = serial.Serial(port=self.tty1, baudrate=self.baudrate, timeout = 1)
           if self.ser1.is_open:
               self.get_logger().info(f"The {self.tty1} Port is Opened Now!")
               
           else:
               self.get_logger().info(f"The {self.tty1} Port is Opened by another program")
        except Exception as e:
            self.get_logger().error(e)
            return False
    def check_connection1(self):
      try:
          if self.ser1.is_open:
              return True
          else:
              return False
      except:
          return False
      
    def connect_serial2(self):
        try: 
           self.ser2 = serial.Serial(port=self.tty2, baudrate=self.baudrate, timeout = 1)
           if self.ser2.is_open:
               self.get_logger().info(f"The {self.tty2} Port is Opened Now!")
               
           else:
               self.get_logger().info(f"The {self.tty2} Port is Opened by another program")
        except Exception as e:
            self.get_logger().error(e)
            return False
    def check_connection2(self):
      try:
          if self.ser2.is_open:
              return True
          else:
              return False
      except:
          return False

    # def connect_serial3(self):
    #     try: 
    #        self.ser3 = serial.Serial(port=self.tty3, baudrate=self.baudrate, timeout = 1)
    #        if self.ser3.is_open:
    #            self.get_logger().info(f"The {self.tty3} Port is Opened Now!")
               
    #        else:
    #            self.get_logger().info(f"The {self.tty3} Port is Opened by another program")
    #     except Exception as e:
    #         self.get_logger().error(e)
    #         return False
    # def check_connection3(self):
    #   try:
    #       if self.ser3.is_open:
    #           return True
    #       else:
    #           return False
    #   except:
    #       return False
      
    # def connect_serial4(self):
    #     try: 
    #        self.ser4 = serial.Serial(port=self.tty4, baudrate=self.baudrate, timeout = 1)
    #        if self.ser4.is_open:
    #            self.get_logger().info(f"The {self.tty4} Port is Opened Now!")
               
    #        else:
    #            self.get_logger().info(f"The {self.tty4} Port is Opened by another program")
    #     except Exception as e:
    #         self.get_logger().error(e)
    #         return False   
    # def check_connection4(self):
    #   try:
    #       if self.ser4.is_open:
    #           return True
    #       else:
    #           return False
    #   except:
    #       return False


    def call_back_groups_setup(self):
        self.pusher_one_callback_group = MutuallyExclusiveCallbackGroup()
        self.pusher_two_callback_group = MutuallyExclusiveCallbackGroup()
        self.pusher_three_callback_group = MutuallyExclusiveCallbackGroup()
        self.pusher_four_callback_group = MutuallyExclusiveCallbackGroup()


    def service_setup(self):
        self.pusher1_service = self.create_service(PusherMaster, "/pusher/one/open_close", self.open_close1, callback_group=self.pusher_one_callback_group)
        self.pusher2_service = self.create_service(PusherMaster, "/pusher/two/open_close", self.open_close2, callback_group=self.pusher_two_callback_group)
        self.pusher3_service = self.create_service(PusherMaster, "/pusher/three/open_close", self.open_close3, callback_group=self.pusher_three_callback_group)
        self.pusher4_service = self.create_service(PusherMaster, "/pusher/four/open_close", self.open_close4, callback_group=self.pusher_four_callback_group)

    def convert_pusherMasterRequest_to_string(self, PusherMasterRequest :PusherMaster.Request):
        command = f"d{PusherMasterRequest.pusher_id}_{PusherMasterRequest.command}\n"
        return command
    
    def open_close1(self, request, response):
        try:
            command = self.convert_pusherMasterRequest_to_string(request)
            if self.check_connection1():
                self.ser1.write(command.encode())
                response.success = True
            else:
                response.success = False
                self.get_logger().info("The port to arduino could not be opened")
            return response

        except Exception as e:
            self.get_logger().error(e)
            response.success = False
        return response
    
    def open_close2(self, request, response):
        try:
            command = self.convert_pusherMasterRequest_to_string(request)
            if self.check_connection1():
                self.ser1.write(command.encode())
                response.success = True
            else:
                response.success = False
                self.get_logger().info("The port to arduino could not be opened")
            return response

        except Exception as e:
            self.get_logger().error(e)
            response.success = False
        return response

    # def open_close3(self, request, response):
    #     try:
    #         command = self.convert_pusherMasterRequest_to_string(request)
    #         if self.check_connection2():
    #             self.ser2.write(command.encode())
    #             response.success = True
    #         else:
    #             response.success = False
    #             self.get_logger().info("The port to arduino could not be opened")
    #         return response

    #     except Exception as e:
    #         self.get_logger().error(e)
    #         response.success = False
    #     return response
    
    def open_close4(self, request, response):
        try:
            command = self.convert_pusherMasterRequest_to_string(request)
            if self.check_connection2():
                self.ser2.write(command.encode())
                try:
                    serial_response = self.ser2.read(self.ser2.in_waiting or 1).decode()
                    self.get_logger().info(f"first serial_response{serial_response}")
                    while not serial_response:
                        serial_response = self.ser2.read(self.ser2.in_waiting or 1).decode()
                        self.get_logger().info(f"Feedback from arduino {serial_response}")
                        self.get_logger().info("Waiting for pusher to finish")
                        time.sleep(0.5)
                    serial_response = ""
                    self.get_logger().info(f"new serial_response{serial_response}")
                except Exception as e:
                    response.success = False
                    self.get_logger().error(str(e))
                    return response

                response.success = True
            else:
                response.success = False
                self.get_logger().info("The port to arduino could not be opened")
            return response
        except Exception as e:
            self.get_logger().error(e)
            response.success = False
        return response

    def open_close3(self, request, response):
        try:
            command = self.convert_pusherMasterRequest_to_string(request)
            if self.check_connection2():
                self.ser2.write(command.encode())
                try:
                    serial_response = self.ser2.read(self.ser2.in_waiting or 1).decode()
                    self.get_logger().info(f"first serial_response{serial_response}")
                    while not serial_response:
                        serial_response = self.ser2.read(self.ser2.in_waiting or 1).decode()
                        self.get_logger().info(f"Feedback from arduino {serial_response}")
                        self.get_logger().info("Waiting for pusher to finish")
                        time.sleep(0.5)
                    serial_response = ""
                    self.get_logger().info(f"new serial_response{serial_response}")
                except Exception as e:
                    response.success = False
                    self.get_logger().error(str(e))
                    return response
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
        #pusherMasterNode.ser.close()
        pusherMasterNode.destroy_node()
       

if __name__ == '__main__':
    main()        





        
        



        