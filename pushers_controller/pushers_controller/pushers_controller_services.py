import serial
import asyncio
import serial_asyncio
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
import time as t

from gripper_interfaces.srv import PusherTimeMaster

COMMANDS = {"open_cilinder":"open", "close_cilinder":"close"}
PUSHERS = ['1','2','3','4']

class SerialReader(asyncio.Protocol):
    def __init__(self, port_name, stop_event):
        self.port_name = port_name
        self.stop_event =stop_event
        self.received_data = None
    
    def data_received(self, data):
        
        #* Called when data received
        message = data.decode().strip()
        
        #* store received message
        self.received_data = message

        #* Notify data was received
        self.stop_event.set()
        


class PushersControlNode(Node):
    def __init__(self):
        super().__init__('pusher_controller_node')
        self.tty1 = "/dev/pusher12"
        self.tty2 = "/dev/pusher34"
        
        self.baudrate = 9600

        self.connect_serial1()
        self.connect_serial2()
       
        self.call_back_groups_setup()
        self.service_setup()
    

    def connect_serial1(self):
        try: 
           self.ser1 = serial.Serial(port=self.tty1, baudrate=self.baudrate, timeout = 5)
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
           self.ser2 = serial.Serial(port=self.tty2, baudrate=self.baudrate, timeout = 5)
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

    def detect_pushers(self, idReceived):
        message1 = ""
        message2 = ""
        if idReceived > 4321:
            print("The bigger combination is 4321")
            return False
        
        stringId = str(idReceived)
        #self.get_logger().info(f"the len is: {len(stringId)}")
        #self.get_logger().info(f"the set is: {set(stringId)}")
        if (len(stringId) != len(set(stringId))):
            print("Please do not repeat ids")
            return False
        
        for number in stringId:
            if number not in PUSHERS:
                print("allowable pushers only from 1 to 4")
                return False
            if ((number == '1') or (number == '2')):
                message1 = message1 + number
            else:
                message2 = message2 + number
        idsToMove = [message1, message2]
        idsToMove = [ids for ids in idsToMove if ids != ""]
        return idsToMove


    def call_back_groups_setup(self):
       self.pusher_master_group = MutuallyExclusiveCallbackGroup()


    def service_setup(self):
        self.pusher_time_master_srv = self.create_service(PusherTimeMaster, "/pusher/open_close", self.pusher_master, callback_group=self.pusher_master_group)

    def convert_pusherMasterRequest_to_string(self, PusherMasterRequest :PusherTimeMaster.Request):
        command = f"{PusherMasterRequest.pusher_id}_{PusherMasterRequest.command}_{PusherMasterRequest.time}\n"
        return command
    
    def generate_command(self, pusher_id, command, time):
        command = f"{pusher_id}_{command}_{time}\r\n"
        return command
    
    async def read_from_two_ports(self):
        loop = asyncio.get_running_loop()
        
        #*Event to signal when responses are received
        stop_event1 = asyncio.Event()
        stop_event2 = asyncio.Event()

        #*Create readers for both serial ports
        _, protocol1 = await serial_asyncio.create_serial_connection(loop, lambda: SerialReader(self.tty1, stop_event1), self.tty1, baudrate=self.baudrate)
        _, protocol2 = await serial_asyncio.create_serial_connection(loop, lambda: SerialReader(self.tty2, stop_event2), self.tty2, baudrate=self.baudrate)

        await asyncio.gather(stop_event1.wait(), stop_event2.wait())
        loop.stop()
        return [protocol1.received_data, protocol2.received_data]

    async def pusher_master(self, request, response):
        pushersToMove = self.detect_pushers(request.id)
        if pushersToMove == False:
            response.success = False
            response.message = "The id used couldn't be identified"
            return response
        commandOpenClose = request.command
        time = request.time  #* Negative or 0 time will result in arduino using the default time
        if (len(pushersToMove)<2):
            command = self.generate_command(pushersToMove[0], commandOpenClose, time)
            if (("1" in pushersToMove[0]) or ("2" in pushersToMove[0]) or ("21" in pushersToMove[0]) or ("12" in pushersToMove[0])):
                if self.check_connection1():
                    self.get_logger().info(f"the command I am about to send is: {command}")
                    self.get_logger().info(f"I am sending to port {self.tty1}")
                    
                    self.ser1.reset_input_buffer()
                    self.ser1.write(command.encode("utf-8"))
                    try:
                        serial_response = self.ser1.read(self.ser1.in_waiting).decode().strip()
                        while True:
                            self.get_logger().info(f"first serial_response: {serial_response}")
                            if (("Closed" in serial_response) or ("Opened" in serial_response)):
                                break
                            if "Unknown" in serial_response:
                                response.success = False
                                response.message = serial_response
                                return response
                            self.get_logger().info("Waiting for response from arduino")
                            serial_response = self.ser1.read(self.ser1.in_waiting).decode().strip()
                            t.sleep(0.25)
                        response.success = True
                        response.message = serial_response
                        serial_response = ""
                        return response
                    except Exception as e:
                        response.success = False
                        response.message = str(e)
                        return response
                else:
                    response.success = False
                    self.get_logger().info("The port to arduino could not be opened")
                    return response
            else:
                if self.check_connection2():
                    self.get_logger().info(f"the command I am about to send is: {command}")
                    self.get_logger().info(f"I am sending to port {self.tty2}")
                    self.ser2.reset_input_buffer()
                    self.ser2.write(command.encode("utf-8"))
                    try:
                        serial_response = self.ser2.read(self.ser2.in_waiting).decode().strip()
                        while True:
                            self.get_logger().info(f"first serial_response: {serial_response}")
                            if (("Closed" in serial_response) or ("Opened" in serial_response)):
                                break
                            if "Unknown" in serial_response:
                                response.success = False
                                response.message = serial_response
                                return response
                            self.get_logger().info("Waiting for response from arduino")
                            serial_response = self.ser2.read(self.ser2.in_waiting).decode().strip()
                            t.sleep(0.25)
                        response.success = True
                        response.message = serial_response
                        serial_response = ""
                        return response
                    except Exception as e:
                        response.success = False
                        response.message = str(e)
                        return response
                else:
                    response.success = False
                    response.message = "The port to arduino could not be opened"
                    return response
        else:
            command1 = self.generate_command(pushersToMove[0], commandOpenClose, time)
            command2 = self.generate_command(pushersToMove[1], commandOpenClose, time)
            if self.check_connection1() and self.check_connection2():
                try:
                    self.ser1.reset_input_buffer()
                    self.ser2.reset_input_buffer()

                    self.ser1.write(command1.encode("utf-8"))
                    self.ser2.write(command2.encode("utf-8"))
                    condition1 = False
                    condition2 = False
                    while True:
                        serial_response1 = self.ser1.read(self.ser1.in_waiting).decode().strip()
                        self.get_logger().info(f"first serial_response1: {serial_response1}")
                        if (("Closed" in serial_response1) or ("Opened" in serial_response1)):
                            condition1 = True
                            success_message1 = serial_response1
                        if ("Unknown" in serial_response1):
                            response.success = False
                            response.message = serial_response1 + f" for {self.tty1}"
                            return response
                        
                        serial_response2 = self.ser2.read(self.ser2.in_waiting).decode().strip()
                        self.get_logger().info(f"first serial_response2: {serial_response2}")
                        if (("Closed" in serial_response2) or ("Opened" in serial_response2)):
                            condition2 = True
                            success_message2 = serial_response2
                        if ("Unknown" in serial_response2):
                            response.success = False
                            response.message = serial_response2 + f" for {self.tty2}"
                            return response
                        if (condition1 and condition2):
                            break
                        t.sleep(0.25)

                    response.success = True
                    response.message = success_message1 + " and " + success_message2
                    return response
                except Exception as e:
                    response.success = False
                    response.message = str(e)
            else:
                response.success = False
                response.message = "The port to arduino could not be opened"
                return response




        

        
    
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





        
        



        