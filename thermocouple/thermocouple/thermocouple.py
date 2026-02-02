import serial
import rclpy
import rclpy.duration
import rclpy.executors
import rclpy.logging
import numpy as np
import time
from rclpy.node import Node
import rclpy.time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from gripper_interfaces.msg import Temperature

class Thermocouple(Node):
    def __init__(self):
        super().__init__('Thermocouple_node')
        self.tty = "/dev/Arduino"
        
        self.connect_serial()
        self.call_back_groups_setup()
        self.publisher_setup()
        self.timer_setup()
        time.sleep(2.5)
        self.get_logger().info("Thermocouple fully setup!!!!")
    
    
    def connect_serial(self):
        try: 
            self.ser = serial.Serial(port=self.tty, baudrate= 9600, timeout = 1)
            if self.ser.is_open:
                self.get_logger().info(f"The {self.tty} Port is Opened Now!")
                
            else:
                self.get_logger().info(f"The {self.tty} Port is Opened by another program")
        except Exception as e:
            self.get_logger().error(e)
            return False
    
    def call_back_groups_setup(self):
        self.thermocouple_publisher_group = MutuallyExclusiveCallbackGroup()
        self.thermocouple_timer_group =     MutuallyExclusiveCallbackGroup()

    def publisher_setup(self):
        try:
            self.temperature_publisher = self.create_publisher(Temperature, "/thermocouple_temperature" ,10,callback_group= self.thermocouple_publisher_group)
        except Exception as e:
            self.get_logger().error(str(e))
    
    def timer_setup(self):
        self.temperature_catcher = self.create_timer(timer_period_sec=0.5,callback=self.read_temperature, callback_group=self.thermocouple_timer_group)

    def check_connection(self):
        try:
            if self.ser.is_open:
                return True
            else:
                return False
        except:
            return False
        
    def read_temperature(self):
        receivedTemperature = self.ser.read(self.ser.in_waiting or 1).decode().split(",")
        #self.get_logger().info(f"message received: {temperature}")
        try: 
            #self.get_logger().info(f"processed_temperature: {receivedTemperature}")
            receivedTemperature[1] = receivedTemperature[1].split('\r\n')[0]
            
            #self.get_logger().info(f"processed_temperature: {receivedTemperature}")

            temperatureMsg = Temperature()
            try:
                temperatureMsg.temperature = [float(x) for x in receivedTemperature]
                self.temperature_publisher.publish(temperatureMsg)
            except:
                pass
        except Exception as e:
            self.get_logger().error(str(e))

def main():
    rclpy.init()
    thermocoupleNode = Thermocouple()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(thermocoupleNode, executor=executor)
    except KeyboardInterrupt:
        print('Shutting down')
        thermocoupleNode.ser.close()
       

if __name__ == '__main__':
    main()    




