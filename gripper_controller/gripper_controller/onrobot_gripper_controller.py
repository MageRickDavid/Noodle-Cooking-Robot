#!/usr/bin/env python3
import sys
import time
import rclpy
import rclpy.duration
import rclpy.executors
import rclpy.logging
import time
from rclpy.node import Node
import rclpy.time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger

from gripper_controller.onrobot.onrobot_gripper_api import RG
from gripper_interfaces.srv import GripperCom

OPEN_CLOSE_COMMANDS = ['open', 'close']
class OnRobotGripperNode(Node):
    def __init__(self):
        super().__init__("onrobot_gripper_controller_node")
        self.gripper = "rg6"  # Default to rg6
        self.toolchanger_ip = "192.168.1.1"  # Default IP address
        self.toolchanger_port = 502  # Default port number

        self.rg = RG(gripper=self.gripper, ip=self.toolchanger_ip, port=self.toolchanger_port)
       
#
#
        self.call_back_groups_setup()
        self.service_setup()
        self.get_logger().info("onrobot gripper node fully setup!!")
        self.get_logger().info("exit 1")


    def call_back_groups_setup(self):
        self.gripper_control_group = MutuallyExclusiveCallbackGroup()

    def service_setup(self):
        self.gripper_control_service = self.create_service(GripperCom,"/gripper_controller/open_close",self.open_close,callback_group=self.gripper_control_group)
    
    def open_close(self, request, response):
       
        command = request.message
        self.get_logger().info(f"Gripper {command} was requested" )
        sys.stdout.flush()
        if not command in OPEN_CLOSE_COMMANDS:
            response.success  = False
            self.get_logger().error("Please only use either 'open' or 'close' ")
            return response
        
        if not self.rg.get_status()[0]:
            if command == 'open':
                self.rg.move_gripper(800)
            else:
                self.rg.close_gripper()
            while True:
                time.sleep(0.5)
                if not self.rg.get_status()[0]:
                    break
            response.success = True
            self.get_logger().info(f"Command {command} gripper completed")
            sys.stdout.flush()
            return response
                   
        else:
            response.success = False
            self.get_logger().error("A command was sent while the gripper was busy!!!")
            sys.stdout.flush()
            return response

def main():
    rclpy.init()
    gripperController = OnRobotGripperNode()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(gripperController, executor=executor)
    except KeyboardInterrupt:
        print('Shutting down')
        gripperController.rg.close_connection()
        gripperController.get_logger().error("ErrorGripper")

       

if __name__ == '__main__':
    main() 


