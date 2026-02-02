from jodellSdk.jodellSdkDemo import ClawControl
from jodellSdk.jodellSdkDemo import EpgClawControl
import time
import sys
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


from gripper_interfaces.srv import GripperCom
from gripper_interfaces.srv import GripperCheck

COMMANDS = {"open": (5, 0, 250, 250), "close": (5, 250, 250, 250)}

class GripperControlNode(Node):
    def __init__(self):
        super().__init__('gripper_controller_node')
        self.tty = "/dev/Gripper"
        self.initialize_claw_control()
        self.connect_serial()

        #TODO: Get feedback from gripper
        self.call_back_groups_setup()
        if self.gripper_setup():
            self.service_setup()
            self.get_logger().info("gripper node fully setup!!\n")
            sys.stdout.flush() # Gives the output to the console
            # Gripper ready status
            self.gripper_open = True
        else:
            self.get_logger().info("gripper node was not fully setup!!\n")
            sys.stdout.flush()
            # Gripper not ready status
            self.gripper_open = False




    def initialize_claw_control(self):
        # Initialize claw controls
        self.claw_control = ClawControl()
        self.epg_claw_control = EpgClawControl()
    
    
    def connect_serial(self):
       try:
           self.claw_control.serialOperation(self.tty, 115200, True)
       except Exception as e:
           self.get_logger().error(e)
           return False
    

    def call_back_groups_setup(self):
        self.gripper_control_group = MutuallyExclusiveCallbackGroup()
        self.gripper_check_group = MutuallyExclusiveCallbackGroup()


    def service_setup(self):
        self.gripper_control_service = self.create_service(GripperCom,"/gripper_controller/open_close",self.open_close,callback_group=self.gripper_control_group)
        self.gripper_check_service = self.create_service(GripperCheck, "/gripper_controller/check", self.gripper_check, callback_group=self.gripper_check_group)

    def gripper_setup(self):
        try:
            if self.claw_control.runWithParam(*COMMANDS['open']):
                self.get_logger().info("Claw is opening...")
                sys.stdout.flush()
            time.sleep(1)  # Wait for 1 second
            return True
        except Exception as e:
            self.get_logger().error(f'There was an error while setting up the grippe: {e}')
            sys.stdout.flush()
            return False

    
    def gripper_check(self, request, response):
        try:
            response.open = self.gripper_open
            response.success = True
        except Exception as e:
          self.get_logger().error(e)
          response.success = False
        return response 

    
    def open_close(self, request, response):
        try:
            command = request.message
            # Command status
            self.get_logger().info(f"Initiating {command} sequence...")  # Status update
            sys.stdout.flush()

            # Close the claw
            if self.claw_control.runWithParam(*COMMANDS[command]):
                self.get_logger().info(f"please wait, claw is about to {command}...")
                sys.stdout.flush()
                time.sleep(1)  # Wait for 1 second
                response.success = True
                # Success status
                self.gripper_open = (command == "open")
                return response
            else:
                self.get_logger().info(f"please wait, it was not possible to send command to gripper")
                sys.stdout.flush()
                response.success = False
                # Failure status
                return response
        except Exception as e:
          self.get_logger().error(e)
          response.success = False
          # Failure status
          return response




    






# # Initialize claw controls
# claw_control = ClawControl()
# epg_claw_control = EpgClawControl()

# # Set up serial communication
# # sudo chmod 666 /dev/ttyUSB0
# claw_control.serialOperation('/dev/Gripper', 115200, True)

# # Define parameters for open and close actions
# open_params = (5, 0, 250, 250)
# close_params = (5, 180, 250, 250)

# while True:
#     # Close the claw
#     if claw_control.runWithParam(*close_params):
#         print("Claw is closing...")
#     time.sleep(1)  # Wait for 5 seconds

# Open the claw
#       if claw_control.runWithParam(*open_params):
#           print("Claw is opening...")
#       time.sleep(1)  # Wait for 5 seconds


def main():
    rclpy.init()
    gripperController = GripperControlNode()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(gripperController, executor=executor)
    except KeyboardInterrupt:
        print('Shutting down')
        gripperController.destroy_node()
       

if __name__ == '__main__':
    main() 


