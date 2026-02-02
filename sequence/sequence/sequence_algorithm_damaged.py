import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import subprocess

from std_srvs.srv import Trigger

from gripper_interfaces.srv import ServoAbs
from gripper_interfaces.srv import GripperCom
from gripper_interfaces.srv import PusherMaster
from gripper_interfaces.srv import ServoAbsNice
from gripper_interfaces.srv import CartesianLocation
from gripper_interfaces.srv import RotatingServo
from gripper_interfaces.srv import ZaxisCommands
from gripper_interfaces.srv import RotatingServoTimeBased

import time  # Import time module for adding delays between movements
import subprocess
import logging
import aiohttp


trajectory_time = 1.0
locations = {
            "reset": (-120.0,0.0),
             "home": (-120.0,0.0),

             "H1":(-620.0,-70.0),
             "H1_back":(-420.0,-70.0),
             "H1_exit":(-420.0,-670.0),

             "dispenser4_back": (-120.0,-60.0), 
             "dispenser4": (-70.0,-60.0),
             "head4_back":(-420.0,-60.0),
             "head4": (-610.0,-60.0),

             "dispenser3":  (-120.0,-294.0),
             "head3_back":  (-420.0,-294.0),
             "head3":       (-620.0,-294.0),

             "dispenser2":(-120.0,-516.0),
             "head2_back":(-420.0,-516.0),
             "head2":     (-620.0,-516.0),

             "dispenser1":(-120.0,-737.0),
             "head2_back":(-420.0,-737.0),
             "head1":     (-620.0,-737.0),

             "exit":(-420.0,-750.0),

             "center1":(-370.0, -400.0),
             "middle2":(-370.0, -400.0)

}

angles = {
    "rotating_z_reset": 180.0,
    "rotating_z_facing_H1": 90.0,
    "rotating_z_facing_dispenser":270.0,

    "rotating_x_reset": 135.18,
    "rotating_x_drop": 315.18,
}

import aiohttp
import asyncio
import logging
import subprocess
import requests

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(message)s')
logger = logging.getLogger("FastClient")
class RobotClient:
    def __init__(self, base_url="http://172.30.1.52:8080"):
        self.base_url = base_url
        self.session = None
        self.lock = asyncio.Lock()
    async def __aenter__(self):
        if not self.session:
            # Configure client for best performance
            conn = aiohttp.TCPConnector(
                limit=0,  # No limit on concurrent connections
                ttl_dns_cache=300,  # Cache DNS results
                force_close=False  # Keep connections alive
            )
            timeout = aiohttp.ClientTimeout(total=30)
            self.session = aiohttp.ClientSession(
                connector=conn,
                timeout=timeout
            )
        return self
    async def __aexit__(self, exc_type, exc, tb):
        if self.session:
            await self.session.close()
            self.session = None
    async def send_command(self, command: str) -> bool:
        """Send command with minimal overhead"""
        try:
            async with self.lock:  # Ensure sequential execution
                async with self.session.get(f"{self.base_url}/{command}") as response:
                    return response.status == 200
        except Exception as e:
            logger.error(f"Error sending {command}: {str(e)}")
            return False
# Synchronous wrapper for compatibility
def send_http_request(command: str) -> bool:
    async def _send():
        async with RobotClient() as client:
            return await client.send_command(command)
    return asyncio.run(_send())
class CookerRobotSequence(Node):
    def __init__(self):
        super().__init__('CookerRobotSequenceAlgorithm')
        self.clients_and_callbacks_setup()
        self.service_setup()
        self.get_logger().info(f"cooker robot fully setup")

    def callbacks_for_services_of_sequence(self):
        # The callbacks describe start to end
        self.reset_dispenser1_callback = MutuallyExclusiveCallbackGroup()
        self.reset_dispenser2_callback = MutuallyExclusiveCallbackGroup()
        self.reset_dispenser3_callback = MutuallyExclusiveCallbackGroup()
        self.reset_dispenser4_callback = MutuallyExclusiveCallbackGroup()
        
        self.dispenser1_head1_callback = MutuallyExclusiveCallbackGroup()
        self.dispenser1_head2_callback = MutuallyExclusiveCallbackGroup()
        self.dispenser1_head3_callback = MutuallyExclusiveCallbackGroup()
        self.dispenser1_head4_callback = MutuallyExclusiveCallbackGroup()

        self.dispenser2_head1_callback = MutuallyExclusiveCallbackGroup()
        self.dispenser2_head2_callback = MutuallyExclusiveCallbackGroup()
        self.dispenser2_head3_callback = MutuallyExclusiveCallbackGroup()
        self.dispenser2_head4_callback = MutuallyExclusiveCallbackGroup()

        self.dispenser3_head1_callback = MutuallyExclusiveCallbackGroup()
        self.dispenser3_head2_callback = MutuallyExclusiveCallbackGroup()
        self.dispenser3_head3_callback = MutuallyExclusiveCallbackGroup()
        self.dispenser3_head4_callback = MutuallyExclusiveCallbackGroup()

        self.dispenser4_head1_callback = MutuallyExclusiveCallbackGroup()
        self.dispenser4_head2_callback = MutuallyExclusiveCallbackGroup()
        self.dispenser4_head3_callback = MutuallyExclusiveCallbackGroup()
        self.dispenser4_head4_callback = MutuallyExclusiveCallbackGroup()

        self.head1_exit_callback = MutuallyExclusiveCallbackGroup()
        self.head2_exit_callback = MutuallyExclusiveCallbackGroup()
        self.head3_exit_callback = MutuallyExclusiveCallbackGroup()
        self.head4_exit_callback = MutuallyExclusiveCallbackGroup()

        self.exit_head1_callback = MutuallyExclusiveCallbackGroup()
        self.exit_head2_callback = MutuallyExclusiveCallbackGroup()
        self.exit_head3_callback = MutuallyExclusiveCallbackGroup()
        self.exit_head4_callback = MutuallyExclusiveCallbackGroup()

        self.head1_reset_callback = MutuallyExclusiveCallbackGroup()
        self.head2_reset_callback = MutuallyExclusiveCallbackGroup()
        self.head3_reset_callback = MutuallyExclusiveCallbackGroup()
        self.head4_reset_callback = MutuallyExclusiveCallbackGroup()
        self.demo_callback = MutuallyExclusiveCallbackGroup()
        
    def callbacks_for_pick_bucket_sequence(self):
        self.pick_bucket_4_group =  MutuallyExclusiveCallbackGroup()
        self.pick_bucket_3_group =  MutuallyExclusiveCallbackGroup()
        self.pick_bucket_2_group =  MutuallyExclusiveCallbackGroup()
        self.pick_bucket_1_group =  MutuallyExclusiveCallbackGroup()

    def callbacks_for_place_bucket_sequence(self):
        self.place_bucket_4_group =  MutuallyExclusiveCallbackGroup()
        self.place_bucket_3_group =  MutuallyExclusiveCallbackGroup()
        self.place_bucket_2_group =  MutuallyExclusiveCallbackGroup()
        self.place_bucket_1_group =  MutuallyExclusiveCallbackGroup()
        self.test_group = MutuallyExclusiveCallbackGroup()
        

    def clients_and_callbacks_setup(self):        
        self.callback_servox = MutuallyExclusiveCallbackGroup()
        self.callback_servoy = MutuallyExclusiveCallbackGroup()
        self.callback_servox_homing = MutuallyExclusiveCallbackGroup()
        self.callback_servoy_homing = MutuallyExclusiveCallbackGroup()
        self.callback_rotating_servo_init_group = MutuallyExclusiveCallbackGroup()

        self.callback_servoz = MutuallyExclusiveCallbackGroup()

        self.callback_rotating_servoZ = MutuallyExclusiveCallbackGroup()
        self.callback_rotating_servoX = MutuallyExclusiveCallbackGroup()
        self.callback_rotating_servoZ_time = MutuallyExclusiveCallbackGroup()
        self.callback_rotating_servoX_time = MutuallyExclusiveCallbackGroup()
        
        self.callback_gripper_group = MutuallyExclusiveCallbackGroup()
        self.callback_pusher_group = MutuallyExclusiveCallbackGroup()

        self.algorithm_sequence_group = MutuallyExclusiveCallbackGroup()

        self.noodle_pick_the_basket_group = MutuallyExclusiveCallbackGroup()
        self.noodle_place_the_basket_group  = MutuallyExclusiveCallbackGroup()
        self.noodle_exit_the_basket_group = MutuallyExclusiveCallbackGroup()

        self.end_effector_init_group = MutuallyExclusiveCallbackGroup()
        self.callback_rotating_servoX_init_group = MutuallyExclusiveCallbackGroup()


        self.servo_x_client = self.create_client(ServoAbsNice, "/servo_controllerX/moveabsNice", callback_group=self.callback_servox)
        while not self.servo_x_client.wait_for_service(timeout_sec=20.0):
             self.get_logger().info('/servo_controllerX/moveabsNice not available, waiting again...')

        self.servo_y_client = self.create_client(ServoAbsNice, "/servo_controllerY/moveabsNice", callback_group=self.callback_servoy)
        while not self.servo_y_client.wait_for_service(timeout_sec=20.0):
             self.get_logger().info('/servo_controllerY/moveabsNice not available, waiting again...')
        
        # self.rotating_servoZ_client = self.create_client(RotatingServo, "/rotating_servoZ/command", callback_group=self.callback_rotating_servoZ)
        # while not self.rotating_servoZ_client.wait_for_service(timeout_sec=20.0):
        #     self.get_logger().info('/rotating_servoZ/command not available, waiting again...')

        # self.rotating_servoX_client = self.create_client(RotatingServo, "/rotating_servoX/command", callback_group=self.callback_rotating_servoX)
        # while not self.rotating_servoX_client.wait_for_service(timeout_sec=20.0):
        #     self.get_logger().info('/rotating_servoX/command not available, waiting again...')
        
        self.rotating_servoZ_time_based_client = self.create_client(RotatingServoTimeBased, "/rotating_servoZ/rotatingTimeBased", callback_group=self.callback_rotating_servoZ_time)
        while not self.rotating_servoZ_time_based_client.wait_for_service(timeout_sec=20.0):
             self.get_logger().info('/rotating_servoZ/rotatingTimeBased not available, waiting again...')

        self.rotating_servoX_time_based_client = self.create_client(RotatingServoTimeBased, "/rotating_servoX/rotatingTimeBased", callback_group=self.callback_rotating_servoX_time)
        while not self.rotating_servoX_time_based_client.wait_for_service(timeout_sec=20.0):
             self.get_logger().info('/rotating_servoX/rotatingTimeBased not available, waiting again...')

        self.gripper_client = self.create_client(GripperCom, "/gripper_controller/open_close", callback_group=self.callback_gripper_group)
        while not self.gripper_client.wait_for_service(timeout_sec=20.0):
             self.get_logger().info('/gripper_controller/open_close not available, waiting again...')

        self.pusher_client = self.create_client(PusherMaster, "/pusher/four/open_close", callback_group= self.callback_pusher_group)
        while not self.pusher_client.wait_for_service(timeout_sec=20.0):
            self.get_logger().info('/pusher/four/open_close not available, waiting again...')

        self.callbacks_for_services_of_sequence()
        self.callbacks_for_pick_bucket_sequence()
        self.callbacks_for_place_bucket_sequence()

    def services_sequence_setup(self):
        # The services describe start to end
        self.reset_dispenser1_srv = self.create_service(Trigger, "/cooker_robot/algorithm/reset_dispenser1", self.reset_dispenser1, callback_group=self.reset_dispenser1_callback)
        self.reset_dispenser2_srv = self.create_service(Trigger, "/cooker_robot/algorithm/reset_dispenser2", self.reset_dispenser2, callback_group=self.reset_dispenser2_callback)
        self.reset_dispenser3_srv = self.create_service(Trigger, "/cooker_robot/algorithm/reset_dispenser3", self.reset_dispenser3, callback_group=self.reset_dispenser3_callback)
        self.reset_dispenser4_srv = self.create_service(Trigger, "/cooker_robot/algorithm/reset_dispenser4", self.reset_dispenser4, callback_group=self.reset_dispenser4_callback)

        self.dispenser1_head1_srv = self.create_service(Trigger,"/cooker_robot/algorithm/dispenser1_head1", self.dispenser1_head1, callback_group=self.dispenser1_head1_callback)
        self.dispenser1_head2_srv = self.create_service(Trigger,"/cooker_robot/algorithm/dispenser1_head2", self.dispenser1_head2, callback_group=self.dispenser1_head2_callback)
        self.dispenser1_head3_srv = self.create_service(Trigger,"/cooker_robot/algorithm/dispenser1_head3", self.dispenser1_head3, callback_group=self.dispenser1_head3_callback)
        self.dispenser1_head4_srv = self.create_service(Trigger,"/cooker_robot/algorithm/dispenser1_head4", self.dispenser1_head4, callback_group=self.dispenser1_head4_callback)
# 
        self.dispenser2_head1_srv = self.create_service(Trigger,"/cooker_robot/algorithm/dispenser2_head1", self.dispenser2_head1, callback_group=self.dispenser2_head1_callback)
        self.dispenser2_head2_srv = self.create_service(Trigger,"/cooker_robot/algorithm/dispenser2_head2", self.dispenser2_head2, callback_group=self.dispenser2_head2_callback)
        self.dispenser2_head3_srv = self.create_service(Trigger,"/cooker_robot/algorithm/dispenser2_head3", self.dispenser2_head3, callback_group=self.dispenser2_head3_callback)
        self.dispenser2_head4_srv = self.create_service(Trigger,"/cooker_robot/algorithm/dispenser2_head4", self.dispenser2_head4, callback_group=self.dispenser2_head4_callback)
# 
        self.dispenser3_head1_srv = self.create_service(Trigger,"/cooker_robot/algorithm/dispenser3_head1", self.dispenser3_head1, callback_group=self.dispenser3_head1_callback)
        self.dispenser3_head2_srv = self.create_service(Trigger,"/cooker_robot/algorithm/dispenser3_head2", self.dispenser3_head2, callback_group=self.dispenser3_head2_callback)
        self.dispenser3_head3_srv = self.create_service(Trigger,"/cooker_robot/algorithm/dispenser3_head3", self.dispenser3_head3, callback_group=self.dispenser3_head3_callback)
        self.dispenser3_head4_srv = self.create_service(Trigger,"/cooker_robot/algorithm/dispenser3_head4", self.dispenser3_head4, callback_group=self.dispenser3_head4_callback)
# 
        self.dispenser4_head1_srv = self.create_service(Trigger,"/cooker_robot/algorithm/dispenser4_head1", self.dispenser4_head1, callback_group=self.dispenser4_head1_callback)
        self.dispenser4_head2_srv = self.create_service(Trigger,"/cooker_robot/algorithm/dispenser4_head2", self.dispenser4_head2, callback_group=self.dispenser4_head2_callback)
        self.dispenser4_head3_srv = self.create_service(Trigger,"/cooker_robot/algorithm/dispenser4_head3", self.dispenser4_head3, callback_group=self.dispenser4_head3_callback)
        self.dispenser4_head4_srv = self.create_service(Trigger,"/cooker_robot/algorithm/dispenser4_head4", self.dispenser4_head4, callback_group=self.dispenser4_head4_callback)

        self.head1_exit_srv = self.create_service(Trigger,"/cooker_robot/algorithm/head1_exit", self.head1_exit, callback_group=self.head1_exit_callback)
        self.head2_exit_srv = self.create_service(Trigger,"/cooker_robot/algorithm/head2_exit", self.head2_exit, callback_group=self.head2_exit_callback)
        self.head3_exit_srv = self.create_service(Trigger,"/cooker_robot/algorithm/head3_exit", self.head3_exit, callback_group=self.head3_exit_callback)
        self.head4_exit_srv = self.create_service(Trigger,"/cooker_robot/algorithm/head4_exit", self.head4_exit, callback_group=self.head4_exit_callback)
        
        self.exit_head1_srv = self.create_service(Trigger,"/cooker_robot/algorithm/exit_head1", self.exit_head1, callback_group=self.exit_head1_callback)
        self.exit_head2_srv = self.create_service(Trigger,"/cooker_robot/algorithm/exit_head2", self.exit_head2, callback_group=self.exit_head2_callback)
        self.exit_head3_srv = self.create_service(Trigger,"/cooker_robot/algorithm/exit_head3", self.exit_head3, callback_group=self.exit_head3_callback)
        self.exit_head4_srv = self.create_service(Trigger,"/cooker_robot/algorithm/exit_head4", self.exit_head4, callback_group=self.exit_head4_callback)
#       
        self.head1_reset_srv = self.create_service(Trigger,"/cooker_robot/algorithm/head1_reset", self.head1_reset, callback_group=self.head1_reset_callback)
        self.head2_reset_srv = self.create_service(Trigger,"/cooker_robot/algorithm/head2_reset", self.head2_reset, callback_group=self.head2_reset_callback)
        self.head3_reset_srv = self.create_service(Trigger,"/cooker_robot/algorithm/head3_reset", self.head3_reset, callback_group=self.head3_reset_callback)
        self.head4_reset_srv = self.create_service(Trigger,"/cooker_robot/algorithm/head4_reset", self.head4_reset, callback_group=self.head4_reset_callback)

        self.demo_srv = self.create_service(Trigger, "/cooker_robot/algorithm/demo", self.demo, callback_group=self.demo_callback)
        self.test_srv = self.create_service(Trigger, "/cooker/test", self.test, callback_group=self.test_group)

    def pick_bucket_services_setup(self):
        self.pick_bucket_4_srv = self.create_service(Trigger, '/cooker_robot/pick_bucket4', self.pick_bucket_4, callback_group=self.pick_bucket_4_group)    
        self.pick_bucket_3_srv = self.create_service(Trigger, '/cooker_robot/pick_bucket3', self.pick_bucket_3, callback_group=self.pick_bucket_3_group)    
        self.pick_bucket_2_srv = self.create_service(Trigger, '/cooker_robot/pick_bucket2', self.pick_bucket_2, callback_group=self.pick_bucket_2_group)    
        self.pick_bucket_1_srv = self.create_service(Trigger, '/cooker_robot/pick_bucket1', self.pick_bucket_1, callback_group=self.pick_bucket_1_group)    

    def place_bucket_services_setup(self):
        self.place_bucket_4_srv = self.create_service(Trigger, '/cooker_robot/place_bucket4', self.place_bucket_4, callback_group=self.place_bucket_4_group)    
        self.place_bucket_3_srv = self.create_service(Trigger, '/cooker_robot/place_bucket3', self.place_bucket_3, callback_group=self.place_bucket_3_group)    
        self.place_bucket_2_srv = self.create_service(Trigger, '/cooker_robot/place_bucket2', self.place_bucket_2, callback_group=self.place_bucket_2_group)    
        self.place_bucket_1_srv = self.create_service(Trigger, '/cooker_robot/place_bucket1', self.place_bucket_1, callback_group=self.place_bucket_1_group)    


        
    def service_setup(self):
        self.algorithm_sequence_srv     =  self.create_service(Trigger, "/cooker_robot/algorithm",           self.algorithm,            callback_group=self.algorithm_sequence_group)
        self.noodle_pick_the_basket_srv =  self.create_service(Trigger, "/cooker_robot/noodle_pick_basket",  self.pick_the_basket,      callback_group=self.noodle_pick_the_basket_group)
        self.noodle_place_the_place_srv =  self.create_service(Trigger, "/cooker_robot/noodle_place_basket", self.place_the_basket,     callback_group=self.noodle_place_the_basket_group)
        self.noodle_exit_the_basket_srv =  self.create_service(Trigger, "/cooker_robot/noodle_exit_basket",  self.exit_the_basket,      callback_group=self.noodle_exit_the_basket_group)
        self.end_effector_init_srv   =  self.create_service(Trigger, "/cooker_robot/end_effector_init",      self.end_effector_init, callback_group=self.end_effector_init_group)
        self.rotating_servoX_init_srv   =  self.create_service(Trigger, "/cooker_robot/rotatingX_init",      self.rotatingX_servo_init, callback_group=self.callback_rotating_servoX_init_group)


        self.services_sequence_setup()
        self.pick_bucket_services_setup()
        self.place_bucket_services_setup()
    


    async def reset_dispenser1(self, request, response):
        try:
            # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            # if location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move"
            #     return response
            
            location_result = await self.cartesian_location(*locations["dispenser1"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            response.success = True
            response.message = "Location sent"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    async def reset_dispenser2(self, request, response):
        try:
            # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            # if location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move"
            #     return response
            
            
            location_result = await self.cartesian_location(*locations["dispenser2"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            response.success = True
            response.message = "Location sent"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    async def reset_dispenser3(self, request, response):
        try:
            # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            # if location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move"
            #     return response
            
            location_result = await self.cartesian_location(*locations["dispenser3"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            response.success = True
            response.message = "Location sent"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    async def reset_dispenser4(self, request, response):
        try:
            # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            # if location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move"
            #     return response
            
            location_result = await self.cartesian_location(*locations["dispenser4"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            response.success = True
            response.message = "Location sent"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    
    async def dispenser1_head1(self, request, response):
        try:
            # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            # if location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move"
            #     return response
            
            location_result = await self.cartesian_location(*locations["dispenser1"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["head1"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            response.success = True
            response.message = "Location sent"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    async def dispenser1_head2(self, request, response):
        try:
            # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            # if location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move"
            #     return response
            
            location_result = await self.cartesian_location(*locations["dispenser1"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["head2"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            response.success = True
            response.message = "Location sent"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    async def dispenser1_head3(self, request, response):
        try:
            # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            # if location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move"
            #     return response
            
            location_result = await self.cartesian_location(*locations["dispenser1"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["head3"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            response.success = True
            response.message = "Location sent"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    async def dispenser1_head4(self, request, response):
        try:
            # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            # if location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move"
            #     return response
            
            location_result = await self.cartesian_location(*locations["dispenser1"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["head4"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            response.success = True
            response.message = "Location sent"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    
    async def dispenser2_head1(self, request, response):
        try:
            # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            # if location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move"
            #     return response
            
            location_result = await self.cartesian_location(*locations["dispenser2"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["head1"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            response.success = True
            response.message = "Location sent"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    async def dispenser2_head2(self, request, response):
        try:
            # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            # if location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move"
            #     return response
            
            location_result = await self.cartesian_location(*locations["dispenser2"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["head2"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            response.success = True
            response.message = "Location sent"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    async def dispenser2_head3(self, request, response):
        try:
            # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            # if location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move"
            #     return response
            
            location_result = await self.cartesian_location(*locations["dispenser2"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["head3"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            response.success = True
            response.message = "Location sent"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    async def dispenser2_head4(self, request, response):
        try:
            # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            # if location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move"
            #     return response
            
            location_result = await self.cartesian_location(*locations["dispenser2"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["head4"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            response.success = True
            response.message = "Location sent"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    async def dispenser3_head1(self, request, response):
        try:
            # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            # if location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move"
            #     return response
            
            location_result = await self.cartesian_location(*locations["dispenser3"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["head1"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            response.success = True
            response.message = "Location sent"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    async def dispenser3_head2(self, request, response):
        try:
            # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            # if location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move"
            #     return response
            
            location_result = await self.cartesian_location(*locations["dispenser3"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["head2"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            response.success = True
            response.message = "Location sent"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    async def dispenser3_head3(self, request, response):
        try:
            # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            # if location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move"
            #     return response
            
            location_result = await self.cartesian_location(*locations["dispenser3"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["head3"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            response.success = True
            response.message = "Location sent"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    async def dispenser3_head4(self, request, response):
        try:
            # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            # if location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move"
            #     return response
            
            location_result = await self.cartesian_location(*locations["dispenser3"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["head4"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            response.success = True
            response.message = "Location sent"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    async def dispenser4_head1(self, request, response):
        try:
            # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            # if location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move"
            #     return response
            
            location_result = await self.cartesian_location(*locations["dispenser4"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["head1"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            response.success = True
            response.message = "Location sent"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    async def dispenser4_head2(self, request, response):
        try:
            # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            # if location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move"
            #     return response
            
            location_result = await self.cartesian_location(*locations["dispenser4"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["head2"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            response.success = True
            response.message = "Location sent"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    async def dispenser4_head3(self, request, response):
        try:
            # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            # if location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move"
            #     return response
            
            location_result = await self.cartesian_location(*locations["dispenser4"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["head3"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            response.success = True
            response.message = "Location sent"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    async def dispenser4_head4(self, request, response):
        try:
            # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            # if location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move"
            #     return response
            
            location_result = await self.cartesian_location(*locations["dispenser4"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["head4"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            response.success = True
            response.message = "Location sent"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    async def head1_exit(self, request, response):
        try:
            # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            # if location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move"
            #     return response
            
            location_result = await self.cartesian_location(*locations["head1"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["exit"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            response.success = True
            response.message = "Location sent"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    async def head2_exit(self, request, response):
        try:
            # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            # if location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move"
            #     return response
            
            location_result = await self.cartesian_location(*locations["head2"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["exit"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            response.success = True
            response.message = "Location sent"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    async def head3_exit(self, request, response):
        try:
            # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            # if location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move"
            #     return response
            
            location_result = await self.cartesian_location(*locations["head3"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["exit"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            response.success = True
            response.message = "Location sent"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    async def head4_exit(self, request, response):
        try:
            # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            # if location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move"
            #     return response
            
            location_result = await self.cartesian_location(*locations["head4"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["exit"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            response.success = True
            response.message = "Location sent"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    async def exit_head1(self, request, response):
        try:
            # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            # if location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move"
            #     return response
            
            location_result = await self.cartesian_location(*locations["exit"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["head1"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            response.success = True
            response.message = "Location sent"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    async def exit_head2(self, request, response):
        try:
            # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            # if location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move"
            #     return response
            
            location_result = await self.cartesian_location(*locations["exit"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["head2"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            response.success = True
            response.message = "Location sent"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    async def exit_head3(self, request, response):
        try:
            # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            # if location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move"
            #     return response
            
            location_result = await self.cartesian_location(*locations["exit"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["head3"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            response.success = True
            response.message = "Location sent"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    async def exit_head4(self, request, response):
        try:
            # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            # if location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move"
            #     return response
            
            location_result = await self.cartesian_location(*locations["exit"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["head4"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            response.success = True
            response.message = "Location sent"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    async def head1_reset(self, request, response):
        try:
            # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            # if location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move"
            #     return response
            
            location_result = await self.cartesian_location(*locations["head1"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            response.success = True
            response.message = "Location sent"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    async def head2_reset(self, request, response):
        try:
            # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            # if location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move"
            #     return response
            
            location_result = await self.cartesian_location(*locations["head2"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            response.success = True
            response.message = "Location sent"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    async def head3_reset(self, request, response):
        try:
            # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            # if location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move"
            #     return response
            
            location_result = await self.cartesian_location(*locations["head3"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            response.success = True
            response.message = "Location sent"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    async def head4_reset(self, request, response):
        try:
            # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            # if location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move"
            #     return response
            
            location_result = await self.cartesian_location(*locations["head4"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response
            
            response.success = True
            response.message = "Location sent"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    async def algorithm(self, request, response):
        try:
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if location_result:
                response.success = True
                response.message = "Location sent"
            else:
                response.success = False
                response.message = "One or the two axis did not move"
            location_result = await self.cartesian_location(*locations["dispenser1"], time=trajectory_time)
            if location_result:
                response.success = True
                response.message = "Location sent"
            else:
                response.success = False
                response.message = "One or the two axis did not move"
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if location_result:
                response.success = True
                response.message = "Location sent"
            else:
                response.success = False
                response.message = "One or the two axis did not move"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    
    async def cartesian_location(self, x, y, time=trajectory_time):
        xLocationRequest = ServoAbsNice.Request()
        xLocationRequest.position = x
        xLocationRequest.time = time
        
        yLocationRequest = ServoAbsNice.Request()
        yLocationRequest.position = y
        yLocationRequest.time = time

        futurex = self.servo_x_client.call_async(xLocationRequest)
        futurey = self.servo_y_client.call_async(yLocationRequest)

        resultx = await futurex
        resulty = await futurey
        if (resulty.success and resultx.success):
            return True
        else:
            return False
    
    def rotate_servoZ_180degrees(self, direction):
        rotatingServoRequest = RotatingServo.Request()
        rotatingServoRequest.direction = direction
        rotatingServoRequest.angle = 180
        response = self.rotating_servoZ_client.call(rotatingServoRequest)
        return response.success
    
    def rotate_servoZ_90degrees(self, direction):
        rotatingServoRequest = RotatingServo.Request()
        rotatingServoRequest.direction = direction
        rotatingServoRequest.angle = 90
        response = self.rotating_servoZ_client.call(rotatingServoRequest)
        return response.success
    
    def rotate_servoX_180degrees(self, direction):
        rotatingServoRequest = RotatingServo.Request()
        rotatingServoRequest.direction = direction
        rotatingServoRequest.angle = 180
        response = self.rotating_servoX_client.call(rotatingServoRequest)
        return response.success
    
    def rotate_servoZ(self, angle):
        rotatingServoTimeBasedRequest = RotatingServoTimeBased.Request()
        rotatingServoTimeBasedRequest.angle = angle
        rotatingServoTimeBasedRequest.time = 2.0
        response = self.rotating_servoZ_time_based_client.call(rotatingServoTimeBasedRequest)
        return response.success
    
    def rotate_servoX(self, angle):
        rotatingServoTimeBasedRequest = RotatingServoTimeBased.Request()
        rotatingServoTimeBasedRequest.angle = angle
        rotatingServoTimeBasedRequest.time = 2.0
        response = self.rotating_servoX_time_based_client.call(rotatingServoTimeBasedRequest)
        return response.success

    def gripper_close(self):
        gripperRequest = GripperCom.Request()
        gripperRequest.message = 'close'
        response = self.gripper_client.call(gripperRequest)
        return response.success
    
    def gripper_open(self):
        gripperRequest = GripperCom.Request()
        gripperRequest.message = 'open'
        response = self.gripper_client.call(gripperRequest)
        return response.success
    
    def pusher_open(self):
        pusherRequest = PusherMaster.Request()
        pusherRequest.command = 'open'
        pusherRequest.pusher_id = 4
        response = self.pusher_client.call(pusherRequest)
        return response.success
    
    def pusher_close(self):
        pusherRequest = PusherMaster.Request()
        pusherRequest.command = 'close'
        pusherRequest.pusher_id = 4
        response = self.pusher_client.call(pusherRequest)
        return response.success
    
    def end_effector_init(self, request, response):
    
        # location_result = await self.cartesian_location(*locations["reset"], time=trajectory_time)
        # if  location_result != True:
        #      response.success = False
        #      response.message = "One or the two axis did not move, (step1)"
        #      return response


        rotatingServoXResponse = self.rotate_servoX(angle=angles["rotating_x_reset"]) 
        if rotatingServoXResponse != True:
            response.success = False
            response.message = "Something failed when rotating the servoZ, to its init position"
            return response

     
        rotatingServoZResponse = self.rotate_servoZ(angle=angles["rotating_z_reset"]) 
        if rotatingServoZResponse != True:
            response.success = False
            response.message = "Something failed when rotating the servoZ, to its init position"
            return response
        
        response.success = True
        response.message = "Init function finished"
        return response
    
    def rotatingX_servo_init(self, request, response):
                
        rotatingServoXResponse = self.rotate_servoX(angle=angles["rotating_x_reset"]) 
        if rotatingServoXResponse != True:
            response.success = False
            response.message = "Something failed when rotating the servoZ, to its init position"
            return response
        
        response.success = True
        response.message = "Init function finished"
        return response

    def test(self, request, response):
            success = send_http_request('z_init')
            self.get_logger().info(f"z init success: {success}")
            if not success:
                response.success = False
                response.message = "z init failed"
                return response
            
            success = send_http_request('lifter_1_init')
            self.get_logger().info(f"lifter 1 init init success: {success}")
            if not success:
                response.success = False
                response.message = "lifter 1 init failed"
                return response


    async def demo(self, request, response):
        try:
           # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
           # if location_result != True:
           #     response.success = False
           #     response.message = "One or the two axis did not move"
           #     return response
         while True:
            location_result = await self.cartesian_location(*locations["dispenser4"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response

            location_result = await self.cartesian_location(*locations["dispenser1"], time=trajectory_time)
            if  location_result != True:
               response.success = False
               response.message = "One or the two axis did not move"
               return response

            location_result = await self.cartesian_location(*locations["dispenser4"], time=trajectory_time)
            if  location_result != True:
               response.success = False
               response.message = "One or the two axis did not move"
               return response

            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if location_result != True:
                response.success = False
                response.message = "One or the two axis did not move"
                return response

            response.success = True
            response.message = "Location sent"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    
    async def pick_the_basket(self, request, response):
        try:
            #* SETP0_0
            location_result = await self.cartesian_location(*locations["reset"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response


            # #* SETP0_1
            # gripperResponse = self.gripper_open()
            # if gripperResponse != True:
            #     response.success = False
            #     response.message = "something failed when closing the gripper, (step-1)"
            #     return response
            
            #*SETP0_2
            rotatingServoZResponse = self.rotate_servoZ(angle=180.0) 
            if rotatingServoZResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            
            
            #* SETP1_0
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            
            #* SETP1_1
            rotatingServoZResponse = self.rotate_servoZ(angle=90.0) 
            if rotatingServoZResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            
            #* SETP2_0
            location_result = await self.cartesian_location(*locations["H1"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            #* SETP3_0
            rotatingServoZResponse = self.rotate_servoZ(angle=270.0) 
            if rotatingServoZResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            
            #* SETP3_1
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            


            # #* SETP0
            # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            # if  location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move, (step0)"
            #     return response

            # #* SETP1
            # location_result = await self.cartesian_location(*locations["dispenser4"], time=trajectory_time)
            # if  location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move, (step1)"
            #     return response
            
            # #* SETP2
            # rotatingServoZResponse = self.rotate_servoZ_180degrees(direction='CW') #rotating 180 counter-clock-wise
            # if rotatingServoZResponse != True:
            #     response.success = False
            #     response.message = "Something failed when rotating the servoZ, (step2)"
            #     return response
            
            # #* SETP3
            # location_result = await self.cartesian_location(*locations["head4_bucket"], time=trajectory_time)
            # if  location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move, (step3)"
            #     return response
            

            # #* SETP5
            # gripperResponse = self.gripper_close()
            # if gripperResponse != True:
            #     response.success = False
            #     response.message = "something failed when closing the gripper, (step5)"
            #     return response
            
            
            # #* SETP4
            # servoDriverZResponse = self.websocketZ_command("lifter_1_down")
            # if servoDriverZResponse != True:
            #     response.success = False
            #     response.message = "Something failed when moving the servo motor down short, (step4)"
            #     return response
            # time.sleep(3)

            #  #* SETP4
            # servoDriverZResponse = self.websocketZ_command("z_up_short")
            # if servoDriverZResponse != True:
            #     response.success = False
            #     response.message = "Something failed when moving the servo motor down short, (step4)"
            #     return response
            # time.sleep(3)


            # #* SETP5
            # gripperResponse = self.gripper_close()
            # if gripperResponse != True:
            #     response.success = False
            #     response.message = "something failed when closing the gripper, (step5)"
            #     return response
            
            # #* SETP4
            # servoDriverZResponse = self.websocketZ_command("z_up_short")
            # if servoDriverZResponse != True:
            #     response.success = False
            #     response.message = "Something failed when moving the servo motor up short, (step4)"
            #     return response

            # #* SETP7
            # rotatingServoZResponse = self.rotate_servoZ_180degrees(direction='CW') #rotating 180 clock-wise
            # if rotatingServoZResponse != True:
            #     response.success = False
            #     response.message = "something failed when rotating the servoZ, (step7)"
            #     return response
            
            # #* SETP8
            # location_result = await self.cartesian_location(*locations["dispenser4"], time=trajectory_time)
            # if  location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move, (step8)"
            #     return response
            
            # #* SETP9
            # servoDriverZResponse = self.websocketZ_command("z_down_long")
            # if servoDriverZResponse != True:
            #     response.success = False
            #     response.message = "Something failed when moving the servo motor down long, (step4)"
            #     return response

            # #* SETP10
            # pusherResponse = self.pusher_open()
            # if pusherResponse != True:
            #     response.success = False
            #     response.message = "something failed when opening the pusher, (step10)"
            #     return response
            
            # #* SETP11
            # pusherResponse = self.pusher_close()
            # if pusherResponse != True:
            #     response.success = False
            #     response.message = "something failed when closing the pusher, (step11)"
            #     return response
            
            # #* SETP12
            # servoDriverZResponse = self.websocketZ_command("z_up_long")
            # if servoDriverZResponse != True:
            #     response.success = False
            #     response.message = "Something failed when moving the servo motor up long, (step4)"
            #     return response

            # #* SETP13
            # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            # if  location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move, (step13)"
            #     return response
            
            # #* SETP14
            # gripperResponse = self.gripper_open()
            # if gripperResponse != True:
            #     response.success = False
            #     response.message = "something failed when closing the gripper, (step5)"
            #     return response

            response.success = True
            response.message = "Sequence finished"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    async def pick_bucket_4(self, request, response):
        try:


            #* SETP01
            gripperResponse = self.gripper_open()
            if gripperResponse != True:
                response.success = False
                response.message = "something failed when closing the gripper, (step-1)"
                return response
            

            #* SETP02
            location_result = await self.cartesian_location(*locations["reset"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            
            # send_http_request(http_commands['z_up'])
            # send_http_request(http_commands['z_up'])

            #*SETP03
            self.get_logger().info("I will move rotating servoX")
            rotatingServoXResponse = self.rotate_servoX(angle=135.18) 
            if rotatingServoXResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            self.get_logger().info("Finish rotating servoX")


            success = send_http_request('z_init')
            self.get_logger().info(f"z init success: {success}")
            if not success:
                response.success = False
                response.message = "z init failed"
                return response
            

            success = send_http_request('lifter_1_init')
            self.get_logger().info(f"lifter 1 init init success: {success}")
            if not success:
                response.success = False
                response.message = "lifter 1 init failed"
                return response
            

            #*SETP04
            self.get_logger().info("I will move rotating servoZ")
            rotatingServoZResponse = self.rotate_servoZ(angle=180.0) 
            if rotatingServoZResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            
            
            #* SETP05
            location_result = await self.cartesian_location(*locations["dispenser4_back"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            
            #* SETP06
            rotatingServoZResponse = self.rotate_servoZ(angle=90.0) 
            if rotatingServoZResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            
            # #* SETP7
            success = send_http_request('z_clearance')
            if not success:
                response.success = False
                response.message = "Z clearance failed"
                return response
            
            success = send_http_request('lifter_1_up')
            if not success:
                response.success = False
                response.message = "Lifter 1 up failed"
                return response


            #* SETP8
            location_result = await self.cartesian_location(*locations["head4_back"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            
            #* SETP9
            location_result = await self.cartesian_location(*locations["head4"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response

            success = send_http_request('z_up')
            if not success:
                response.success = False
                response.message = "Z up failed"
                return response

            #* SETP10
            gripperResponse = self.gripper_close()
            if gripperResponse != True:
                response.success = False
                response.message = "something failed when closing the gripper, (step-1)"
                return response
            
            #* SETP11
            success = send_http_request('lifter_1_down')
            if not success:
                response.success = False
                response.message = "lifter_1_down failed"
                return response
            
            success = send_http_request('z_clearance')
            if not success:
                response.success = False
                response.message = "z_clearance failed"
                return response

            #* SETP12
            location_result = await self.cartesian_location(*locations["head4_back"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response

            #* SETP13
            rotatingServoZResponse = self.rotate_servoZ(angle=270.0) 
            if rotatingServoZResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            
            #* SETP14
            location_result = await self.cartesian_location(*locations["dispenser4_back"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            
            #* SETP15
            success = send_http_request('z_down')
            if not success:
                response.success = False
                response.message = "z_down failed"
                return response
            
            #* SETP16
            location_result = await self.cartesian_location(*locations["dispenser4"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            

            #* SETP17
            pusherResponse = self.pusher_open()
            if pusherResponse != True:
                response.success = False
                response.message = "something failed when opening the pusher, (step10)"
                return response
            

            #* SETP18
            pusherResponse = self.pusher_close()
            if pusherResponse != True:
                response.success = False
                response.message = "something failed when closing the pusher, (step11)"
                return response


            #* SETP19
            location_result = await self.cartesian_location(*locations["dispenser4_back"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response

            success = send_http_request('z_clearance')
            if not success:
                response.success = False
                response.message = "z_clearance failed"
                return response


            #*SETP20
            self.get_logger().info("I will move rotating servoZ")
            rotatingServoZResponse = self.rotate_servoZ(angle=90.0) 
            if rotatingServoZResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            

            #* SETP21
            location_result = await self.cartesian_location(*locations["head4_back"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            


            #* SETP22
            location_result = await self.cartesian_location(*locations["head4"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            
            #* SETP23
            send_http_request('lifter_1_up')


            #* SETP23
            send_http_request('z_up')


            time.sleep(6)
            #* SETP24
            gripperResponse = self.gripper_open()
            if gripperResponse != True:
                response.success = False
                response.message = "something failed when closing the gripper, (step-1)"
                return response
            
            #* SETP25
            send_http_request('lifter_1_down')
            #time.sleep(15)
            send_http_request('lifter_1_shaking')
            #time.sleep(15)
            #time.sleep(20)

            send_http_request('z_clearance')
            #* SETP26
            location_result = await self.cartesian_location(*locations["head4_back"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
        
            #time.sleep(5)


            #* SETP27
            send_http_request('lifter_1_up')
        
            #time.sleep(7)


            #* SETP28
            location_result = await self.cartesian_location(*locations["head4"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response

            send_http_request(http_commands['z_up'])
            time.sleep(8)
            #* SETP29
            gripperResponse = self.gripper_close()
            if gripperResponse != True:
                response.success = False
                response.message = "something failed when closing the gripper, (step-1)"
                return response
        
           # time.sleep(7)


            #* SETP30
            send_http_request('lifter_1_down')

           # time.sleep(1)

            #* SETP30
            send_http_request('z_clearance')

           # time.sleep(5)
            #* SETP31
            location_result = await self.cartesian_location(*locations["head4_back"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
        
            #* SETP32
            location_result = await self.cartesian_location(*locations["exit"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
        
            #*SETP33
            rotatingServoXResponse = self.rotate_servoX(angle=315.18) 
            if rotatingServoXResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            

            #*SETP34
            rotatingServoXResponse = self.rotate_servoX(angle=135.18) 
            if rotatingServoXResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            

            #* SETP35
            location_result = await self.cartesian_location(*locations["head4_back"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
        

            #* SETP36
            location_result = await self.cartesian_location(*locations["head4"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response


            #* SETP30
            send_http_request('z_up')
            #time.sleep(8)
            #* SETP37
            send_http_request('lifter_1_up')

           # time.sleep(5)

            #* SETP38
            gripperResponse = self.gripper_open()
            if gripperResponse != True:
                response.success = False
                response.message = "something failed when closing the gripper, (step-1)"
                return response
            

            #* SETP39
            location_result = await self.cartesian_location(*locations["head4_back"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response




            response.success = True
            response.message = "Sequence finished"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    async def pick_bucket_3(self, request, response):
        try:
            #* SETP0_0
            location_result = await self.cartesian_location(*locations["reset"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response

            #*SETP0_1
            rotatingServoXResponse = self.rotate_servoX(angle=135.18) 
            if rotatingServoXResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response

            # #* SETP0_1
            # gripperResponse = self.gripper_open()
            # if gripperResponse != True:
            #     response.success = False
            #     response.message = "something failed when closing the gripper, (step-1)"
            #     return response
            
            #*SETP0_2
            rotatingServoZResponse = self.rotate_servoZ(angle=180.0) 
            if rotatingServoZResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            
            
            #* SETP1_0
            location_result = await self.cartesian_location(*locations["dispenser3"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            
            #* SETP1_1
            rotatingServoZResponse = self.rotate_servoZ(angle=90.0) 
            if rotatingServoZResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            
            #* SETP2_0
            location_result = await self.cartesian_location(*locations["head3"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            #* SETP3_0
            rotatingServoZResponse = self.rotate_servoZ(angle=270.0) 
            if rotatingServoZResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            
            #* SETP3_1
            location_result = await self.cartesian_location(*locations["dispenser3"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            


            response.success = True
            response.message = "Sequence finished"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    async def pick_bucket_2(self, request, response):
        try:
            #* SETP0_0
            location_result = await self.cartesian_location(*locations["reset"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            
            #*SETP0_1
            rotatingServoXResponse = self.rotate_servoX(angle=135.18) 
            if rotatingServoXResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response

            # #* SETP0_1
            # gripperResponse = self.gripper_open()
            # if gripperResponse != True:
            #     response.success = False
            #     response.message = "something failed when closing the gripper, (step-1)"
            #     return response
            
            #*SETP0_2
            rotatingServoZResponse = self.rotate_servoZ(angle=180.0) 
            if rotatingServoZResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            
            
            #* SETP1_0
            location_result = await self.cartesian_location(*locations["dispenser2"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            
            #* SETP1_1
            rotatingServoZResponse = self.rotate_servoZ(angle=90.0) 
            if rotatingServoZResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            
            #* SETP2_0
            location_result = await self.cartesian_location(*locations["head2"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            #* SETP3_0
            rotatingServoZResponse = self.rotate_servoZ(angle=270.0) 
            if rotatingServoZResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            
            #* SETP3_1
            location_result = await self.cartesian_location(*locations["dispenser2"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            


            response.success = True
            response.message = "Sequence finished"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    async def pick_bucket_1(self, request, response):
        try:
            #* SETP0_0
            location_result = await self.cartesian_location(*locations["reset"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response

            #*SETP0_1
            rotatingServoXResponse = self.rotate_servoX(angle=135.18) 
            if rotatingServoXResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            
            # #* SETP0_1
            # gripperResponse = self.gripper_open()
            # if gripperResponse != True:
            #     response.success = False
            #     response.message = "something failed when closing the gripper, (step-1)"
            #     return response
            
            #*SETP0_2
            rotatingServoZResponse = self.rotate_servoZ(angle=180.0) 
            if rotatingServoZResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            
            
            #* SETP1_0
            location_result = await self.cartesian_location(*locations["dispenser1"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            
            #* SETP1_1
            rotatingServoZResponse = self.rotate_servoZ(angle=90.0) 
            if rotatingServoZResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            
            #* SETP2_0
            location_result = await self.cartesian_location(*locations["head1"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            #* SETP3_0
            rotatingServoZResponse = self.rotate_servoZ(angle=270.0) 
            if rotatingServoZResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            
            #* SETP3_1
            location_result = await self.cartesian_location(*locations["dispenser1"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            


            response.success = True
            response.message = "Sequence finished"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response


    async def place_the_basket(self, request, response):
        try:
            #* SETP0_0
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            
            #*SETP0_1
            rotatingServoXResponse = self.rotate_servoX(angle=135.18) 
            if rotatingServoXResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            

            #*SETP0_2
            rotatingServoZResponse = self.rotate_servoZ(angle=270.0) 
            if rotatingServoZResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            
            
            #*SETP1_1
            rotatingServoZResponse = self.rotate_servoZ(angle=90.0) 
            if rotatingServoZResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            
            #* SETP1_2
            location_result = await self.cartesian_location(*locations["H1"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            
            #* SETP1_3
            location_result = await self.cartesian_location(*locations["H1_back"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            
            #* SETP1_4
            location_result = await self.cartesian_location(*locations["H1_exit"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            
            #*SETP0_1
            rotatingServoXResponse = self.rotate_servoX(angle=315.18) 
            if rotatingServoXResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            
            
            





            # #* SETP0
            # location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            # if  location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move, (step0)"
            #     return response

            # #* SETP1
            # location_result = await self.cartesian_location(*locations["dispenser4"], time=trajectory_time)
            # if  location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move, (step1)"
            #     return response
            
            # #* SETP2
            # rotatingServoZResponse = self.rotate_servoZ_180degrees(direction='CCW') #rotating 180 counter-clock-wise
            # if rotatingServoZResponse != True:
            #     response.success = False
            #     response.message = "Something failed when rotating the servoZ, (step2)"
            #     return response
            
            # #* SETP3
            # location_result = await self.cartesian_location(*locations["head4"], time=trajectory_time)
            # if  location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move, (step3)"
            #     return response
            
            # #* SETP4
            # servoDriverZResponse = self.websocketZ_command("z_down_short")
            # if servoDriverZResponse != True:
            #     response.success = False
            #     response.message = "Something failed when moving the servo motor down short, (step4)"
            #     return response


            # #* SETP5
            # gripperResponse = self.gripper_open()
            # if gripperResponse != True:
            #     response.success = False
            #     response.message = "something failed when opening the gripper, (step5)"
            #     return response
            
            # #* SETP4
            # servoDriverZResponse = self.websocketZ_command("z_up_short")
            # if servoDriverZResponse != True:
            #     response.success = False
            #     response.message = "Something failed when moving the servo motor up short, (step4)"
            #     return response

            # #* SETP7
            # rotatingServoZResponse = self.rotate_servoZ_180degrees(direction='CW') #rotating 180 clock-wise
            # if rotatingServoZResponse != True:
            #     response.success = False
            #     response.message = "something failed when rotating the servoZ, (step7)"
            #     return response
            
            # #* SETP8
            # location_result = await self.cartesian_location(*locations["dispenser4"], time=trajectory_time)
            # if  location_result != True:
            #     response.success = False
            #     response.message = "One or the two axis did not move, (step8)"
            #     return response
            
           
            response.success = True
            response.message = "Sequence finished"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    async def place_bucket_4(self, request, response):
        try:
            #* SETP0_0
            location_result = await self.cartesian_location(*locations["dispenser4"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            
            #*SETP0_1
            rotatingServoXResponse = self.rotate_servoX(angle=135.18) 
            if rotatingServoXResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            

            #*SETP0_2
            rotatingServoZResponse = self.rotate_servoZ(angle=270.0) 
            if rotatingServoZResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            
            
            #*SETP1_1
            rotatingServoZResponse = self.rotate_servoZ(angle=90.0) 
            if rotatingServoZResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            
            #* SETP1_2
            location_result = await self.cartesian_location(*locations["head4"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            
            #* SETP1_3
            location_result = await self.cartesian_location(*locations["head4_back"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            
            #* SETP1_4
            location_result = await self.cartesian_location(*locations["exit"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            
            #*SETP0_1
            rotatingServoXResponse = self.rotate_servoX(angle=315.18) 
            if rotatingServoXResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            
            
              
           
            response.success = True
            response.message = "Sequence finished"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    
    async def place_bucket_3(self, request, response):
        try:
            #* SETP0_0
            location_result = await self.cartesian_location(*locations["dispenser3"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            
            #*SETP0_1
            rotatingServoXResponse = self.rotate_servoX(angle=135.18) 
            if rotatingServoXResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            

            #*SETP0_2
            rotatingServoZResponse = self.rotate_servoZ(angle=270.0) 
            if rotatingServoZResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            
            
            #*SETP1_1
            rotatingServoZResponse = self.rotate_servoZ(angle=90.0) 
            if rotatingServoZResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            
            #* SETP1_2
            location_result = await self.cartesian_location(*locations["head3"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            
            #* SETP1_3
            location_result = await self.cartesian_location(*locations["head3_back"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            
            #* SETP1_4
            location_result = await self.cartesian_location(*locations["exit"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            
            #*SETP0_1
            rotatingServoXResponse = self.rotate_servoX(angle=315.18) 
            if rotatingServoXResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            
            
              
           
            response.success = True
            response.message = "Sequence finished"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    
    async def place_bucket_2(self, request, response):
        try:
            #* SETP0_0
            location_result = await self.cartesian_location(*locations["dispenser2"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            
            #*SETP0_1
            rotatingServoXResponse = self.rotate_servoX(angle=135.18) 
            if rotatingServoXResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            

            #*SETP0_2
            rotatingServoZResponse = self.rotate_servoZ(angle=270.0) 
            if rotatingServoZResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            
            
            #*SETP1_1
            rotatingServoZResponse = self.rotate_servoZ(angle=90.0) 
            if rotatingServoZResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            
            #* SETP1_2
            location_result = await self.cartesian_location(*locations["head2"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            
            #* SETP1_3
            location_result = await self.cartesian_location(*locations["head2_back"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            
            #* SETP1_4
            location_result = await self.cartesian_location(*locations["exit"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            
            #*SETP0_1
            rotatingServoXResponse = self.rotate_servoX(angle=315.18) 
            if rotatingServoXResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            
            
              
           
            response.success = True
            response.message = "Sequence finished"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    async def place_bucket_1(self, request, response):
        try:
            #* SETP0_0
            location_result = await self.cartesian_location(*locations["dispenser1"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            
            #*SETP0_1
            rotatingServoXResponse = self.rotate_servoX(angle=135.18) 
            if rotatingServoXResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            

            #*SETP0_2
            rotatingServoZResponse = self.rotate_servoZ(angle=270.0) 
            if rotatingServoZResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            
            
            #*SETP1_1
            rotatingServoZResponse = self.rotate_servoZ(angle=90.0) 
            if rotatingServoZResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            
            #* SETP1_2
            location_result = await self.cartesian_location(*locations["head1"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            
            #* SETP1_3
            location_result = await self.cartesian_location(*locations["head1_back"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            
            #* SETP1_4
            location_result = await self.cartesian_location(*locations["exit"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move, (step1)"
                 return response
            
            #*SETP0_1
            rotatingServoXResponse = self.rotate_servoX(angle=315.18) 
            if rotatingServoXResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            
            
              
           
            response.success = True
            response.message = "Sequence finished"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    async def exit_the_basket(self, request, response):
        try:
            #* SETP0
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move, (step0)"
                return response

            #* SETP1
            location_result = await self.cartesian_location(*locations["dispenser4"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move, (step1)"
                return response
            
            #* SETP2
            rotatingServoZResponse = self.rotate_servoZ_180degrees(direction='CCW') #rotating 180 counter-clock-wise
            if rotatingServoZResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            
            #* SETP3
            location_result = await self.cartesian_location(*locations["head4"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move, (step3)"
                return response
            
            #* SETP4
            servoDriverZResponse = self.websocketZ_command("z_down_short")
            if servoDriverZResponse != True:
                response.success = False
                response.message = "Something failed when moving the servo motor down short, (step4)"
                return response


            #* SETP5
            gripperResponse = self.gripper_close()
            if gripperResponse != True:
                response.success = False
                response.message = "something failed when closing the gripper, (step5)"
                return response
            
            #* SETP4
            servoDriverZResponse = self.websocketZ_command("z_up_short")
            if servoDriverZResponse != True:
                response.success = False
                response.message = "Something failed when moving the servo motor up short, (step4)"
                return response

            #* SETP8
            location_result = await self.cartesian_location(*locations["exit"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move, (step8)"
                return response
            
            #* SETP9
            rotatingServoXResponse = self.rotate_servoX_180degrees(direction='CCW') #rotating 180 counter-clock-wise
            if rotatingServoXResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoX, (step2)"
                return response
            
            #* SETP10
            rotatingServoXResponse = self.rotate_servoX_180degrees(direction='CW') #rotating 180 counter-clock-wise
            if rotatingServoXResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoX, (step2)"
                return response
            
            
            #* SETP8
            location_result = await self.cartesian_location(*locations["dispenser4"], time=trajectory_time)
            if  location_result != True:
                response.success = False
                response.message = "One or the two axis did not move, (step8)"
                return response
            
           
            response.success = True
            response.message = "Sequence finished"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response







def main():
    rclpy.init()
    cooker_robot_sequence = CookerRobotSequence()
    executor = MultiThreadedExecutor()
    rclpy.spin(cooker_robot_sequence, executor=executor)
    cooker_robot_sequence.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()  




