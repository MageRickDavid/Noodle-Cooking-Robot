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
from gripper_interfaces.srv import PumpControl
from gripper_interfaces.srv import ServingNoodle
from gripper_interfaces.srv import PusherTimeMaster
from gripper_interfaces.srv import GrabExitBucket



import time  # Import time module for adding delays between movements
import subprocess


trajectory_time = 2.0
locations = {
            "reset": (0.0, 0.0),
             "home": (0.0,0.0),

            "head1":(859.26, 719.883),
            "head1_back": (686.532, 719.883),

            "head2":(859.26, 536.183),
            "head2_back": (686.532, 536.529),

            "head3":(859.26, 229.834),
            "head3_back":(686.532, 229.834),


            "head4":(859.26, 42.762),
            "head4_back": (686.532, 42.762),

            "inlet0": (686.532, 720.0),
                                    
            "inlet1": (930.000,730.917),
            "inlet2": (930.000, 730.963),

}

angles = {
    "rotating_z_reset": 180.0,
    "rotating_z_drop": 270.0,

    "rotating_x_reset": 270.0,
    "rotating_x_drop": 90.0,

    "inlet1x": 200.0,
    "inlet2x": 110.0,
    "inlet3x": 110.0,
}

http_commands = {
    "lifter_4_up"     :  "curl 192.168.150.15:8080/lifter_4_down",
    "lifter_4_down"   :  "curl 192.168.150.15:8080/lifter_4_up",
    "lifter_4_init"   :  "curl 192.168.150.15:8080/lifter_4_init",
    "lifter_4_shaking":  "curl 192.168.150.15:8080/lifter_4_shaking",
    "lifter_3_up"     :  "curl 192.168.150.15:8080/lifter_3_down",
    "lifter_3_down"   :  "curl 192.168.150.15:8080/lifter_3_up",
    "lifter_3_init"   :  "curl 192.168.150.15:8080/lifter_3_init",
    "lifter_3_shaking":  "curl 192.168.150.15:8080/lifter_3_shaking",
    "lifter_2_up"     :  "curl 192.168.150.15:8080/lifter_2_down",
    "lifter_2_down"   :  "curl 192.168.150.15:8080/lifter_2_up",
    "lifter_2_init"   :  "curl 192.168.150.15:8080/lifter_2_init",
    "lifter_2_shaking":  "curl 192.168.150.15:8080/lifter_2_shaking",
    "lifter_1_up"     :  "curl 192.168.150.15:8080/lifter_1_down",
    "lifter_1_down"   :  "curl 192.168.150.15:8080/lifter_1_up",
    "lifter_1_init"   :  "curl 192.168.150.15:8080/lifter_1_init",
    "lifter_1_shaking":  "curl 192.168.150.15:8080/lifter_1_shaking",
    "z_up"            :  "curl 192.168.150.15:8080/z_up",
    "z_down"          :  "curl 192.168.150.15:8080/z_down",
    "z_clearance"     :  "curl 192.168.150.15:8080/z_clearance",
    "z_init"          :  "curl 192.168.150.15:8080/z_init"

}
servings = {
    "one":1,
    "two":2,
    "three":3,
    "four":4
}



def send_http_request(command):
    subprocess.run(command, capture_output=True, text=True, shell=True)

class CookerRobotSequence(Node):
    def __init__(self):
        super().__init__('CookerRobotSequenceAlgorithm')
        self.clients_and_callbacks_setup()
        self.service_setup()
        self.get_logger().info(f"cooker robot fully setup")

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
        self.callback_rotating_servoX_command = MutuallyExclusiveCallbackGroup()
        self.callback_rotating_servoZ_command = MutuallyExclusiveCallbackGroup()
        
        self.callback_gripper_group = MutuallyExclusiveCallbackGroup()
        
        self.callback_pusher_group = MutuallyExclusiveCallbackGroup()
        self.callback_pusher_master_group = MutuallyExclusiveCallbackGroup()
       

        self.callback_pump_controller_group = MutuallyExclusiveCallbackGroup()

        self.end_effector_init_group = MutuallyExclusiveCallbackGroup()
        self.callback_rotating_servoX_init_group = MutuallyExclusiveCallbackGroup()

        self.sequence_service_group = MutuallyExclusiveCallbackGroup()
        self.serving_noodle_group = MutuallyExclusiveCallbackGroup()
        self.serving_noodle2_group = MutuallyExclusiveCallbackGroup()
        self.gantry_pick_up_group = MutuallyExclusiveCallbackGroup()
        self.dispense_noodle_group = MutuallyExclusiveCallbackGroup()

        self.servo_x_client = self.create_client(ServoAbsNice, "/servo_controllerX/moveabsNice", callback_group=self.callback_servox)
        while not self.servo_x_client.wait_for_service(timeout_sec=20.0):
             self.get_logger().info('/servo_controllerX/moveabsNice not available, waiting again...')

        self.servo_y_client = self.create_client(ServoAbsNice, "/servo_controllerY/moveabsNice", callback_group=self.callback_servoy)
        while not self.servo_y_client.wait_for_service(timeout_sec=20.0):
             self.get_logger().info('/servo_controllerY/moveabsNice not available, waiting again...')
        
        self.rotating_servoZ_time_based_client = self.create_client(RotatingServoTimeBased, "/rotating_servoZ/rotatingTimeBased", callback_group=self.callback_rotating_servoZ_time)
        while not self.rotating_servoZ_time_based_client.wait_for_service(timeout_sec=20.0):
             self.get_logger().info('/rotating_servoZ/rotatingTimeBased not available, waiting again...')

        self.rotating_servoX_time_based_client = self.create_client(RotatingServoTimeBased, "/rotating_servoX/rotatingTimeBased", callback_group=self.callback_rotating_servoX_time)
        while not self.rotating_servoX_time_based_client.wait_for_service(timeout_sec=20.0):
             self.get_logger().info('/rotating_servoX/rotatingTimeBased not available, waiting again...')

        
        # self.rotating_servoZ_client = self.create_client(RotatingServo, "/rotating_servoZ/command", callback_group=self.callback_rotating_servoZ_command)
        # while not self.rotating_servoZ_client.wait_for_service(timeout_sec=20.0):
        #     self.get_logger().info('/rotating_servoX/command not available, waiting again...')
        
        # self.rotating_servoX_client = self.create_client(RotatingServo, "/rotating_servoX/command", callback_group=self.callback_rotating_servoX_command)
        # while not self.rotating_servoX_client.wait_for_service(timeout_sec=20.0):
        #     self.get_logger().info('/rotating_servoX/command not available, waiting again...')
        
        self.gripper_client = self.create_client(GripperCom, "/gripper_controller/open_close", callback_group=self.callback_gripper_group)
        while not self.gripper_client.wait_for_service(timeout_sec=20.0):
             self.get_logger().info('/gripper_controller/open_close not available, waiting again...')


        self.pusher_client_master = self.create_client(PusherTimeMaster, "/pusher/open_close", callback_group=self.callback_pusher_master_group)
        while not self.pusher_client_master.wait_for_service(timeout_sec=20.0):
            self.get_logger().info('/pusher/open_close not available, waiting again...')

        # self.pump_client = self.create_client(PumpControl, "/pump_control", callback_group= self.callback_pump_controller_group)
        # while not self.pump_client.wait_for_service(timeout_sec=20.0):
        #     self.get_logger().info('/pump_control not available, waiting again...')
        
    def service_setup(self):
        self.end_effector_init_srv      =  self.create_service(Trigger, "/cooker_robot/end_effector_init",  self.end_effector_init,     callback_group  =   self.end_effector_init_group)
        self.rotating_servoX_init_srv   =  self.create_service(Trigger, "/cooker_robot/rotatingX_init",     self.rotatingX_servo_init,  callback_group  =   self.callback_rotating_servoX_init_group)
        self.sequence_algorithm_srv     =  self.create_service(Trigger, "/cooker_robot/one_serve",          self.one_serve,             callback_group  =   self.sequence_service_group)
        self.serving_noodle_srv         =  self.create_service(ServingNoodle, "/cooker_robot/serving",      self.serving_noodle,        callback_group  =   self.serving_noodle_group)
        self.serving_noodle2_srv        =  self.create_service(ServingNoodle, "/cooker_robot/serving2",       self.serving_noodle_2, callback_group= self.serving_noodle2_group)
        self.picking_up_noodle_srv      =  self.create_service(GrabExitBucket, "/cooker_robot/gantry_pickup", self.gantry_pick_up,   callback_group=self.gantry_pick_up_group)
        self.dispense_noodle_srv        =  self.create_service(ServingNoodle, "/Dispense", self.dispense_noodle, callback_group=self.dispense_noodle_group)

    def open_pusher_master(self, number_of_pushers):
        if number_of_pushers == 1:
            pusherResponse = self.pusher_master(1, 'open', timeToComplete=1675)
        elif number_of_pushers == 2:
            #pusherResponse = self.pusher_master(12, 'open', timeToComplete=1600)
            pusherResponse = self.pusher_master(1, 'open', timeToComplete=1675)
            pusherResponse = self.pusher_master(2, 'open', timeToComplete=1550)

        elif number_of_pushers == 3:
            #pusherResponse = self.pusher_master(123, 'open', timeToComplete=2000)
            pusherResponse = self.pusher_master(1, 'open', timeToComplete=1675)
            pusherResponse = self.pusher_master(2, 'open', timeToComplete=1550)
            pusherResponse = self.pusher_master(3, 'open', timeToComplete=1700)

        elif number_of_pushers == 4:
            #pusherResponse = self.pusher_master(1234, 'open',timeToComplete=2000)

            pusherResponse = self.pusher_master(1, 'open', timeToComplete=1675)
            pusherResponse = self.pusher_master(2, 'open', timeToComplete=1550)
            pusherResponse = self.pusher_master(3, 'open', timeToComplete=1700)
            pusherResponse = self.pusher_master(4, 'open', timeToComplete=1650)
        else:
            self.get_logger().error("pusher open master only accepts 1 - 4")
            return False
        return pusherResponse
    
    def close_pusher_master(self, number_of_pushers):
        if number_of_pushers == 1:
            pusherResponse = self.pusher_master(1, 'close', timeToComplete=2600)
        elif number_of_pushers == 2:
            #pusherResponse = self.pusher_master(12, 'close', timeToComplete=2600)
            pusherResponse = self.pusher_master(1, 'close', timeToComplete=2600)
            pusherResponse = self.pusher_master(2, 'close', timeToComplete=2600)

        elif number_of_pushers == 3:
            #pusherResponse = self.pusher_master(123, 'close')

            pusherResponse = self.pusher_master(1, 'close', timeToComplete=2600)
            pusherResponse = self.pusher_master(2, 'close', timeToComplete=2600)
            pusherResponse = self.pusher_master(3, 'close', timeToComplete=2600)
        elif number_of_pushers == 4:
            #pusherResponse = self.pusher_master(1234, 'close')

            pusherResponse = self.pusher_master(1, 'close', timeToComplete=2600)
            pusherResponse = self.pusher_master(2, 'close', timeToComplete=2600)
            pusherResponse = self.pusher_master(3, 'close', timeToComplete=2600)
            pusherResponse = self.pusher_master(4, 'close', timeToComplete=2600)

        else:
            self.get_logger().error("pusher close master only accepts 1 - 4")
            return False
        return pusherResponse

    def down_lift_master(self, number_of_lifters):
        try:
            if number_of_lifters == 1:
                send_http_request(http_commands['lifter_4_down'])
            elif number_of_lifters == 2:
                send_http_request(http_commands['lifter_4_down'])
                send_http_request(http_commands['lifter_3_down'])
            elif number_of_lifters == 3:
                send_http_request(http_commands['lifter_4_down'])
                send_http_request(http_commands['lifter_3_down'])
                send_http_request(http_commands['lifter_2_down'])
            elif number_of_lifters == 4:
                send_http_request(http_commands['lifter_4_down'])
                send_http_request(http_commands['lifter_3_down'])
                send_http_request(http_commands['lifter_2_down'])
                send_http_request(http_commands['lifter_1_down'])
            else: 
                self.get_logger().error("down lifter master only accepts 1 - 4")
                return False
            return True
        except Exception as e:
            self.get_logger().error(f"down lifter master says: {str(e)}")
            self.get_logger().error(str(e))
            return False

    def up_lift_master(self, number_of_lifters):
        try:
            if number_of_lifters == 1:
                send_http_request(http_commands['lifter_2_up'])
            elif number_of_lifters == 2:
                send_http_request(http_commands['lifter_4_up'])
                send_http_request(http_commands['lifter_3_up'])
            elif number_of_lifters == 3:
                send_http_request(http_commands['lifter_4_up'])
                send_http_request(http_commands['lifter_3_up'])
                send_http_request(http_commands['lifter_2_up'])
            elif number_of_lifters == 4:
                send_http_request(http_commands['lifter_4_up'])
                send_http_request(http_commands['lifter_3_up'])
                send_http_request(http_commands['lifter_2_up'])
                send_http_request(http_commands['lifter_1_up'])
            else: 
                self.get_logger().error("up lifter master only accepts 1 - 4")
                return False
            return True
        except Exception as e:
            self.get_logger().error(f"up lifter master says: {str(e)}")
            return False

    async def grab_to_exit_master(self, head):
        self.get_logger().info(f"Approaching to head_{head}_back")
        location_result = await self.cartesian_location(*locations[f"head{head}_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching to head_{head}_back")
            return location_result

        self.get_logger().info(f"Approaching to head {head}")
        location_result = await self.cartesian_location(*locations[f"head{head}"], time=trajectory_time)
        if  location_result != True:
             self.get_logger().error(f"grab from head {head} to exit failed when approaching to head {head}")
             return location_result
        
        #? This part can change, I think the lifters will be already up by this point!!!!!
        # try:
        #     send_http_request(http_commands[f'lifter_{head}_up'])
        #     self.get_logger().info(f"Waiting for lifter {head} to go up")
        # except Exception as e:
        #     self.get_logger().error(f"lifter {head} failed while going up")
        #     self.get_logger().error(f"lifter {head} says: {str(e)}")
        #     return False
        # time.sleep(7)
        #?###########################################################

        send_http_request(http_commands[f'lifter_{head}_up'])
        
        time.sleep(5)
        
        self.get_logger().info(f"Closing gripper!!!")
        gripperResponse = self.gripper_close()
        if gripperResponse != True:
            self.get_logger().error("something failed when closing the gripper")
            return gripperResponse
        
       

        
        # time.sleep(5)

        
        send_http_request(http_commands['z_up'])

        send_http_request(http_commands[f'lifter_{head}_down'])
        time.sleep(5)

        self.get_logger().info(f"Approaching to head_{head}_back")
        location_result = await self.cartesian_location(*locations[f"head{head}_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching to head_{head}_back")
            return location_result
        
        self.get_logger().info(f"Approaching to head_{head}_back")
        location_result = await self.cartesian_location(*locations[f"head2_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching to head_{head}_back")
            return location_result
        
        
        # self.get_logger().info(f"Z-Clearance")
        # location_result = await self.cartesian_location(*locations[f"z_clearance"], time=trajectory_time)
        # if  location_result != True:
        #     self.get_logger().error(f"grab from head {head} to exit failed when approaching to head_{head}_back")
        #     return location_result
        send_http_request(http_commands[f'z_clearance'])
        time.sleep(6)



        self.get_logger().info(f"rotating servoZ to drop position")
        rotatingServoZResponse = self.rotate_servoZ(angle=angles["rotating_z_drop"]) 
        if rotatingServoZResponse != True:
            self.get_logger().error("Something failed when rotating the servoZ, to drop position")
            return rotatingServoZResponse

        self.get_logger().info(f"Approaching to inlet0")
        location_result = await self.cartesian_location(*locations["inlet0"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching to inlet1")
            return location_result

    
        self.get_logger().info(f"rotating servoX to inlet1 position")
        rotatingServoXResponse = self.rotate_servoX(angle=angles["inlet1x"]) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet1 position")
            return rotatingServoXResponse


        self.get_logger().info(f"Approaching to inlet1")
        location_result = await self.cartesian_location(*locations["inlet1"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching to inlet1")
            return location_result

        self.get_logger().info(f"rotating servoX to inlet2 position")
        rotatingServoXResponse = self.rotate_servoX(angle=angles["inlet2x"]) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet2 position")
            return rotatingServoXResponse

        
        self.get_logger().info(f"Approaching to inlet2")
        location_result = await self.cartesian_location(*locations["inlet2"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching inlet2")
            return location_result
        
        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=angles["inlet3x"]) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse

        
        self.get_logger().info(f"Approaching to head_2_back")
        location_result = await self.cartesian_location(*locations[f"head2_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 2 to exit failed when approaching to head_2_back")
            return location_result
        
        


        self.get_logger().info(f"rotating servoX to reset position")
        rotatingServoXResponse = self.rotate_servoX(angle=angles["rotating_x_reset"]) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to reset position")
            return rotatingServoXResponse
        
        self.get_logger().info(f"rotating servoZ to reset position")
        rotatingServoXResponse = self.rotate_servoZ(angle=angles["rotating_z_reset"]) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoZ, to reset position")
            return rotatingServoXResponse
        



        # self.get_logger().info(f"Approaching to head_{head}_back and rotating servoZ to reset position")
        # movement_result = await self.rotating_translatingZ_movement(*locations[f"head{head}_back"],angle=angles["rotating_z_reset"], time=2.0)
        # if movement_result != True:
        #     self.get_logger().error("Something failed when rotating and translating at the same time")
        #     return movement_result




        self.get_logger().info(f"Approaching to head_{head}_back")
        location_result = await self.cartesian_location(*locations[f"head{head}_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching to head_{head}_back")
            return location_result
        

        
        


        send_http_request(http_commands[f'z_up'])
        time.sleep(6)


        self.get_logger().info(f"Approaching to head {head}")
        location_result = await self.cartesian_location(*locations[f"head{head}"], time=trajectory_time)
        if  location_result != True:
             self.get_logger().error(f"grab from head {head} to exit failed when approaching to head {head}")
             return location_result
        
       
        # send_http_request(http_commands['z_down'])
      
        # time.sleep(7)

       
        send_http_request(http_commands[f'lifter_{head}_up'])
          
        time.sleep(10)

        self.get_logger().info(f"Openning gripper!!!")
        gripperResponse = self.gripper_open()
        if gripperResponse != True:
            self.get_logger().error("something failed when opening the gripper")
            return gripperResponse
        time.sleep(2)

        self.get_logger().info(f"Approaching to head_{head}_back")
        location_result = await self.cartesian_location(*locations[f"head{head}_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching to head_{head}_back")
            return location_result
        
        return True
    
    # async def grab_to_exit_master_2(self, head):
        self.get_logger().info(f"Approaching to head_1_back")
        location_result = await self.cartesian_location(*locations[f"head1_back"], time=1.3)
        if  location_result != True:
            self.get_logger().error(f"grab from head 1 to exit failed when approaching to head_1_back")
            return location_result

        self.get_logger().info(f"Approaching to head 1")
        location_result = await self.cartesian_location(*locations[f"head1"], time=trajectory_time)
        if  location_result != True:
             self.get_logger().error(f"grab from head 1 to exit failed when approaching to head 1")
             return location_result
        
        #? This part can change, I think the lifters will be already up by this point!!!!!
        # try:
        #     send_http_request(http_commands[f'lifter_{head}_up'])
        #     self.get_logger().info(f"Waiting for lifter {head} to go up")
        # except Exception as e:
        #     self.get_logger().error(f"lifter {head} failed while going up")
        #     self.get_logger().error(f"lifter {head} says: {str(e)}")
        #     return False
        # time.sleep(7)
        #?###########################################################

        send_http_request(http_commands[f'lifter_1_up'])
        send_http_request(http_commands[f'lifter_2_up'])       
        time.sleep(3)
        
        self.get_logger().info(f"Closing gripper!!!")
        gripperResponse = self.gripper_close()
        if gripperResponse != True:
            self.get_logger().error("something failed when closing the gripper")
            return gripperResponse
        
       

        
        # time.sleep(5)

        
        send_http_request(http_commands['z_up'])

        send_http_request(http_commands[f'lifter_1_down'])
        time.sleep(4)

        self.get_logger().info(f"Approaching to head_1_back")
        location_result = await self.cartesian_location(*locations[f"head1_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 1 to exit failed when approaching to head_1_back")
            return location_result
        
        send_http_request(http_commands[f'z_clearance'])
        time.sleep(4)

        
        self.get_logger().info(f"Approaching to head_1_back")
        location_result = await self.cartesian_location(*locations[f"head2_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 1 to exit failed when approaching to head_1_back")
            return location_result
        
        
        # self.get_logger().info(f"Z-Clearance")
        # location_result = await self.cartesian_location(*locations[f"z_clearance"], time=trajectory_time)
        # if  location_result != True:
        #     self.get_logger().error(f"grab from head {head} to exit failed when approaching to head_{head}_back")
        #     return location_result



        self.get_logger().info(f"rotating servoZ to drop position")
        rotatingServoZResponse = self.rotate_servoZ(angle=angles["rotating_z_drop"]) 
        if rotatingServoZResponse != True:
            self.get_logger().error("Something failed when rotating the servoZ, to drop position")
            return rotatingServoZResponse

        self.get_logger().info(f"Approaching to inlet0")
        location_result = await self.cartesian_location(*locations["inlet0"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching to inlet1")
            return location_result

    
        self.get_logger().info(f"rotating servoX to inlet1 position")
        rotatingServoXResponse = self.rotate_servoX(angle=angles["inlet1x"]) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet1 position")
            return rotatingServoXResponse

        self.get_logger().info(f"Approaching to inlet0")
        location_result = await self.cartesian_location(*locations["inlet1"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching to inlet1")
            return location_result


        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=190.0) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse

        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=170.0) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse


        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=150.0) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse
   
        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=130.0) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse
     
        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=110.0) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse

        
        

        
        self.get_logger().info(f"Approaching to head_2_back")
        location_result = await self.cartesian_location(*locations[f"head2_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 2 to exit failed when approaching to head_2_back")
            return location_result
        
        


        self.get_logger().info(f"rotating servoX to reset position")
        rotatingServoXResponse = self.rotate_servoX(angle=angles["rotating_x_reset"]) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to reset position")
            return rotatingServoXResponse
        
        self.get_logger().info(f"rotating servoZ to reset position")
        rotatingServoXResponse = self.rotate_servoZ(angle=angles["rotating_z_reset"]) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoZ, to reset position")
            return rotatingServoXResponse
        



        # self.get_logger().info(f"Approaching to head_{head}_back and rotating servoZ to reset position")
        # movement_result = await self.rotating_translatingZ_movement(*locations[f"head{head}_back"],angle=angles["rotating_z_reset"], time=2.0)
        # if movement_result != True:
        #     self.get_logger().error("Something failed when rotating and translating at the same time")
        #     return movement_result




        self.get_logger().info(f"Approaching to head_1_back")
        location_result = await self.cartesian_location(*locations[f"head1_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 1 to exit failed when approaching to head_1_back")
            return location_result
        

        
        


        send_http_request(http_commands[f'z_up'])
        time.sleep(6)


        self.get_logger().info(f"Approaching to head 1")
        location_result = await self.cartesian_location(*locations[f"head1"], time=trajectory_time)
        if  location_result != True:
             self.get_logger().error(f"grab from head 1 to exit failed when approaching to head 1")
             return location_result
        
       
        # send_http_request(http_commands['z_down'])
      
        # time.sleep(7)

       
        send_http_request(http_commands[f'lifter_1_up'])
        send_http_request(http_commands[f'z_down'])  
        time.sleep(5)

        self.get_logger().info(f"Openning gripper!!!")
        gripperResponse = self.gripper_open()
        if gripperResponse != True:
            self.get_logger().error("something failed when opening the gripper")
            return gripperResponse
        time.sleep(2)

        self.get_logger().info(f"Approaching to head_1_back")
        location_result = await self.cartesian_location(*locations[f"head{head}_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching to head_{head}_back")
            return location_result
        
        self.get_logger().info(f"Approaching to head_2_back")
        location_result = await self.cartesian_location(*locations[f"head2_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 1 to exit failed when approaching to head_1_back")
            return location_result

        self.get_logger().info(f"Approaching to head 2")
        location_result = await self.cartesian_location(*locations[f"head2"], time=trajectory_time)
        if  location_result != True:
             self.get_logger().error(f"grab from head 2 to exit failed when approaching to head 1")
             return location_result
        
        #? This part can change, I think the lifters will be already up by this point!!!!!
        # try:
        #     send_http_request(http_commands[f'lifter_{head}_up'])
        #     self.get_logger().info(f"Waiting for lifter {head} to go up")
        # except Exception as e:
        #     self.get_logger().error(f"lifter {head} failed while going up")
        #     self.get_logger().error(f"lifter {head} says: {str(e)}")
        #     return False
        # time.sleep(7)
        #?###########################################################


        
        time.sleep(3)
        
        self.get_logger().info(f"Closing gripper!!!")
        gripperResponse = self.gripper_close()
        if gripperResponse != True:
            self.get_logger().error("something failed when closing the gripper")
            return gripperResponse
        
       

        
        # time.sleep(5)

        
        send_http_request(http_commands['z_up'])

        send_http_request(http_commands[f'lifter_2_down'])
        time.sleep(3)

        self.get_logger().info(f"Approaching to head_2_back")
        location_result = await self.cartesian_location(*locations[f"head2_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 2 to exit failed when approaching to head_1_back")
            return location_result
        
        self.get_logger().info(f"Approaching to head_2_back")
        location_result = await self.cartesian_location(*locations[f"head2_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 1 to exit failed when approaching to head_1_back")
            return location_result
        
        
        # self.get_logger().info(f"Z-Clearance")
        # location_result = await self.cartesian_location(*locations[f"z_clearance"], time=trajectory_time)
        # if  location_result != True:
        #     self.get_logger().error(f"grab from head {head} to exit failed when approaching to head_{head}_back")
        #     return location_result
        send_http_request(http_commands[f'z_clearance'])
        time.sleep(4)



        self.get_logger().info(f"rotating servoZ to drop position")
        rotatingServoZResponse = self.rotate_servoZ(angle=angles["rotating_z_drop"]) 
        if rotatingServoZResponse != True:
            self.get_logger().error("Something failed when rotating the servoZ, to drop position")
            return rotatingServoZResponse

        self.get_logger().info(f"Approaching to inlet0")
        location_result = await self.cartesian_location(*locations["inlet0"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching to inlet1")
            return location_result

    
        self.get_logger().info(f"rotating servoX to inlet1 position")
        rotatingServoXResponse = self.rotate_servoX(angle=angles["inlet1x"]) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet1 position")
            return rotatingServoXResponse


        self.get_logger().info(f"Approaching to inlet1")
        location_result = await self.cartesian_location(*locations["inlet1"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching to inlet1")
            return location_result

        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=190.0) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse

        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=170.0) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse


        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=150.0) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse
    
        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=130.0) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse
        
 
        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=110.0) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse

        
        self.get_logger().info(f"Approaching to head_2_back")
        location_result = await self.cartesian_location(*locations[f"head2_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 2 to exit failed when approaching to head_2_back")
            return location_result
        
        


        self.get_logger().info(f"rotating servoX to reset position")
        rotatingServoXResponse = self.rotate_servoX(angle=angles["rotating_x_reset"]) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to reset position")
            return rotatingServoXResponse
        
        self.get_logger().info(f"rotating servoZ to reset position")
        rotatingServoXResponse = self.rotate_servoZ(angle=angles["rotating_z_reset"]) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoZ, to reset position")
            return rotatingServoXResponse
        



        # self.get_logger().info(f"Approaching to head_{head}_back and rotating servoZ to reset position")
        # movement_result = await self.rotating_translatingZ_movement(*locations[f"head{head}_back"],angle=angles["rotating_z_reset"], time=2.0)
        # if movement_result != True:
        #     self.get_logger().error("Something failed when rotating and translating at the same time")
        #     return movement_result




        self.get_logger().info(f"Approaching to head_2_back")
        location_result = await self.cartesian_location(*locations[f"head2_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 1 to exit failed when approaching to head_1_back")
            return location_result
        

        
        


        send_http_request(http_commands[f'z_up'])
        time.sleep(4)


        self.get_logger().info(f"Approaching to head 1")
        location_result = await self.cartesian_location(*locations[f"head2"], time=trajectory_time)
        if  location_result != True:
             self.get_logger().error(f"grab from head 1 to exit failed when approaching to head 1")
             return location_result
        
       
        # send_http_request(http_commands['z_down'])
      
        # time.sleep(7)



        send_http_request(http_commands[f'lifter_2_up'])
        send_http_request(http_commands[f'z_down'])
          
        time.sleep(5)

        self.get_logger().info(f"Openning gripper!!!")
        gripperResponse = self.gripper_open()
        if gripperResponse != True:
            self.get_logger().error("something failed when opening the gripper")
            return gripperResponse
        time.sleep(2)

        self.get_logger().info(f"Approaching to head_2_back")
        location_result = await self.cartesian_location(*locations[f"head2_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 2 to exit failed when approaching to head_2_back")
            return location_result
        
        return True
    async def grab_to_exit_master_2(self, head):
        self.get_logger().info(f"Approaching to head_1_back")
        location_result = await self.cartesian_location(*locations[f"head1_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 1 to exit failed when approaching to head_1_back")
            return location_result

        self.get_logger().info(f"Approaching to head 1")
        location_result = await self.cartesian_location(*locations[f"head1"], time=trajectory_time)
        if  location_result != True:
             self.get_logger().error(f"grab from head 1 to exit failed when approaching to head 1")
             return location_result
        
        #? This part can change, I think the lifters will be already up by this point!!!!!
        # try:
        #     send_http_request(http_commands[f'lifter_{head}_up'])
        #     self.get_logger().info(f"Waiting for lifter {head} to go up")
        # except Exception as e:
        #     self.get_logger().error(f"lifter {head} failed while going up")
        #     self.get_logger().error(f"lifter {head} says: {str(e)}")
        #     return False
        # time.sleep(7)
        #?###########################################################

        send_http_request(http_commands[f'lifter_1_up'])
        time.sleep(1)
        send_http_request(http_commands[f'lifter_2_up'])  
        time.sleep(1)  
        send_http_request(http_commands[f'lifter_3_up'])    
        time.sleep(1)
        send_http_request(http_commands[f'lifter_4_up'])       
        time.sleep(3)
        
        self.get_logger().info(f"Closing gripper!!!")
        gripperResponse = self.gripper_close()
        if gripperResponse != True:
            self.get_logger().error("something failed when closing the gripper")
            return gripperResponse
        
       

        
        # time.sleep(5)

        
        send_http_request(http_commands['z_up'])

        send_http_request(http_commands[f'lifter_1_down'])
        time.sleep(4)

        self.get_logger().info(f"Approaching to head_1_back")
        location_result = await self.cartesian_location(*locations[f"head1_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 1 to exit failed when approaching to head_1_back")
            return location_result
        
        send_http_request(http_commands[f'z_clearance'])
        time.sleep(4)

        
        self.get_logger().info(f"Approaching to head_1_back")
        location_result = await self.cartesian_location(*locations[f"head2_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 1 to exit failed when approaching to head_1_back")
            return location_result
        
        
        # self.get_logger().info(f"Z-Clearance")
        # location_result = await self.cartesian_location(*locations[f"z_clearance"], time=trajectory_time)
        # if  location_result != True:
        #     self.get_logger().error(f"grab from head {head} to exit failed when approaching to head_{head}_back")
        #     return location_result



        self.get_logger().info(f"rotating servoZ to drop position")
        rotatingServoZResponse = self.rotate_servoZ(angle=angles["rotating_z_drop"]) 
        if rotatingServoZResponse != True:
            self.get_logger().error("Something failed when rotating the servoZ, to drop position")
            return rotatingServoZResponse

        self.get_logger().info(f"Approaching to inlet0")
        location_result = await self.cartesian_location(*locations["inlet0"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching to inlet1")
            return location_result

    
        self.get_logger().info(f"rotating servoX to inlet1 position")
        rotatingServoXResponse = self.rotate_servoX(angle=angles["inlet1x"]) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet1 position")
            return rotatingServoXResponse

        self.get_logger().info(f"Approaching to inlet0")
        location_result = await self.cartesian_location(*locations["inlet1"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching to inlet1")
            return location_result


        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=190.0) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse

        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=170.0) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse


        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=150.0) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse
   
        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=130.0) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse
     
        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=110.0) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse

        
        

        
        self.get_logger().info(f"Approaching to head_2_back")
        location_result = await self.cartesian_location(*locations[f"head2_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 2 to exit failed when approaching to head_2_back")
            return location_result
        
        


        self.get_logger().info(f"rotating servoX to reset position")
        rotatingServoXResponse = self.rotate_servoX(angle=angles["rotating_x_reset"]) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to reset position")
            return rotatingServoXResponse
        
        self.get_logger().info(f"rotating servoZ to reset position")
        rotatingServoXResponse = self.rotate_servoZ(angle=angles["rotating_z_reset"]) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoZ, to reset position")
            return rotatingServoXResponse
        



        # self.get_logger().info(f"Approaching to head_{head}_back and rotating servoZ to reset position")
        # movement_result = await self.rotating_translatingZ_movement(*locations[f"head{head}_back"],angle=angles["rotating_z_reset"], time=2.0)
        # if movement_result != True:
        #     self.get_logger().error("Something failed when rotating and translating at the same time")
        #     return movement_result




        self.get_logger().info(f"Approaching to head_1_back")
        location_result = await self.cartesian_location(*locations[f"head1_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 1 to exit failed when approaching to head_1_back")
            return location_result
        

        
        


        send_http_request(http_commands[f'z_up'])
        time.sleep(6)


        self.get_logger().info(f"Approaching to head 1")
        location_result = await self.cartesian_location(*locations[f"head1"], time=trajectory_time)
        if  location_result != True:
             self.get_logger().error(f"grab from head 1 to exit failed when approaching to head 1")
             return location_result
        
       
        # send_http_request(http_commands['z_down'])
      
        # time.sleep(7)

       
        send_http_request(http_commands[f'lifter_1_up'])
        send_http_request(http_commands[f'z_down'])  
        time.sleep(5)

        self.get_logger().info(f"Openning gripper!!!")
        gripperResponse = self.gripper_open()
        if gripperResponse != True:
            self.get_logger().error("something failed when opening the gripper")
            return gripperResponse
        time.sleep(2)

        self.get_logger().info(f"Approaching to head_1_back")
        location_result = await self.cartesian_location(*locations[f"head{head}_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching to head_{head}_back")
            return location_result
        
        self.get_logger().info(f"Approaching to head_2_back")
        location_result = await self.cartesian_location(*locations[f"head2_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 1 to exit failed when approaching to head_1_back")
            return location_result

        self.get_logger().info(f"Approaching to head 2")
        location_result = await self.cartesian_location(*locations[f"head2"], time=trajectory_time)
        if  location_result != True:
             self.get_logger().error(f"grab from head 2 to exit failed when approaching to head 1")
             return location_result
        
        #? This part can change, I think the lifters will be already up by this point!!!!!
        # try:
        #     send_http_request(http_commands[f'lifter_{head}_up'])
        #     self.get_logger().info(f"Waiting for lifter {head} to go up")
        # except Exception as e:
        #     self.get_logger().error(f"lifter {head} failed while going up")
        #     self.get_logger().error(f"lifter {head} says: {str(e)}")
        #     return False
        # time.sleep(7)
        #?###########################################################


        
        time.sleep(3)
        
        self.get_logger().info(f"Closing gripper!!!")
        gripperResponse = self.gripper_close()
        if gripperResponse != True:
            self.get_logger().error("something failed when closing the gripper")
            return gripperResponse
        
       

        
        # time.sleep(5)

        
        send_http_request(http_commands['z_up'])

        send_http_request(http_commands[f'lifter_2_down'])
        time.sleep(3)

        self.get_logger().info(f"Approaching to head_2_back")
        location_result = await self.cartesian_location(*locations[f"head2_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 2 to exit failed when approaching to head_1_back")
            return location_result
        
        self.get_logger().info(f"Approaching to head_2_back")
        location_result = await self.cartesian_location(*locations[f"head2_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 1 to exit failed when approaching to head_1_back")
            return location_result
        
        
        # self.get_logger().info(f"Z-Clearance")
        # location_result = await self.cartesian_location(*locations[f"z_clearance"], time=trajectory_time)
        # if  location_result != True:
        #     self.get_logger().error(f"grab from head {head} to exit failed when approaching to head_{head}_back")
        #     return location_result
        send_http_request(http_commands[f'z_clearance'])
        time.sleep(4)



        self.get_logger().info(f"rotating servoZ to drop position")
        rotatingServoZResponse = self.rotate_servoZ(angle=angles["rotating_z_drop"]) 
        if rotatingServoZResponse != True:
            self.get_logger().error("Something failed when rotating the servoZ, to drop position")
            return rotatingServoZResponse

        self.get_logger().info(f"Approaching to inlet0")
        location_result = await self.cartesian_location(*locations["inlet0"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching to inlet1")
            return location_result

    
        self.get_logger().info(f"rotating servoX to inlet1 position")
        rotatingServoXResponse = self.rotate_servoX(angle=angles["inlet1x"]) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet1 position")
            return rotatingServoXResponse


        self.get_logger().info(f"Approaching to inlet1")
        location_result = await self.cartesian_location(*locations["inlet1"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching to inlet1")
            return location_result

        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=190.0) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse

        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=170.0) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse


        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=150.0) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse
    
        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=130.0) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse
        
 
        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=110.0) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse

        
        self.get_logger().info(f"Approaching to head_2_back")
        location_result = await self.cartesian_location(*locations[f"head2_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 2 to exit failed when approaching to head_2_back")
            return location_result
        
        


        self.get_logger().info(f"rotating servoX to reset position")
        rotatingServoXResponse = self.rotate_servoX(angle=angles["rotating_x_reset"]) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to reset position")
            return rotatingServoXResponse
        
        self.get_logger().info(f"rotating servoZ to reset position")
        rotatingServoXResponse = self.rotate_servoZ(angle=angles["rotating_z_reset"]) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoZ, to reset position")
            return rotatingServoXResponse
        



        # self.get_logger().info(f"Approaching to head_{head}_back and rotating servoZ to reset position")
        # movement_result = await self.rotating_translatingZ_movement(*locations[f"head{head}_back"],angle=angles["rotating_z_reset"], time=2.0)
        # if movement_result != True:
        #     self.get_logger().error("Something failed when rotating and translating at the same time")
        #     return movement_result




        self.get_logger().info(f"Approaching to head_2_back")
        location_result = await self.cartesian_location(*locations[f"head2_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 1 to exit failed when approaching to head_1_back")
            return location_result
        

        
        


        send_http_request(http_commands[f'z_up'])
        time.sleep(4)


        self.get_logger().info(f"Approaching to head 1")
        location_result = await self.cartesian_location(*locations[f"head2"], time=trajectory_time)
        if  location_result != True:
             self.get_logger().error(f"grab from head 1 to exit failed when approaching to head 1")
             return location_result
        
       
        # send_http_request(http_commands['z_down'])
      
        # time.sleep(7)



        send_http_request(http_commands[f'lifter_2_up'])
        send_http_request(http_commands[f'z_down'])
          
        time.sleep(5)

        self.get_logger().info(f"Openning gripper!!!")
        gripperResponse = self.gripper_open()
        if gripperResponse != True:
            self.get_logger().error("something failed when opening the gripper")
            return gripperResponse
        time.sleep(2)

        self.get_logger().info(f"Approaching to head_2_back")
        location_result = await self.cartesian_location(*locations[f"head2_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 2 to exit failed when approaching to head_2_back")
            return location_result
        


# 




        self.get_logger().info(f"Approaching to head_3_back")
        location_result = await self.cartesian_location(*locations[f"head3_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 1 to exit failed when approaching to head_1_back")
            return location_result

        self.get_logger().info(f"Approaching to head 3")
        location_result = await self.cartesian_location(*locations[f"head3"], time=trajectory_time)
        if  location_result != True:
             self.get_logger().error(f"grab from head 3 to exit failed when approaching to head 1")
             return location_result
        
        #? This part can change, I think the lifters will be already up by this point!!!!!
        # try:
        #     send_http_request(http_commands[f'lifter_{head}_up'])
        #     self.get_logger().info(f"Waiting for lifter {head} to go up")
        # except Exception as e:
        #     self.get_logger().error(f"lifter {head} failed while going up")
        #     self.get_logger().error(f"lifter {head} says: {str(e)}")
        #     return False
        # time.sleep(7)
        #?###########################################################


        
        time.sleep(3)
        
        self.get_logger().info(f"Closing gripper!!!")
        gripperResponse = self.gripper_close()
        if gripperResponse != True:
            self.get_logger().error("something failed when closing the gripper")
            return gripperResponse
        
       

        
        # time.sleep(5)

        
        send_http_request(http_commands['z_up'])

        send_http_request(http_commands[f'lifter_3_down'])
        time.sleep(3)

        self.get_logger().info(f"Approaching to head_3_back")
        location_result = await self.cartesian_location(*locations[f"head3_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 2 to exit failed when approaching to head_1_back")
            return location_result
        
        self.get_logger().info(f"Approaching to head_2_back")
        location_result = await self.cartesian_location(*locations[f"head2_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 1 to exit failed when approaching to head_1_back")
            return location_result
        
        
        # self.get_logger().info(f"Z-Clearance")
        # location_result = await self.cartesian_location(*locations[f"z_clearance"], time=trajectory_time)
        # if  location_result != True:
        #     self.get_logger().error(f"grab from head {head} to exit failed when approaching to head_{head}_back")
        #     return location_result
        send_http_request(http_commands[f'z_clearance'])
        time.sleep(4)



        self.get_logger().info(f"rotating servoZ to drop position")
        rotatingServoZResponse = self.rotate_servoZ(angle=angles["rotating_z_drop"]) 
        if rotatingServoZResponse != True:
            self.get_logger().error("Something failed when rotating the servoZ, to drop position")
            return rotatingServoZResponse

        self.get_logger().info(f"Approaching to inlet0")
        location_result = await self.cartesian_location(*locations["inlet0"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching to inlet1")
            return location_result

    
        self.get_logger().info(f"rotating servoX to inlet1 position")
        rotatingServoXResponse = self.rotate_servoX(angle=angles["inlet1x"]) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet1 position")
            return rotatingServoXResponse


        self.get_logger().info(f"Approaching to inlet1")
        location_result = await self.cartesian_location(*locations["inlet1"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching to inlet1")
            return location_result

        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=190.0) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse

        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=170.0) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse


        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=150.0) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse
    
        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=130.0) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse
        
 
        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=110.0) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse

        
        self.get_logger().info(f"Approaching to head_2_back")
        location_result = await self.cartesian_location(*locations[f"head2_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 2 to exit failed when approaching to head_2_back")
            return location_result
        
        


        self.get_logger().info(f"rotating servoX to reset position")
        rotatingServoXResponse = self.rotate_servoX(angle=angles["rotating_x_reset"]) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to reset position")
            return rotatingServoXResponse
        
        self.get_logger().info(f"rotating servoZ to reset position")
        rotatingServoXResponse = self.rotate_servoZ(angle=angles["rotating_z_reset"]) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoZ, to reset position")
            return rotatingServoXResponse
        



        # self.get_logger().info(f"Approaching to head_{head}_back and rotating servoZ to reset position")
        # movement_result = await self.rotating_translatingZ_movement(*locations[f"head{head}_back"],angle=angles["rotating_z_reset"], time=2.0)
        # if movement_result != True:
        #     self.get_logger().error("Something failed when rotating and translating at the same time")
        #     return movement_result




        self.get_logger().info(f"Approaching to head_3_back")
        location_result = await self.cartesian_location(*locations[f"head3_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 1 to exit failed when approaching to head_1_back")
            return location_result
        

        
        


        send_http_request(http_commands[f'z_up'])
        time.sleep(4)


        self.get_logger().info(f"Approaching to head 1")
        location_result = await self.cartesian_location(*locations[f"head3"], time=trajectory_time)
        if  location_result != True:
             self.get_logger().error(f"grab from head 1 to exit failed when approaching to head 1")
             return location_result
        
       
        # send_http_request(http_commands['z_down'])
      
        # time.sleep(7)



        send_http_request(http_commands[f'lifter_3_up'])
        send_http_request(http_commands[f'z_down'])
          
        time.sleep(5)

        self.get_logger().info(f"Openning gripper!!!")
        gripperResponse = self.gripper_open()
        if gripperResponse != True:
            self.get_logger().error("something failed when opening the gripper")
            return gripperResponse
        time.sleep(2)

        self.get_logger().info(f"Approaching to head_3_back")
        location_result = await self.cartesian_location(*locations[f"head3_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 2 to exit failed when approaching to head_2_back")
            return location_result


        return True


    async def grab_to_exit_master_4(self, head):
        self.get_logger().info(f"Approaching to head_1_back")
        location_result = await self.cartesian_location(*locations[f"head1_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 1 to exit failed when approaching to head_1_back")
            return location_result

        self.get_logger().info(f"Approaching to head 1")
        location_result = await self.cartesian_location(*locations[f"head1"], time=trajectory_time)
        if  location_result != True:
             self.get_logger().error(f"grab from head 1 to exit failed when approaching to head 1")
             return location_result
        
        #? This part can change, I think the lifters will be already up by this point!!!!!
        # try:
        #     send_http_request(http_commands[f'lifter_{head}_up'])
        #     self.get_logger().info(f"Waiting for lifter {head} to go up")
        # except Exception as e:
        #     self.get_logger().error(f"lifter {head} failed while going up")
        #     self.get_logger().error(f"lifter {head} says: {str(e)}")
        #     return False
        # time.sleep(7)
        #?###########################################################

        send_http_request(http_commands[f'lifter_1_up'])
        send_http_request(http_commands[f'lifter_2_up'])       
        time.sleep(3)
        
        self.get_logger().info(f"Closing gripper!!!")
        gripperResponse = self.gripper_close()
        if gripperResponse != True:
            self.get_logger().error("something failed when closing the gripper")
            return gripperResponse
        
       

        
        # time.sleep(5)

        
        send_http_request(http_commands['z_up'])

        send_http_request(http_commands[f'lifter_1_down'])
        time.sleep(4)

        self.get_logger().info(f"Approaching to head_1_back")
        location_result = await self.cartesian_location(*locations[f"head1_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 1 to exit failed when approaching to head_1_back")
            return location_result
        
        send_http_request(http_commands[f'z_clearance'])
        time.sleep(4)

        
        self.get_logger().info(f"Approaching to head_1_back")
        location_result = await self.cartesian_location(*locations[f"head2_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 1 to exit failed when approaching to head_1_back")
            return location_result
        
        
        # self.get_logger().info(f"Z-Clearance")
        # location_result = await self.cartesian_location(*locations[f"z_clearance"], time=trajectory_time)
        # if  location_result != True:
        #     self.get_logger().error(f"grab from head {head} to exit failed when approaching to head_{head}_back")
        #     return location_result



        self.get_logger().info(f"rotating servoZ to drop position")
        rotatingServoZResponse = self.rotate_servoZ(angle=angles["rotating_z_drop"]) 
        if rotatingServoZResponse != True:
            self.get_logger().error("Something failed when rotating the servoZ, to drop position")
            return rotatingServoZResponse

        self.get_logger().info(f"Approaching to inlet0")
        location_result = await self.cartesian_location(*locations["inlet0"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching to inlet1")
            return location_result

    
        self.get_logger().info(f"rotating servoX to inlet1 position")
        rotatingServoXResponse = self.rotate_servoX(angle=angles["inlet1x"]) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet1 position")
            return rotatingServoXResponse

        self.get_logger().info(f"Approaching to inlet0")
        location_result = await self.cartesian_location(*locations["inlet1"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching to inlet1")
            return location_result


        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=190.0) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse
        time.sleep(1)
        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=170.0) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse


        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=150.0) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse

        time.sleep(1)     
        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=130.0) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse
        
        time.sleep(1)
        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=110.0) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse

        
        

        
        self.get_logger().info(f"Approaching to head_2_back")
        location_result = await self.cartesian_location(*locations[f"head2_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 2 to exit failed when approaching to head_2_back")
            return location_result
        
        


        self.get_logger().info(f"rotating servoX to reset position")
        rotatingServoXResponse = self.rotate_servoX(angle=angles["rotating_x_reset"]) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to reset position")
            return rotatingServoXResponse
        
        self.get_logger().info(f"rotating servoZ to reset position")
        rotatingServoXResponse = self.rotate_servoZ(angle=angles["rotating_z_reset"]) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoZ, to reset position")
            return rotatingServoXResponse
        



        # self.get_logger().info(f"Approaching to head_{head}_back and rotating servoZ to reset position")
        # movement_result = await self.rotating_translatingZ_movement(*locations[f"head{head}_back"],angle=angles["rotating_z_reset"], time=2.0)
        # if movement_result != True:
        #     self.get_logger().error("Something failed when rotating and translating at the same time")
        #     return movement_result




        self.get_logger().info(f"Approaching to head_1_back")
        location_result = await self.cartesian_location(*locations[f"head1_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 1 to exit failed when approaching to head_1_back")
            return location_result
        

        
        


        send_http_request(http_commands[f'z_up'])
        time.sleep(6)


        self.get_logger().info(f"Approaching to head 1")
        location_result = await self.cartesian_location(*locations[f"head1"], time=trajectory_time)
        if  location_result != True:
             self.get_logger().error(f"grab from head 1 to exit failed when approaching to head 1")
             return location_result
        
       
        # send_http_request(http_commands['z_down'])
      
        # time.sleep(7)

       
        send_http_request(http_commands[f'lifter_1_up'])
        send_http_request(http_commands[f'z_down'])  
        time.sleep(5)

        self.get_logger().info(f"Openning gripper!!!")
        gripperResponse = self.gripper_open()
        if gripperResponse != True:
            self.get_logger().error("something failed when opening the gripper")
            return gripperResponse
        time.sleep(2)

        self.get_logger().info(f"Approaching to head_1_back")
        location_result = await self.cartesian_location(*locations[f"head{head}_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching to head_{head}_back")
            return location_result
        
        self.get_logger().info(f"Approaching to head_2_back")
        location_result = await self.cartesian_location(*locations[f"head2_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 1 to exit failed when approaching to head_1_back")
            return location_result

        self.get_logger().info(f"Approaching to head 2")
        location_result = await self.cartesian_location(*locations[f"head2"], time=trajectory_time)
        if  location_result != True:
             self.get_logger().error(f"grab from head 2 to exit failed when approaching to head 1")
             return location_result
        
        #? This part can change, I think the lifters will be already up by this point!!!!!
        # try:
        #     send_http_request(http_commands[f'lifter_{head}_up'])
        #     self.get_logger().info(f"Waiting for lifter {head} to go up")
        # except Exception as e:
        #     self.get_logger().error(f"lifter {head} failed while going up")
        #     self.get_logger().error(f"lifter {head} says: {str(e)}")
        #     return False
        # time.sleep(7)
        #?###########################################################


        
        time.sleep(3)
        
        self.get_logger().info(f"Closing gripper!!!")
        gripperResponse = self.gripper_close()
        if gripperResponse != True:
            self.get_logger().error("something failed when closing the gripper")
            return gripperResponse
        
       

        
        # time.sleep(5)

        
        send_http_request(http_commands['z_up'])

        send_http_request(http_commands[f'lifter_2_down'])
        time.sleep(3)

        self.get_logger().info(f"Approaching to head_2_back")
        location_result = await self.cartesian_location(*locations[f"head2_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 2 to exit failed when approaching to head_1_back")
            return location_result
        
        self.get_logger().info(f"Approaching to head_2_back")
        location_result = await self.cartesian_location(*locations[f"head2_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 1 to exit failed when approaching to head_1_back")
            return location_result
        
        
        # self.get_logger().info(f"Z-Clearance")
        # location_result = await self.cartesian_location(*locations[f"z_clearance"], time=trajectory_time)
        # if  location_result != True:
        #     self.get_logger().error(f"grab from head {head} to exit failed when approaching to head_{head}_back")
        #     return location_result
        send_http_request(http_commands[f'z_clearance'])
        time.sleep(4)



        self.get_logger().info(f"rotating servoZ to drop position")
        rotatingServoZResponse = self.rotate_servoZ(angle=angles["rotating_z_drop"]) 
        if rotatingServoZResponse != True:
            self.get_logger().error("Something failed when rotating the servoZ, to drop position")
            return rotatingServoZResponse

        self.get_logger().info(f"Approaching to inlet0")
        location_result = await self.cartesian_location(*locations["inlet0"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching to inlet1")
            return location_result

    
        self.get_logger().info(f"rotating servoX to inlet1 position")
        rotatingServoXResponse = self.rotate_servoX(angle=angles["inlet1x"]) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet1 position")
            return rotatingServoXResponse


        self.get_logger().info(f"Approaching to inlet1")
        location_result = await self.cartesian_location(*locations["inlet1"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching to inlet1")
            return location_result




        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=190.0) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse
        time.sleep(1)
        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=170.0) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse


        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=150.0) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse

        time.sleep(1)     
        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=130.0) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse
        
        time.sleep(1)
        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=110.0) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse

        
        self.get_logger().info(f"Approaching to head_2_back")
        location_result = await self.cartesian_location(*locations[f"head2_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 2 to exit failed when approaching to head_2_back")
            return location_result
        
        


        self.get_logger().info(f"rotating servoX to reset position")
        rotatingServoXResponse = self.rotate_servoX(angle=angles["rotating_x_reset"]) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to reset position")
            return rotatingServoXResponse
        
        self.get_logger().info(f"rotating servoZ to reset position")
        rotatingServoXResponse = self.rotate_servoZ(angle=angles["rotating_z_reset"]) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoZ, to reset position")
            return rotatingServoXResponse
        



        # self.get_logger().info(f"Approaching to head_{head}_back and rotating servoZ to reset position")
        # movement_result = await self.rotating_translatingZ_movement(*locations[f"head{head}_back"],angle=angles["rotating_z_reset"], time=2.0)
        # if movement_result != True:
        #     self.get_logger().error("Something failed when rotating and translating at the same time")
        #     return movement_result




        self.get_logger().info(f"Approaching to head_2_back")
        location_result = await self.cartesian_location(*locations[f"head2_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 1 to exit failed when approaching to head_1_back")
            return location_result
        

        
        


        send_http_request(http_commands[f'z_up'])
        time.sleep(4)


        self.get_logger().info(f"Approaching to head 1")
        location_result = await self.cartesian_location(*locations[f"head2"], time=trajectory_time)
        if  location_result != True:
             self.get_logger().error(f"grab from head 1 to exit failed when approaching to head 1")
             return location_result
        
       
        # send_http_request(http_commands['z_down'])
      
        # time.sleep(7)



        send_http_request(http_commands[f'lifter_2_up'])
        send_http_request(http_commands[f'z_down'])
          
        time.sleep(5)

        self.get_logger().info(f"Openning gripper!!!")
        gripperResponse = self.gripper_open()
        if gripperResponse != True:
            self.get_logger().error("something failed when opening the gripper")
            return gripperResponse
        time.sleep(2)

        self.get_logger().info(f"Approaching to head_2_back")
        location_result = await self.cartesian_location(*locations[f"head2_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 2 to exit failed when approaching to head_2_back")
            return location_result
        
        return True
    async def gantry_pick_up(self, request, response):          
        requested_head = request.headid
        try:   
            grabToExitResponse = await self.grab_to_exit_master(requested_head)
            if grabToExitResponse != True:
                response.success = False
                response.message = f"Something failed during grabing sequence of head {requested_head}"
            
            response.success = True
            response.message = "Sequence finished"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response


    def pusher_master(self, idPusher, command, timeToComplete = -1):
        pusherMasterRequest = PusherTimeMaster.Request()
        pusherMasterRequest.id = idPusher
        pusherMasterRequest.command = command
        self.get_logger().info(f"TIME TO COMPLETE SELECTED {timeToComplete}")
        pusherMasterRequest.time = timeToComplete
        pusherMasterResponse = self.pusher_client_master.call(pusherMasterRequest)
        return pusherMasterResponse.success
    
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
    
    async def rotating_translatingZ_movement(self, x, y, angle, time):
        xLocationRequest = ServoAbsNice.Request()
        xLocationRequest.position = x
        xLocationRequest.time = time
        
        yLocationRequest = ServoAbsNice.Request()
        yLocationRequest.position = y
        yLocationRequest.time = time

        angleRequest = RotatingServoTimeBased.Request()
        angleRequest.angle = angle
        angleRequest.time = time

        futurex         = self.servo_x_client.call_async(xLocationRequest)
        futurey         = self.servo_y_client.call_async(yLocationRequest)
        futureAngleZ    = self.rotating_servoZ_time_based_client.call_async(angleRequest)

        resultx = await futurex
        resulty = await futurey
        resultAngleZ = await futureAngleZ
        if (resulty.success and resultx.success and resultAngleZ.success):
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

    def fill_tank_on(self):
        pumpRequest = PumpControl.Request()
        pumpRequest.commandid = 2
        response = self.pusher_client.call(pumpRequest)
        time.sleep(0.5)
        return response.success
    
    def fill_tank_off(self):
        pumpRequest = PumpControl.Request()
        pumpRequest.commandid = -2
        response = self.pusher_client.call(pumpRequest)
        time.sleep(0.5)
        return response.success
    
    def drain_tank_on(self):
        pumpRequest = PumpControl.Request()
        pumpRequest.commandid = 1
        response = self.pusher_client.call(pumpRequest)
        time.sleep(0.5)
        return response.success
    
    def drain_tank_off(self):
        pumpRequest = PumpControl.Request()
        pumpRequest.commandid = -1
        response = self.pusher_client.call(pumpRequest)
        time.sleep(0.5)
        return response.success
    
    async def reset_function(self):
        
        gripperResponse = self.gripper_open()
        if gripperResponse != True:
            self.get_logger().error("something failed when closing the gripper")
            return False
            
        location_result = await self.cartesian_location(*locations["reset"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error("Reseting position failed")
            return False
        
        rotatingServoXResponse = self.rotate_servoX(angle=angles["rotating_x_reset"]) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoZ, to its init position")
            return False

     
        rotatingServoZResponse = self.rotate_servoZ(angle=angles["rotating_z_reset"]) 
        if rotatingServoZResponse != True:
            self.get_logger().error("Something failed when rotating the servoZ, to its init position")
            return False
        
        try:
            send_http_request(http_commands['lifter_2_up'])
            send_http_request(http_commands['z_down'])
            time.sleep(6)
        except Exception as e:
            self.get_logger().error("HTTP request failed")
            self.get_logger().error(str(e))
            return False

        return True
    
    async def reset(self):
        

        
        try:
            #send_http_request(http_commands['lifter_1_init'])
            send_http_request(http_commands['lifter_2_init'])
            # send_http_request(http_commands['lifter_3_init'])
            # send_http_request(http_commands['lifter_4_init'])
            send_http_request(http_commands['z_init'])
            time.sleep(6)
        except Exception as e:
            self.get_logger().error("HTTP request failed")
            self.get_logger().error(str(e))
            return False

        return True
    
    async def reset_function_master(self):
        
        gripperResponse = self.gripper_open()
        if gripperResponse != True:
            self.get_logger().error("something failed when closing the gripper")
            return False
        
        location_result = await self.cartesian_location(*locations["reset"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error("Reseting position failed")
            return False
        
        rotatingServoXResponse = self.rotate_servoX(angle=angles["rotating_x_reset"]) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoZ, to its init position")
            return False

     
        rotatingServoZResponse = self.rotate_servoZ(angle=angles["rotating_z_reset"]) 
        if rotatingServoZResponse != True:
            self.get_logger().error("Something failed when rotating the servoZ, to its init position")
            return False
        
        try:
            # send_http_request(http_commands['lifter_4_up'])
            # send_http_request(http_commands['lifter_3_up'])
            send_http_request(http_commands['lifter_4_init'])
            # send_http_request(http_commands['lifter_3_init'])
            send_http_request(http_commands['lifter_2_init'])
            send_http_request(http_commands['lifter_1_init'])
            send_http_request(http_commands['z_init'])
            #time.sleep(26)
            # self.get_logger().info("Trying to lift lifter2 up!!!!")
            # send_http_request(http_commands['lifter_2_up'])
            #send_http_request(http_commands['lifter_1_up'])
            time.sleep(5)
        except Exception as e:
            self.get_logger().error("HTTP request failed")
            self.get_logger().error(str(e))
            return False

        return True




    async def end_effector_init(self, request, response):

        gripperResponse = self.gripper_open()
        if gripperResponse != True:
            self.get_logger().error("something failed when closing the gripper")
            return False
        
        location_result = await self.cartesian_location(*locations["reset"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error("Reseting position failed")
            return False
        
        rotatingServoXResponse = self.rotate_servoX(angle=angles["rotating_x_reset"]) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoZ, to its init position")
            return False

     
        rotatingServoZResponse = self.rotate_servoZ(angle=angles["rotating_z_reset"]) 
        if rotatingServoZResponse != True:
            self.get_logger().error("Something failed when rotating the servoZ, to its init position")
            return False
        
        try:
            # send_http_request(http_commands['lifter_4_up'])
            # send_http_request(http_commands['lifter_3_up'])
            send_http_request(http_commands['lifter_4_init'])
            send_http_request(http_commands['lifter_3_init'])
            send_http_request(http_commands['lifter_2_init'])
            send_http_request(http_commands['lifter_1_init'])
            send_http_request(http_commands['z_init'])
            #time.sleep(26)
            # self.get_logger().info("Trying to lift lifter2 up!!!!")
            # send_http_request(http_commands['lifter_2_up'])
            #send_http_request(http_commands['lifter_1_up'])
            time.sleep(5)
        except Exception as e:
            self.get_logger().error("HTTP request failed")
            self.get_logger().error(str(e))
            return False

        return True

        


    
        # location_result = await self.cartesian_location(*locations["reset"], time=trajectory_time)
        # if  location_result != True:
        #      response.success = False
        #      response.message = "One or the two axis did not move, (step1)"
        #      return response

        

        # rotatingServoXResponse = self.rotate_servoX(angle=angles["rotating_x_reset"]) 
        # if rotatingServoXResponse != True:
        #     response.success = False
        #     response.message = "Something failed when rotating the servoZ, to its init position"
        #     return response

     
        # rotatingServoZResponse = self.rotate_servoZ(angle=angles["rotating_z_reset"]) 
        # if rotatingServoZResponse != True:
        #     response.success = False
        #     response.message = "Something failed when rotating the servoZ, to its init position"
        #     return response
        
        # response.success = True
        # response.message = "Init function finished"
        # return response
    
    def rotatingX_servo_init(self, request, response):
                
        rotatingServoXResponse = self.rotate_servoX(angle=angles["rotating_x_reset"]) 
        if rotatingServoXResponse != True:
            response.success = False
            response.message = "Something failed when rotating the servoZ, to its init position"
            return response
        
        response.success = True
        response.message = "Init function finished"
        return response

    async def sequence_algorithm(self, request, response):
        try:
            #* SETP01
            gripperResponse = self.gripper_open()
            if gripperResponse != True:
                response.success = False
                response.message = "something failed when closing the gripper"
                return response
            

            #* SETP02
            location_result = await self.cartesian_location(*locations["home"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move"
                 return response


            #* SETP02
            location_result = await self.cartesian_location(*locations["reset"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move"
                 return response

            #*SETP03
            rotatingServoXResponse = self.rotate_servoX(angle=135.18) 
            if rotatingServoXResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ"
                return response

            #*SETP04
            rotatingServoZResponse = self.rotate_servoZ(angle=270.0) 
            if rotatingServoZResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ"
                return response

            #* SETP02
            location_result = await self.cartesian_location(*locations["head3_back"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move"
                 return response
            
            #* SETP02
            location_result = await self.cartesian_location(*locations["head3"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move"
                 return response

            
            
            # #* SETP05
            # send_http_request(http_commands['lifter_1_up'])
            # time.sleep(1)

            # #* SETP06
            # pusherResponse = self.pusher_open()
            # if pusherResponse != True:
            #     response.success = False
            #     response.message = "something failed when opening the pusher"
            #     return response

            # #* SETP07
            # pusherResponse = self.pusher_close()
            # if pusherResponse != True:
            #     response.success = False
            #     response.message = "something failed when closing the pusher"
            #     return response

            # #* SETP08
            # send_http_request(http_commands['lifter_1_down'])
            # time.sleep(15)

            # #* SETP09
            # send_http_request(http_commands['lifter_1_shaking'])
            # time.sleep(15)
            # time.sleep(20)

            # #* SETP10
            # send_http_request(http_commands['lifter_1_up'])
            # time.sleep(1)

            # #* SETP11
            # location_result = await self.cartesian_location(*locations["head4"], time=trajectory_time)
            # if  location_result != True:
            #      response.success = False
            #      response.message = "One or the two axis did not move"
            #      return response
            #             #* SETP10

            # gripperResponse = self.gripper_close()
            # if gripperResponse != True:
            #     response.success = False
            #     response.message = "something failed when closing the gripper, (step-1)"
            #     return response

            # send_http_request(http_commands['lifter_1_down'])

            # #* SETP12
            # location_result = await self.cartesian_location(*locations["head4_back"], time=trajectory_time)
            # if  location_result != True:
            #      response.success = False
            #      response.message = "One or the two axis did not move"
            #      return response

            # #* SETP13
            # send_http_request(http_commands['z_up'])

            # #* SETP12
            # location_result = await self.cartesian_location(*locations["exit"], time=trajectory_time)
            # if  location_result != True:
            #      response.success = False
            #      response.message = "One or the two axis did not move"
            #      return response

            # #*SETP14
            # rotatingServoXResponse = self.rotate_servoX(angle=315.18) 
            # if rotatingServoXResponse != True:
            #     response.success = False
            #     response.message = "Something failed when rotating the servoZ, (step2)"
            #     return response
            # time.sleep(0.6)

            # #*SETP15
            # rotatingServoXResponse = self.rotate_servoX(angle=135.18) 
            # if rotatingServoXResponse != True:
            #     response.success = False
            #     response.message = "Something failed when rotating the servoZ, (step2)"
            #     return response

            # #* SETP13
            # location_result = await self.cartesian_location(*locations["head4_back"], time=trajectory_time)
            # if  location_result != True:
            #      response.success = False
            #      response.message = "One or the two axis did not move"
            #      return response

            # #* SETP13
            # location_result = await self.cartesian_location(*locations["head4"], time=trajectory_time)
            # if  location_result != True:
            #      response.success = False
            #      response.message = "One or the two axis did not move"
            #      return response

            # send_http_request(http_commands['z_down'])

            # send_http_request(http_commands['lifter_1_up'])

            # gripperResponse = self.gripper_open()
            # if gripperResponse != True:
            #     response.success = False
            #     response.message = "something failed when closing the gripper, (step-1)"
            #     return response

            # #* SETP13
            # location_result = await self.cartesian_location(*locations["head4_back"], time=trajectory_time)
            # if  location_result != True:
            #      response.success = False
            #      response.message = "One or the two axis did not move"
            #      return response

            # send_http_request(http_commands['lifter_1_down'])

            # #* SETP13
            # location_result = await self.cartesian_location(*locations["reset"], time=trajectory_time)
            # if  location_result != True:
            #      response.success = False
            #      response.message = "One or the two axis did not move"
            #      return response
            response.success = True
            response.message = "Sequence finished"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    
    async def one_serve(self, request, response):
        try:
            #*SETP00
            initResponse = await self.reset_function()
            if initResponse != True:
                response.success = False
                response.message = "something failed during reset"
                return response


            
            #* SETP02
            location_result = await self.cartesian_location(*locations["head4_back"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move"
                 return response
            
            # #* SETP02
            # location_result = await self.cartesian_location(*locations["head4"], time=trajectory_time)
            # if  location_result != True:
            #      response.success = False
            #      response.message = "One or the two axis did not move"
            #      return response


            #* SETP03
            pusherResponse = self.pusher_open()
            if pusherResponse != True:
                response.success = False
                response.message = "something failed when opening the pusher"
                return response

            #* SETP04
            pusherResponse = self.pusher_close()
            if pusherResponse != True:
                response.success = False
                response.message = "something failed when closing the pusher"
                return response
            
            #* SETP08
            send_http_request(http_commands['lifter_4_down'])

            time.sleep(25)



            #* SETP02
            location_result = await self.cartesian_location(*locations["head4"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move"
                 return response

            # #* SETP09
            # send_http_request(http_commands['lifter_1_shaking'])
            # time.sleep(15)
            # time.sleep(20)

            #* SETP10
            send_http_request(http_commands['lifter_4_up'])
            time.sleep(1)

            #* SETP11
            location_result = await self.cartesian_location(*locations["head4"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move"
                 return response

            time.sleep(7)


            gripperResponse = self.gripper_close()
            if gripperResponse != True:
                response.success = False
                response.message = "something failed when closing the gripper, (step-1)"
                return response
            
            
            send_http_request(http_commands['lifter_4_down'])
            time.sleep(5)

            send_http_request(http_commands['z_clearance'])
            time.sleep(7)

            #* SETP12
            location_result = await self.cartesian_location(*locations["head4_back"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move"
                 return response

            # #* SETP13
            # send_http_request(http_commands['z_up'])

            #* SETP12
            location_result = await self.cartesian_location(*locations["exit"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move"
                 return response

            #*SETP14
            rotatingServoXResponse = self.rotate_servoX(angle=315.18) 
            if rotatingServoXResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response
            time.sleep(0.6)

            #*SETP15
            rotatingServoXResponse = self.rotate_servoX(angle=135.18) 
            if rotatingServoXResponse != True:
                response.success = False
                response.message = "Something failed when rotating the servoZ, (step2)"
                return response

            #* SETP13
            location_result = await self.cartesian_location(*locations["head4_back"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move"
                 return response
            
            

            #* SETP14
            location_result = await self.cartesian_location(*locations["head4"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move"
                 return response
            
            # send_http_request(http_commands['z_down'])
            send_http_request(http_commands['z_down'])
            time.sleep(7)


            send_http_request(http_commands['lifter_4_up'])

            time.sleep(5)

            gripperResponse = self.gripper_open()
            if gripperResponse != True:
                response.success = False
                response.message = "something failed when closing the gripper, (step-1)"
                return response
            
            time.sleep(3)


            #* SETP13
            location_result = await self.cartesian_location(*locations["head4_back"], time=trajectory_time)
            if  location_result != True:
                 response.success = False
                 response.message = "One or the two axis did not move"
                 return response

            # send_http_request(http_commands['lifter_1_down'])

            # #* SETP13
            # location_result = await self.cartesian_location(*locations["reset"], time=trajectory_time)
            # if  location_result != True:
            #      response.success = False
            #      response.message = "One or the two axis did not move"
            #      return response
            response.success = True
            response.message = "Sequence finished"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
        
    async def reset (self, request, response):
        self.get_logger().info("Reset triggered!!!")

        try:
            self.get_logger().info("Serving_noodle 1 accepted running rseting function!!!")
            initResponse = await self.reset_function_master()
            if initResponse != True:
                response.success = False
                response.message = "something failed during reset"
                return response

            send_http_request(http_commands['z_down'])
               
            time.sleep(3)



           
            
            # for i in range(request.serving):
            #     grabToExitResponse = await self.grab_to_exit_master(5-(i+1))
            #     if grabToExitResponse != True:
            #         response.success = False
            #         response.message = f"Something failed during grabing sequence of head {5-(i+1)}"

            
            grabToExitResponse = await self.grab_to_exit_master(2)
            if grabToExitResponse != True:
                response.success = False
                response.message = f"Something failed during grabing sequence of dispenser 2"
            
            
            response.success = True
            response.message = "Sequence finished"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    async def serving_noodle(self, request, response):
        self.get_logger().info("Serving_noodle 1 has been triggered!!!")
        if (request.serving > 4) or (request.serving <= 0):
            response.success = False
            response.message = "Maximum allowable servings are from 1 to 4 servings"
            return response 
        try:
            # self.get_logger().info("Serving_noodle 1 accepted running rseting function!!!")
            # initResponse = await self.reset_function_master()
            # if initResponse != True:
            #     response.success = False
            #     response.message = "something failed during reset"
            #     return response
            
            # time.sleep(15)
            send_http_request(http_commands['lifter_1_up'])
            time.sleep(10)
            pusherMasterResponse = self.open_pusher_master(request.serving)
            if pusherMasterResponse != True:
                response.success = False
                response.message = f"something failed during opening {request.serving} pushers"
                return response
            time.sleep(1)

            pusherMasterResponse = self.close_pusher_master(request.serving)
            if pusherMasterResponse != True:
                response.success = False
                response.message = f"something failed during closing {request.serving} pushers"
                return response


            send_http_request(http_commands['lifter_1_shaking'])
            time.sleep(20)

            send_http_request(http_commands['lifter_1_down'])
           
            time.sleep(20)
            
            send_http_request(http_commands['lifter_1_up'])

           
            
            # for i in range(request.serving):
            #     grabToExitResponse = await self.grab_to_exit_master(i)
            #     if grabToExitResponse != True:
            #         response.success = False
            #         response.message = f"Something failed during grabing sequence of head {i}"


            #Head 1           
            grabToExitResponse = await self.grab_to_exit_master(1)
            if grabToExitResponse != True:
                response.success = False
                response.message = f"Something failed during grabing sequence of dispenser 2"

            # grabToExitResponse = await self.grab_to_exit_master(2)
            # if grabToExitResponse != True:
            #     response.success = False
            #     response.message = f"Something failed during grabing sequence of dispenser 2"
            
            
            response.success = True
            response.message = "Sequence finished"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response


    # async def serving_noodle_2(self, request, response):
    #     self.get_logger().info("Serving_noodle 1 has been triggered!!!")
    #     if (request.serving > 4) or (request.serving <= 0):
    #         response.success = False
    #         response.message = "Maximum allowable servings are from 1 to 4 servings"
    #         return response 
    #     try:
    #         # self.get_logger().info("Serving_noodle 1 accepted running rseting function!!!")
    #         # initResponse = await self.reset_function_master()
    #         # if initResponse != True:
    #         #     response.success = False
    #         #     response.message = "something failed during reset"
    #         #     return response
            
    #         # time.sleep(15)
    #         send_http_request(http_commands['lifter_1_up'])
    #         send_http_request(http_commands['lifter_2_up'])

    #         time.sleep(7)
    #         pusherMasterResponse = self.open_pusher_master(2)
    #         if pusherMasterResponse != True:
    #             response.success = False
    #             response.message = f"something failed during opening {2} pushers"
    #             return response
    #         time.sleep(1)

    #         pusherMasterResponse = self.close_pusher_master(2)
    #         if pusherMasterResponse != True:
    #             response.success = False
    #             response.message = f"something failed during closing {2} pushers"
    #             return response

    #         send_http_request(http_commands['lifter_2_shaking'])
    #         time.sleep(1)
    #         send_http_request(http_commands['lifter_1_shaking'])
            
           
    #         time.sleep(18)

          

    #         send_http_request(http_commands['lifter_2_down'])
    #         time.sleep(3)
    #         send_http_request(http_commands['lifter_1_down'])
    #         time.sleep(1)
    #         time.sleep(30)
            
    #         send_http_request(http_commands['lifter_1_up'])
    #         time.sleep(1)
    #         send_http_request(http_commands['lifter_2_up'])

           
            
    #         # for i in range(request.serving):
    #         #     grabToExitResponse = await self.grab_to_exit_master(i)
    #         #     if grabToExitResponse != True:
    #         #         response.success = False
    #         #         response.message = f"Something failed during grabing sequence of head {i}"


    #         #Head 1           
    #         grabToExitResponse = await self.grab_to_exit_master_2(1)
    #         if grabToExitResponse != True:
    #             response.success = False
    #             response.message = f"Something failed during grabing sequence of dispenser 2"

    #         # grabToExitResponse = await self.grab_to_exit_master(2)
    #         # if grabToExitResponse != True:
    #         #     response.success = False
    #         #     response.message = f"Something failed during grabing sequence of dispenser 2"
            
            
    #         response.success = True
    #         response.message = "Sequence finished"
    #     except Exception as e:
    #         response.success = False
    #         response.message = str(e)
    #     return response
    async def serving_noodle_2(self, request, response):
        self.get_logger().info("Serving_noodle 1 has been triggered!!!")
        if (request.serving > 4) or (request.serving <= 0):
            response.success = False
            response.message = "Maximum allowable servings are from 1 to 4 servings"
            return response 
        try:
            # self.get_logger().info("Serving_noodle 1 accepted running rseting function!!!")
            # initResponse = await self.reset_function_master()
            # if initResponse != True:
            #     response.success = False
            #     response.message = "something failed during reset"
            #     return response
            
            # time.sleep(15)
            send_http_request(http_commands['lifter_1_up'])
            time.sleep(1)
            send_http_request(http_commands['lifter_2_up'])
            time.sleep(1)
            send_http_request(http_commands['lifter_3_up'])
            time.sleep(1)
            send_http_request(http_commands['lifter_4_up'])
            time.sleep(1)
            
            time.sleep(7)
            pusherMasterResponse = self.open_pusher_master(4)
            if pusherMasterResponse != True:
                response.success = False
                response.message = f"something failed during opening {2} pushers"
                return response
            time.sleep(1)

            pusherMasterResponse = self.close_pusher_master(4)
            if pusherMasterResponse != True:
                response.success = False
                response.message = f"something failed during closing {2} pushers"
                return response


            send_http_request(http_commands['lifter_4_shaking'])
            time.sleep(1)
            send_http_request(http_commands['lifter_3_down'])   
            time.sleep(1)
            send_http_request(http_commands['lifter_2_shaking'])
            time.sleep(1)
            send_http_request(http_commands['lifter_1_shaking'])
            
           
            time.sleep(18)

            send_http_request(http_commands['lifter_4_down'])
            time.sleep(1)
            send_http_request(http_commands['lifter_3_down'])   
            time.sleep(1)
            send_http_request(http_commands['lifter_2_down'])
            time.sleep(1)
            send_http_request(http_commands['lifter_1_down'])
            time.sleep(1)
            time.sleep(30)
            send_http_request(http_commands['lifter_4_up'])
            time.sleep(1)
            send_http_request(http_commands['lifter_3_up'])  
            time.sleep(1)
            send_http_request(http_commands['lifter_2_up'])
            time.sleep(1)
            send_http_request(http_commands['lifter_1_up'])
            time.sleep(1)
           
            
            # for i in range(request.serving):
            #     grabToExitResponse = await self.grab_to_exit_master(i)
            #     if grabToExitResponse != True:
            #         response.success = False
            #         response.message = f"Something failed during grabing sequence of head {i}"


            #Head 1           
            grabToExitResponse = await self.grab_to_exit_master_2(1)
            if grabToExitResponse != True:
                response.success = False
                response.message = f"Something failed during grabing sequence of dispenser 2"

            # grabToExitResponse = await self.grab_to_exit_master(2)
            # if grabToExitResponse != True:
            #     response.success = False
            #     response.message = f"Something failed during grabing sequence of dispenser 2"
            
            
            response.success = True
            response.message = "Sequence finished"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
        
    async def serving_noodle_3 (self, request, response):
        self.get_logger().info("Serving_noodle 1 has been triggered!!!")
        if (request.serving > 4) or (request.serving <= 0):
            response.success = False
            response.message = "Maximum allowable servings are from 1 to 4 servings"
            return response 
        try:
            self.get_logger().info("Serving_noodle 1 accepted running rseting function!!!")
            initResponse = await self.reset_function_master()
            if initResponse != True:
                response.success = False
                response.message = "something failed during reset"
                return response
            
            #time.sleep(20)
            # send_http_request(http_commands['lifter_2_up'])
            # time.sleep(5)
            pusherMasterResponse = self.open_pusher_master(request.serving)
            if pusherMasterResponse != True:
                response.success = False
                response.message = f"something failed during opening {request.serving} pushers"
                return response
            # time.sleep(5)

            pusherMasterResponse = self.close_pusher_master(request.serving)
            if pusherMasterResponse != True:
                response.success = False
                response.message = f"something failed during closing {request.serving} pushers"
                return response

            # send_http_request(http_commands['z_down'])
               
            # time.sleep(3)

            send_http_request(http_commands['lifter_1_down'])
            send_http_request(http_commands['lifter_2_down'])
            send_http_request(http_commands['lifter_3_down'])
                  
            time.sleep(10)
            
            send_http_request(http_commands['lifter_1_up'])
            send_http_request(http_commands['lifter_2_up'])
            send_http_request(http_commands['lifter_3_up'])
            
            time.sleep(10)


           
            
            # for i in range(request.serving):
            #     grabToExitResponse = await self.grab_to_exit_master(5-(i+1))
            #     if grabToExitResponse != True:
            #         response.success = False
            #         response.message = f"Something failed during grabing sequence of head {5-(i+1)}"


            #Head 1           
            grabToExitResponse = await self.grab_to_exit_master(1)
            if grabToExitResponse != True:
                response.success = False
                response.message = f"Something failed during grabing sequence of dispenser 2"

            grabToExitResponse = await self.grab_to_exit_master(2)
            if grabToExitResponse != True:
                response.success = False
                response.message = f"Something failed during grabing sequence of dispenser 2"
            
            grabToExitResponse = await self.grab_to_exit_master(3)
            if grabToExitResponse != True:
                response.success = False
                response.message = f"Something failed during grabing sequence of dispenser 2"
            
            

            response.success = True
            response.message = "Sequence finished"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

            
        
        
            
    async def serving_noodle_4 (self, request, response):
        self.get_logger().info("Serving_noodle 1 has been triggered!!!")
        if (request.serving > 4) or (request.serving <= 0):
            response.success = False
            response.message = "Maximum allowable servings are from 1 to 4 servings"
            return response 
        try:
            self.get_logger().info("Serving_noodle 1 accepted running rseting function!!!")
            initResponse = await self.reset_function_master()
            if initResponse != True:
                response.success = False
                response.message = "something failed during reset"
                return response
            
            #time.sleep(20)
            # send_http_request(http_commands['lifter_2_up'])
            # time.sleep(5)
            pusherMasterResponse = self.open_pusher_master(request.serving)
            if pusherMasterResponse != True:
                response.success = False
                response.message = f"something failed during opening {request.serving} pushers"
                return response
            # time.sleep(5)

            pusherMasterResponse = self.close_pusher_master(request.serving)
            if pusherMasterResponse != True:
                response.success = False
                response.message = f"something failed during closing {request.serving} pushers"
                return response

            # send_http_request(http_commands['z_down'])
               
            # time.sleep(3)

            send_http_request(http_commands['lifter_1_down'])
            send_http_request(http_commands['lifter_2_down'])
            send_http_request(http_commands['lifter_3_down'])
            send_http_request(http_commands['lifter_4_down'])
                  
            time.sleep(10)
            
            send_http_request(http_commands['lifter_1_up'])
            send_http_request(http_commands['lifter_2_up'])
            send_http_request(http_commands['lifter_3_up'])
            send_http_request(http_commands['lifter_4_up'])
            
            time.sleep(10)


           
            
            # for i in range(request.serving):
            #     grabToExitResponse = await self.grab_to_exit_master(5-(i+1))
            #     if grabToExitResponse != True:
            #         response.success = False
            #         response.message = f"Something failed during grabing sequence of head {5-(i+1)}"


            #Head 1           
            grabToExitResponse = await self.grab_to_exit_master(1)
            if grabToExitResponse != True:
                response.success = False
                response.message = f"Something failed during grabing sequence of dispenser 2"

            grabToExitResponse = await self.grab_to_exit_master(2)
            if grabToExitResponse != True:
                response.success = False
                response.message = f"Something failed during grabing sequence of dispenser 2"
            
            grabToExitResponse = await self.grab_to_exit_master(3)
            if grabToExitResponse != True:
                response.success = False
                response.message = f"Something failed during grabing sequence of dispenser 2"

                            
            grabToExitResponse = await self.grab_to_exit_master(4)
            if grabToExitResponse != True:
                response.success = False
                response.message = f"Something failed during grabing sequence of dispenser 2"
            
            

            response.success = True
            response.message = "Sequence finished"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response


    async def one_serve (self, request, response):
        self.get_logger().info("Serving_noodle 1 has been triggered!!!")
        if (request.serving > 4) or (request.serving <= 0):
            response.success = False
            response.message = "Maximum allowable servings are from 1 to 4 servings"
            return response 
        try:
            self.get_logger().info("Serving_noodle 1 accepted running rseting function!!!")
            initResponse = await self.reset_function_master()
            if initResponse != True:
                response.success = False
                response.message = "something failed during reset"
                return response
            
            #time.sleep(20)
            # send_http_request(http_commands['lifter_2_up'])
            # time.sleep(5)
            pusherMasterResponse = self.open_pusher_master(request.serving)
            if pusherMasterResponse != True:
                response.success = False
                response.message = f"something failed during opening {request.serving} pushers"
                return response
            # time.sleep(5)

            pusherMasterResponse = self.close_pusher_master(request.serving)
            if pusherMasterResponse != True:
                response.success = False
                response.message = f"something failed during closing {request.serving} pushers"
                return response

            send_http_request(http_commands['z_down'])
               
            time.sleep(3)


            send_http_request(http_commands['lifter_2_down'])
            
            time.sleep(10)
            
            send_http_request(http_commands['lifter_2_up'])

           
            
            # for i in range(request.serving):
            #     grabToExitResponse = await self.grab_to_exit_master(5-(i+1))
            #     if grabToExitResponse != True:
            #         response.success = False
            #         response.message = f"Something failed during grabing sequence of head {5-(i+1)}"

            
            grabToExitResponse = await self.grab_to_exit_master(2)
            if grabToExitResponse != True:
                response.success = False
                response.message = f"Something failed during grabing sequence of dispenser 2"
            
            
            response.success = True
            response.message = "Sequence finished"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
  
    async def two_serves (self, request, response):
        self.get_logger().info("Serving_noodle 1 has been triggered!!!")
        if (request.serving > 4) or (request.serving <= 0):
            response.success = False
            response.message = "Maximum allowable servings are from 1 to 4 servings"
            return response 
        try:
           
           ## 1st serving 

           ## 2nd serving 
            
            response.success = True
            response.message = "Sequence finished"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
  
        

    async def three_serves (self, request, response):
        self.get_logger().info("Serving_noodle 1 has been triggered!!!")
        if (request.serving > 4) or (request.serving <= 0):
            response.success = False
            response.message = "Maximum allowable servings are from 1 to 4 servings"
            return response 
        try:
           
           ## 1st serving 

           ## 2nd serving 

           ## 3rd serving
            
            response.success = True
            response.message = "Sequence finished"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

            

    async def four_serves (self, request, response):
        self.get_logger().info("Serving_noodle 1 has been triggered!!!")
        if (request.serving > 4) or (request.serving <= 0):
            response.success = False
            response.message = "Maximum allowable servings are from 1 to 4 servings"
            return response 
        try:
           
           ## 1st serving 

           ## 2nd serving 

           ## 3rd serving

           ## 4th serving
            
            response.success = True
            response.message = "Sequence finished"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

        
    def dispense_noodle(self,request, response):
        pusherid = request.serving
        pusherResponse = self.pusher_master(idPusher=pusherid, command='open',timeToComplete=2000)
        if not pusherResponse:
            response.success = False
            response.message = f"Error dispense noodle {pusherid}. Something failed while openning pusher {pusherid}"
            return response
        
        pusherResponse = self.pusher_master(idPusher=pusherid, command='close',timeToComplete=2600)
        if not pusherResponse:
            response.success = False
            response.message = f"Error dispense noodle {pusherid}. Something failed while closing pusher {pusherid}"
            return response
        response.success = True
        response.message = f"Dispense noodle {pusherid} success"
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