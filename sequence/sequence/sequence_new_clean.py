import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
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
from gripper_interfaces.srv import MoveCscam





import time  # Import time module for adding delays between movements
import subprocess


trajectory_time = 1.0
locations = {
            "reset": (0.0, 0.0),
             "home": (0.0,0.0),

            "head1":(402.072, -74.052),
            "head1_back": (32.404, -74.052),

            "head2":(402.072, -263.046), #x: 404.095
            "head2_back": (32.404, -263.046),

            "head3":(402.072, -546.961), #x: 387.238
            "head3_back":(32.404, -546.961),


            "head4":(402.072, -725.324), #x: 391.797
            "head4_back": (32.404, -725.324),

            "dropReady":(32.404, 0.0),
            "dropCompleted":(32.404 + 143.35, 0.0),

            "inlet0": (686.532, 720.0),
                                    
            "inlet1": (930.000,730.917),
            "inlet2": (930.000, 730.963),

            "clean_disposal": (300.000, -300.00),

}

angles = {
    "rotating_z_reset": 180.0,
    "rotating_z_drop": 270.0,
    "rotating_z_clean": 90.0,

    "rotating_x_reset": 270.0,
    "rotating_x_drop": 90.0,

    "inlet1x": 200.0,
    "inlet2x": 110.0,
    "inlet3x": 110.0,

    "cleaning_x": 150.0,
}

lifter_location = {
    "lift_up":-65.0,
    "lift_down": 0.0
}

zlocation = {
    "up": -130.0, #(-321.333)
    "down": -190.0, #(-392)
    "up2": -120.0,
    "down2": -160.0,
    "reset": -10.0
}

#TODO: Make 3D full locations
locations3D = {
    "head1_back": (*locations["head1_back"],zlocation["down"],  angles["rotating_z_reset"], angles["rotating_x_reset"]),
    "head2_back": (*locations["head2_back"],zlocation["down"],  angles["rotating_z_reset"], angles["rotating_x_reset"]),
    "head3_back": (*locations["head3_back"],zlocation["down"],  angles["rotating_z_reset"], angles["rotating_x_reset"]),
    "head4_back": (*locations["head4_back"],zlocation["down"],  angles["rotating_z_reset"], angles["rotating_x_reset"]),
    "reset":      (*locations["reset"],     zlocation["reset"], angles["rotating_z_reset"], angles["rotating_x_reset"]),
    "head1_back_up": (*locations["head1_back"],zlocation["up2"],  angles["rotating_z_reset"], angles["rotating_x_reset"]),
    "head2_back_up": (*locations["head2_back"],zlocation["up2"],  angles["rotating_z_reset"], angles["rotating_x_reset"]),
    "head3_back_up": (*locations["head3_back"],zlocation["up2"],  angles["rotating_z_reset"], angles["rotating_x_reset"]),
    "head4_back_up": (*locations["head4_back"],zlocation["up2"],  angles["rotating_z_reset"], angles["rotating_x_reset"]),

    "dropReady":  (*locations["dropReady"], zlocation["reset"], angles["rotating_z_drop"],  angles["rotating_x_reset"]),
    #"dropCompleted":  (*locations["dropCompleted"], zlocation["reset"], angles["rotating_z_drop"],  angles["rotating_x_drop"]),

}


class LIFTERS:
    def __init__(self, lifterId):
        self.lifterId = lifterId
        self.up = True

class CookerRobotSequence(Node):
    def __init__(self):
        super().__init__('CookerRobotSequenceAlgorithm')
        self.clients_and_callbacks_setup()
        self.service_setup()
        self.get_logger().info(f"cooker robot fully setup")

    def clients_and_callbacks_setup(self):

        #*Linear Servodriver
        self.callback_servox = MutuallyExclusiveCallbackGroup()
        self.callback_servoy = MutuallyExclusiveCallbackGroup()
        self.callback_servox_homing = MutuallyExclusiveCallbackGroup()
        self.callback_servoy_homing = MutuallyExclusiveCallbackGroup()
       
        #*Rotating Servodriver
        self.callback_rotating_servoZ_time = MutuallyExclusiveCallbackGroup()
        self.callback_rotating_servoX_time = MutuallyExclusiveCallbackGroup()
        
        #* Gripper
        self.callback_gripper_group = MutuallyExclusiveCallbackGroup()
        
        #* Pusher
        self.callback_pusher_master_group = MutuallyExclusiveCallbackGroup()

        #* CSCAM motors
        self.callback_cscam_master_group = MutuallyExclusiveCallbackGroup()
        self.callback_cscamZ_master_group = MutuallyExclusiveCallbackGroup()
        
       
       

        #*Actions......
        self.dispense_noodle_group = MutuallyExclusiveCallbackGroup()
        self.cook_noodle_group = ReentrantCallbackGroup()
        #self.callback_cscamZ_master_group = ReentrantCallbackGroup()
        self.clean_noodle_group = MutuallyExclusiveCallbackGroup()
        self.serve_noodle_group = MutuallyExclusiveCallbackGroup()

        #*Linear Servodriver Client
        self.servo_x_client = self.create_client(ServoAbsNice, "/servo_controllerX/moveabsNice", callback_group=self.callback_servox)
        while not self.servo_x_client.wait_for_service(timeout_sec=20.0):
             self.get_logger().info('/servo_controllerX/moveabsNice not available, waiting again...')

        self.servo_y_client = self.create_client(ServoAbsNice, "/servo_controllerY/moveabsNice", callback_group=self.callback_servoy)
        while not self.servo_y_client.wait_for_service(timeout_sec=20.0):
             self.get_logger().info('/servo_controllerY/moveabsNice not available, waiting again...')
        
        #*Rotating Servodriver Client
        self.rotating_servoZ_time_based_client = self.create_client(RotatingServoTimeBased, "/rotating_servoZ/rotatingTimeBased", callback_group=self.callback_rotating_servoZ_time)
        while not self.rotating_servoZ_time_based_client.wait_for_service(timeout_sec=20.0):
             self.get_logger().info('/rotating_servoZ/rotatingTimeBased not available, waiting again...')

        self.rotating_servoX_time_based_client = self.create_client(RotatingServoTimeBased, "/rotating_servoX/rotatingTimeBased", callback_group=self.callback_rotating_servoX_time)
        while not self.rotating_servoX_time_based_client.wait_for_service(timeout_sec=20.0):
             self.get_logger().info('/rotating_servoX/rotatingTimeBased not available, waiting again...')

        #* Gripper Client
        self.gripper_client = self.create_client(GripperCom, "/gripper_controller/open_close", callback_group=self.callback_gripper_group)
        while not self.gripper_client.wait_for_service(timeout_sec=20.0):
             self.get_logger().info('/gripper_controller/open_close not available, waiting again...')

        #* Pusher Client
        self.pusher_client_master = self.create_client(PusherTimeMaster, "/pusher/open_close", callback_group=self.callback_pusher_master_group)
        while not self.pusher_client_master.wait_for_service(timeout_sec=20.0):
            self.get_logger().info('/pusher/open_close not available, waiting again...')

        #*Cscam Client
        self.cscam_client_master = self.create_client(MoveCscam, "/cscam/move", callback_group=self.callback_cscam_master_group)
        while not self.cscam_client_master.wait_for_service(timeout_sec=20.0):
            self.get_logger().info('/cscam/move not available, waiting again...')
        
        #  #*Cscam Client
        self.cscamZ_client_master = self.create_client(MoveCscam, "/cscam/moveZ", callback_group=self.callback_cscamZ_master_group)
        while not self.cscam_client_master.wait_for_service(timeout_sec=20.0):
            self.get_logger().info('/cscam/move not available, waiting again...')
        
        
        # self.client_sinusoidal = self.create_client(SinusoidalCscam, "/cscam/sinusoidal", callback_group=self.callback_cscam_sinusoidal_group)
        # while not self.client_sinusoidal.wait_for_service(timeout_sec=20.0):
        #     self.get_logger().info('/cscam/sinusoidal not available, waiting again...')
        
        # self.pump_client = self.create_client(PumpControl, "/pump_control", callback_group= self.callback_pump_controller_group)
        # while not self.pump_client.wait_for_service(timeout_sec=20.0):
        #     self.get_logger().info('/pump_control not available, waiting again...')
    
    def service_setup(self):
        self.dispense_noodle_srv        =  self.create_service(ServingNoodle, "/Dispense",     self.dispense_noodle,    callback_group=self.dispense_noodle_group)
        self.cook_noodle_srv            =  self.create_service(ServingNoodle, "/Cook"   ,      self.cook_noodle,        callback_group=self.cook_noodle_group)
        self.grab_and_exit_srv          =  self.create_service(ServingNoodle, "/Serve",        self.serve_noodle,       callback_group=self.serve_noodle_group)
        #self.clean_noodle_srv           =  self.create_service(Trigger,       "/CleanNoodle",  self.clean_noodle,       callback_group=self.clean_noodle_group)
        self.clean_noodle_srv = self.create_service(ServingNoodle, "/Clean", self.clean_noodle, callback_group=serve_noodle_group)
    
    
    #*Assisting Functions
    def pusher_master(self, idPusher, command, timeToComplete = -1):
        pusherMasterRequest = PusherTimeMaster.Request()
        pusherMasterRequest.id = idPusher
        pusherMasterRequest.command = command
        self.get_logger().info(f"TIME TO COMPLETE SELECTED {timeToComplete}")
        pusherMasterRequest.time = timeToComplete
        pusherMasterResponse = self.pusher_client_master.call(pusherMasterRequest)
        return pusherMasterResponse.success
    
    def lifter_master_position(self, lifterid, position):
        if lifterid > 4:
            return False
        cscamMasterRequest = MoveCscam.Request()
        cscamMasterRequest.motorid = lifterid 
        cscamMasterRequest.position = position
        cscamMasterResponse = self.cscam_client_master.call(cscamMasterRequest)
        # if cscamMasterRequest.message == 'Error: [Errno 32] Broken pipe':
        #     return True
        return cscamMasterResponse.success
    def z_master_position(self, position):
        cscamMasterRequest = MoveCscam.Request()
        cscamMasterRequest.motorid = 1
        cscamMasterRequest.position = position
        cscamMasterResponse = self.cscamZ_client_master.call(cscamMasterRequest)
        return cscamMasterResponse.success

    async def cartesian_location(self, x, y, time=2.0): #? Default time is two seconds
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
    async def cartesian_rotation_location(self, x, y, anglez, anglex, time=2.0):
        xLocationRequest = ServoAbsNice.Request()
        xLocationRequest.position = x
        xLocationRequest.time = time
        
        yLocationRequest = ServoAbsNice.Request()
        yLocationRequest.position = y
        yLocationRequest.time = time

        anglezRequest = RotatingServoTimeBased.Request()
        anglezRequest.angle = anglez
        anglezRequest.time = time

        anglexRequest = RotatingServoTimeBased.Request()
        anglexRequest.angle = anglex
        anglexRequest.time = time


        futurex = self.servo_x_client.call_async(xLocationRequest)
        futurey = self.servo_y_client.call_async(yLocationRequest)
        futureAngleZ    = self.rotating_servoZ_time_based_client.call_async(anglezRequest)
        futureAngleX    = self.rotating_servoX_time_based_client.call_async(anglexRequest)


        resultx = await futurex
        resulty = await futurey
        resultAngleZ = await futureAngleZ
        resultAngleX = await futureAngleX
        if (resulty.success and resultx.success and resultAngleZ and resultAngleX):
            return True
        else:
            return False

    async def cartesian_location3D(self, x, y, z, time=2.0):
        xLocationRequest = ServoAbsNice.Request()
        xLocationRequest.position = x
        xLocationRequest.time = time
        
        yLocationRequest = ServoAbsNice.Request()
        yLocationRequest.position = y
        yLocationRequest.time = time

        zlocationRequest = MoveCscam.Request()
        zlocationRequest.motorid = 1
        zlocationRequest.position = z

        futurex = self.servo_x_client.call_async(xLocationRequest)
        futurey = self.servo_y_client.call_async(yLocationRequest)
        futurez = self.cscamZ_client_master.call_async(zlocationRequest)

        resultx = await futurex
        resulty = await futurey
        resultz = await futurez
        if (resulty.success and resultx.success and resultz.success):
            return True
        else:
            return False
    async def rotatingZ_translating3D(self, x,y,z,anglez,time=2.0):
        xLocationRequest = ServoAbsNice.Request()
        xLocationRequest.position = x
        xLocationRequest.time = time
        
        yLocationRequest = ServoAbsNice.Request()
        yLocationRequest.position = y
        yLocationRequest.time = time

        zlocationRequest = MoveCscam.Request()
        zlocationRequest.motorid = 1
        zlocationRequest.position = z
        
        
        angleRequest = RotatingServoTimeBased.Request()
        angleRequest.angle = anglez
        angleRequest.time = time

        futurex         = self.servo_x_client.call_async(xLocationRequest)
        futurey         = self.servo_y_client.call_async(yLocationRequest)
        futurez         = self.cscamZ_client_master.call_async(zlocationRequest)
        futureAngleZ    = self.rotating_servoZ_time_based_client.call_async(angleRequest)

        resultx = await futurex
        resulty = await futurey
        resultz = await futurez
        resultAngleZ = await futureAngleZ
        
        if (resulty.success and resultx.success and resultz.success and resultAngleZ.success):
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
    async def full_movement(self, x, y,z,anglez,anglex, time):
        xLocationRequest = ServoAbsNice.Request()
        xLocationRequest.position = x
        xLocationRequest.time = time
        
        yLocationRequest = ServoAbsNice.Request()
        yLocationRequest.position = y
        yLocationRequest.time = time

        zlocationRequest = MoveCscam.Request()
        zlocationRequest.motorid = 1
        zlocationRequest.position = z
        
        
        anglezRequest = RotatingServoTimeBased.Request()
        anglezRequest.angle = anglez
        anglezRequest.time = time

        anglexRequest = RotatingServoTimeBased.Request()
        anglexRequest.angle = anglex
        anglexRequest.time = time

        futurex         = self.servo_x_client.call_async(xLocationRequest)
        futurey         = self.servo_y_client.call_async(yLocationRequest)
        futurez         = self.cscamZ_client_master.call_async(zlocationRequest)
        futureAngleX    = self.rotating_servoX_time_based_client.call_async(anglexRequest)
        futureAngleZ    = self.rotating_servoZ_time_based_client.call_async(anglezRequest)

        resultx = await futurex
        resulty = await futurey
        resultz = await futurez
        resultAngleZ = await futureAngleZ
        resultAngleX = await futureAngleX
        
        if (resulty.success and resultx.success and resultz.success and resultAngleZ.success and resultAngleX.success):
            return True
        else:
            return False   

    async def z_and_lift(self,zposition, lifterid, liftPosition):
        if lifterid > 4:
            return False
        
        cscamMasterRequest = MoveCscam.Request()
        cscamMasterRequest.motorid = lifterid 
        cscamMasterRequest.position = liftPosition
        
        cscamMasterZRequest = MoveCscam.Request()
        cscamMasterZRequest.motorid = 1
        cscamMasterZRequest.position = zposition


        futureZ = self.cscamZ_client_master.call_async(cscamMasterZRequest)
        futureLift = self.cscam_client_master.call_async(cscamMasterRequest)

        resultZ = await futureZ
        resultLift = await futureLift

        if (resultZ.success and resultLift.success):
            return True
        else:
            return False 


        
        
        cscamMasterZResponse = self.cscamZ_client_master.call(cscamMasterZRequest)
        cscamMasterResponse = self.cscam_client_master.call(cscamMasterRequest)

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

        #send_http_request(http_commands[f'lifter_{head}_up'])
        
        time.sleep(5)
        
        self.get_logger().info(f"Closing gripper!!!")
        gripperResponse = self.gripper_close()
        if gripperResponse != True:
            self.get_logger().error("something failed when closing the gripper")
            return gripperResponse
        
        # time.sleep(5)

        #send_http_request(http_commands['z_up'])

        #send_http_request(http_commands[f'lifter_{head}_down'])
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
        #send_http_request(http_commands[f'z_clearance'])
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
        
        #send_http_request(http_commands[f'z_up'])
        time.sleep(6)


        self.get_logger().info(f"Approaching to head {head}")
        location_result = await self.cartesian_location(*locations[f"head{head}"], time=trajectory_time)
        if  location_result != True:
             self.get_logger().error(f"grab from head {head} to exit failed when approaching to head {head}")
             return location_result
        
        # send_http_request(http_commands['z_down'])
      
        # time.sleep(7)

        #send_http_request(http_commands[f'lifter_{head}_up'])
          
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

    async def grab_to_exit_final(self, head):
        
        self.get_logger().info(f"Approaching to head_{head}_back")
        location_result = await self.full_movement(*locations3D[f"head{head}_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching to head_{head}_back")
            return location_result

        self.get_logger().info(f"Approaching to head {head}")
        location_result = await self.cartesian_location(*locations[f"head{head}"], time=trajectory_time)
        if  location_result != True:
             self.get_logger().error(f"grab from head {head} to exit failed when approaching to head {head}")
             return location_result

        self.get_logger().info(f"Closing gripper!!!")
        gripperResponse = self.gripper_close()
        if gripperResponse != True:
            self.get_logger().error("something failed when closing the gripper")
            return gripperResponse
        input()
        self.get_logger().info("Moving Z to up Position and Lift to down position")
        zAndLiftResponse = self.z_and_lift(zposition=zlocation["up"], lifterId=head,liftPosition=lifter_location["lift_down"])
        if zAndLiftResponse != True:
            self.get_logger().error("Something failed when moving Z axis up and Lifter down")
            return zAndLiftResponse
        
        input()
        # self.get_logger().info("Moving Z to up position")
        # #input()
        # zResponse = self.z_master_position(position=zlocation["up"])
        # if zResponse !=True:
        #     self.get_logger().error("Something failed when moving z axis up")
        #     return zResponse
        
        # self.get_logger().info(f"Moving lifter {head} down")
        # #input()
        # lifterResponse = self.lifter_master_position(lifterid=head, position=lifter_location["lift_down"])
        # if lifterResponse != True:
        #     self.get_logger().error(f"something failed when getting lifter {head} to up position")
        #     return lifterResponse
        #time.sleep(0.5)
        #input()
        self.get_logger().info(f"Approaching to head_{head}_back")
        location_result = await self.cartesian_location(*locations[f"head{head}_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching to head_{head}_back")
            return location_result
        
        #input()
        #* Going back to reset position

        self.get_logger().info(f"Approaching to dropReady")
        location_result = await self.full_movement(*locations3D[f"dropReady"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"Approaching to drop ready failed")
            return location_result
        #input()
        self.get_logger().info(f"Approaching to drop Completed")
        location_result = await self.cartesian_rotation_location(*locations[f"dropCompleted"],angles["rotating_z_drop"],angles["rotating_x_drop"], time=1.0)
        if  location_result != True:
            self.get_logger().error(f"Approaching to drop Completed failed")
            return location_result
        #input()
        self.get_logger().info(f"Approaching to drop Completed")
        location_result = await self.cartesian_rotation_location(*locations[f"dropReady"],angles["rotating_z_drop"],angles["rotating_x_reset"], time=1.0)
        if  location_result != True:
            self.get_logger().error(f"Approaching to drop Completed failed")
            return location_result
        
        #input()

        self.get_logger().info(f"Approaching to head_{head}_back")
        location_result = await self.full_movement(*locations3D[f"head{head}_back_up"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching to head_{head}_back")
            return location_result
        #input()
        self.get_logger().info(f"Approaching to head {head}")
        location_result = await self.cartesian_location(*locations[f"head{head}"], time=trajectory_time)
        if  location_result != True:
             self.get_logger().error(f"grab from head {head} to exit failed when approaching to head {head}")
             return location_result
        #time.sleep(0.5)
        
        input()
        self.get_logger().info("Moving Z to down2 Position and Lift to up position")
        zAndLiftResponse = self.z_and_lift(zposition=zlocation["down2"], lifterId=head,liftPosition=lifter_location["lift_up"])
        if zAndLiftResponse != True:
            self.get_logger().error("Something failed when moving Z axis down2 and Lifter up")
            return zAndLiftResponse
        
        input()
        # self.get_logger().info(f"Moving lifter {head} up")
        # #input()
        # lifterResponse = self.lifter_master_position(lifterid=head, position=lifter_location["lift_up"])
        # if lifterResponse != True:
        #     self.get_logger().error(f"something failed when getting lifter {head} to up position")
        #     return lifterResponse
        
        # self.get_logger().info("Moving Z to down position")
        # #input()
        # zResponse = self.z_master_position(position=zlocation["down2"])
        # if zResponse !=True:
        #     self.get_logger().error("Something failed when moving z axis down")
        #     return zResponse
        
        
        #input()
        self.get_logger().info(f"Openning gripper!!!")
        gripperResponse = self.gripper_open()
        if gripperResponse != True:
            self.get_logger().error("something failed when openning the gripper")
            return gripperResponse
        #input()
        self.get_logger().info(f"Approaching to head {head}_back")
        location_result = await self.cartesian_location(*locations[f"head{head}_back"], time=trajectory_time)
        if  location_result != True:
             self.get_logger().error(f"grab from head {head} to exit failed when approaching to head {head} back")
             return location_result
        
        #input()
        self.get_logger().info(f"Approaching to reset")
        location_result = await self.full_movement(*locations3D["reset"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"going back to reset")
            return location_result

        
        
        
        
        
        # self.get_logger().info(f"Approaching to Exit")
        # location_result = await self.full_movement(*locations3D[f"Exit"], time=trajectory_time)
        # if  location_result != True:
        #     self.get_logger().error(f"grab from head {head} to exit failed when approaching to head_{head}_back")
        #     return location_result

        return True

 
   # Step 3: Create the clean_basket function
    async def grab_to_clean(self, head):
    """
    Function to grab a basket from a cooking position and move it to cleaning area
    Similar to grab_to_exit_final but with different disposal location
    """
    self.get_logger().info(f"Starting cleaning sequence for head {head}")
    
    # Step 1: Approach the cooking position
    self.get_logger().info(f"Approaching to head_{head}_back")
    location_result = await self.full_movement(*locations3D[f"head{head}_back"], time=trajectory_time)
    if location_result != True:
        self.get_logger().error(f"clean from head {head} failed when approaching to head_{head}_back")
        return location_result

    # Step 2: Move to exact position
    self.get_logger().info(f"Approaching to head {head}")
    location_result = await self.cartesian_location(*locations[f"head{head}"], time=trajectory_time)
    if location_result != True:
        self.get_logger().error(f"clean from head {head} failed when approaching to head {head}")
        return location_result
    
    # Step 3: Close gripper to grab basket
    self.get_logger().info(f"Closing gripper")
    gripperResponse = self.gripper_close()
    if gripperResponse != True:
        self.get_logger().error("something failed when closing the gripper")
        return gripperResponse
    
    # Step 4: Move Z-axis up
    self.get_logger().info("Moving Z to up position")
    zResponse = self.z_master_position(position=zlocation["up"])
    if zResponse != True:
        self.get_logger().error("Something failed when moving z axis up")
        return zResponse
    
    # Step 5: Move lifter down
    self.get_logger().info(f"Moving lifter {head} down")
    lifterResponse = self.lifter_master_position(lifterid=head, position=lifter_location["lift_down"])
    if lifterResponse != True:
        self.get_logger().error(f"something failed when moving lifter {head} down")
        return lifterResponse
    
    # Step 6: Back away from cooking position
    self.get_logger().info(f"Backing away to head_{head}_back")
    location_result = await self.cartesian_location(*locations[f"head{head}_back"], time=trajectory_time)
    if location_result != True:
        self.get_logger().error(f"clean from head {head} failed when backing away")
        return location_result
    
    # Step 7: Rotate Z to disposal angle
    self.get_logger().info(f"Rotating servoZ to disposal position")
    rotatingServoZResponse = self.rotate_servoZ(angle=angles["rotating_z_drop"]) 
    if rotatingServoZResponse != True:
        self.get_logger().error("Something failed when rotating the servoZ to disposal position")
        return rotatingServoZResponse
    
    # Step 8: Move to cleaning disposal area
    self.get_logger().info(f"Moving to cleaning disposal area")
    location_result = await self.cartesian_location(*locations["clean_disposal"], time=trajectory_time)
    if location_result != True:
        self.get_logger().error(f"clean from head {head} failed when moving to disposal area")
        return location_result
    
    # Step 9: Rotate X servo to cleaning angle
    self.get_logger().info(f"Rotating servoX for cleaning")
    rotatingServoXResponse = self.rotate_servoX(angle=angles["cleaning_x"]) 
    if rotatingServoXResponse != True:
        self.get_logger().error("Something failed when rotating servoX for cleaning")
        return rotatingServoXResponse
    
    # Step 10: Open gripper to release debris
    self.get_logger().info(f"Opening gripper to release debris")
    gripperResponse = self.gripper_open()
    if gripperResponse != True:
        self.get_logger().error("something failed when opening the gripper")
        return gripperResponse
    
    # Step 11: Close gripper to grab basket again
    self.get_logger().info(f"Closing gripper to grab basket again")
    gripperResponse = self.gripper_close()
    if gripperResponse != True:
        self.get_logger().error("something failed when closing the gripper")
        return gripperResponse
    
    # Step 12: Reset X and Z rotations
    self.get_logger().info(f"Resetting servo rotations")
    rotatingServoXResponse = self.rotate_servoX(angle=angles["rotating_x_reset"]) 
    if rotatingServoXResponse != True:
        self.get_logger().error("Something failed when resetting servoX rotation")
        return rotatingServoXResponse
    
    rotatingServoZResponse = self.rotate_servoZ(angle=angles["rotating_z_reset"]) 
    if rotatingServoZResponse != True:
        self.get_logger().error("Something failed when resetting servoZ rotation")
        return rotatingServoZResponse
    
    # Step 13: Return to cooking position
    self.get_logger().info(f"Returning to head_{head}_back")
    location_result = await self.cartesian_location(*locations[f"head{head}_back"], time=trajectory_time)
    if location_result != True:
        self.get_logger().error(f"clean from head {head} failed when returning to head_{head}_back")
        return location_result
    
    self.get_logger().info(f"Moving to head {head}")
    location_result = await self.cartesian_location(*locations[f"head{head}"], time=trajectory_time)
    if location_result != True:
        self.get_logger().error(f"clean from head {head} failed when moving to head {head}")
        return location_result

     # Step 14: Move lifter up
    self.get_logger().info(f"Moving lifter {head} up")
    lifterResponse = self.lifter_master_position(lifterid=head, position=lifter_location["lift_up"])
    if lifterResponse != True:
        self.get_logger().error(f"something failed when moving lifter {head} up")
        return lifterResponse
    
    # Step 15: Lower Z axis
    self.get_logger().info("Moving Z to down position")
    zResponse = self.z_master_position(position=zlocation["down"])
    if zResponse != True:
        self.get_logger().error("Something failed when moving z axis down")
        return zResponse
    
    # Step 16: Open gripper to release basket
    self.get_logger().info(f"Opening gripper to release basket")
    gripperResponse = self.gripper_open()
    if gripperResponse != True:
        self.get_logger().error("something failed when opening the gripper")
        return gripperResponse
    
    # Step 17: Back away
    self.get_logger().info(f"Backing away to head_{head}_back")
    location_result = await self.cartesian_location(*locations[f"head{head}_back"], time=trajectory_time)
    if location_result != True:
        self.get_logger().error(f"clean from head {head} failed when backing away")
        return location_result
    
    # Step 18: Return to reset position
    self.get_logger().info(f"Returning to reset position")
    location_result = await self.full_movement(*locations3D["reset"], time=trajectory_time)
    if location_result != True:
        self.get_logger().error(f"clean from head {head} failed when returning to reset")
        return location_result
    
    return True

    

    #* Services functions
    def dispense_noodle(self,request, response):
        pusherid = request.serving
        if (pusherid < 1) or (pusherid > 4):
            response.success = False
            response.message = "Please input id numbers between 1 and 4"
            return response
        
        cscamResponse = self.lifter_master_position(lifterid=pusherid, position=lifter_location["lift_up"])
        if not cscamResponse:
            response.success = False
            response.message = f"Error dispense noodle {pusherid}. Something failed while moving lifter up"
            return response
        
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
    
    def cook_noodle(self, request, response):
        cookid = request.serving
        if (cookid < 1) or (cookid > 4):
            response.success = False
            response.message = "Please input id numbers between 1 and 4"
            return response

        #TODO: Function to Boil Water
      
        if cookid < 5:
            start_time = time.time()  # Get the current time
            cscamResponse = self.lifter_master_position(lifterid=cookid, position=0.0)
            if not cscamResponse:
                response.success = False
                response.message = f"Error cook noodle {cookid}. Something failed while moving lifter shaking down"
                return response
            time.sleep(5)
            cscamResponse = self.lifter_master_position(lifterid=cookid, position=-25.0)
            if not cscamResponse:
                response.success = False
                response.message = f"Error cook noodle {cookid}. Something failed while moving lifter shaking up"
                return response
            
            while ((time.time() - start_time) < 40):  # Run for 30 seconds
                cscamResponse = self.lifter_master_position(lifterid=cookid, position=0.0)
                if not cscamResponse:
                    response.success = False
                    response.message = f"Error cook noodle {cookid}. Something failed while moving lifter shaking down"
                    return response
                time.sleep(1.5)
                cscamResponse = self.lifter_master_position(lifterid=cookid, position=-25.0)
                if not cscamResponse:
                    response.success = False
                    response.message = f"Error cook noodle {cookid}. Something failed while moving lifter shaking up"
                    return response

            cscamResponse = self.lifter_master_position(lifterid=cookid, position=lifter_location["lift_up"])
            if not cscamResponse:
                response.success = False
                response.message = f"Error cook noodle {cookid}. Something failed while moving lifter up"
                return response
    
        #TODO: Function to turn off boiler

        response.success = True
        response.message = f"Cook noodle {cookid} success"
        return response

    async def serve_noodle(self, request, response):
        servingid = request.serving
        if (servingid < 1) or (servingid > 4):
            response.success = False
            response.message = "Please input id numbers between 1 and 4"
            return response
        
        responseGrabBasket = await self.grab_to_exit_final(head=servingid)
        if not responseGrabBasket:
                response.success = False
                response.message = f"Error serve noodle {servingid}. Something failed while serving"
                return response
        
        response.success = True
        response.message = f"Serve noodle {servingid} success"
        return response
        
    async def clean_noodle(self, request, response):
    """
    Service function for cleaning a cooking station
    """
    cleanid = request.serving
    if (cleanid < 1) or (cleanid > 4):
        response.success = False
        response.message = "Please input id numbers between 1 and 4"
        return response
    
    responseCleanBasket = await self.grab_to_clean(head=cleanid)
    if not responseCleanBasket:
        response.success = False
        response.message = f"Error cleaning station {cleanid}. Something failed during cleaning"
        return response
    
    response.success = True
    response.message = f"Clean station {cleanid} successful"
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
        
